"""
Search endpoints for querying the RAG system.
"""

import time
from typing import List
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import get_db, ChunkMetadata, SearchQuery
from app.schemas import (
    SearchRequest,
    SearchResponse,
    SearchResultItem,
    ChunkMetadataSchema,
)
from app.services.embeddings import get_embedding_service
from app.services.qdrant_service import get_qdrant_service
from sqlalchemy import select

router = APIRouter(prefix="/search", tags=["Search"])


@router.post("/", response_model=SearchResponse)
async def search_chunks(
    request: SearchRequest,
    session: AsyncSession = Depends(get_db),
    embedding_service = Depends(get_embedding_service),
    qdrant_service = Depends(get_qdrant_service),
):
    """
    Search for relevant chunks using semantic similarity.

    This endpoint:
    1. Generates embedding for the query text
    2. Searches Qdrant for similar vectors
    3. Retrieves full metadata from Postgres
    4. Logs the query for analytics

    Args:
        request: SearchRequest with query text, top_k, filters, and threshold

    Returns:
        SearchResponse with matching chunks and metadata
    """
    start_time = time.time()

    try:
        # Step 1: Generate query embedding
        query_vector = await embedding_service.generate_embedding(request.query)

        # Step 2: Search Qdrant
        qdrant_results = await qdrant_service.search(
            query_vector=query_vector,
            top_k=request.top_k,
            filters=request.filters,
            score_threshold=request.similarity_threshold,
        )

        # Step 3: Retrieve full metadata from Postgres
        chunk_ids = [r["chunk_id"] for r in qdrant_results]

        if not chunk_ids:
            # No results found
            processing_time = time.time() - start_time
            return SearchResponse(
                query=request.query,
                results=[],
                total_results=0,
                processing_time_seconds=round(processing_time, 2),
            )

        # Query Postgres for full metadata
        result = await session.execute(
            select(ChunkMetadata).where(ChunkMetadata.chunk_id.in_(chunk_ids))
        )
        chunks = result.scalars().all()

        # Create a mapping for quick lookup
        chunk_map = {chunk.chunk_id: chunk for chunk in chunks}

        # Build results maintaining Qdrant's order and scores
        search_results: List[SearchResultItem] = []
        for qdrant_result in qdrant_results:
            chunk_id = qdrant_result["chunk_id"]
            chunk = chunk_map.get(chunk_id)

            if chunk:
                metadata = ChunkMetadataSchema(
                    chapter_type=chunk.chapter_type,
                    chapter_number=chunk.chapter_number,
                    chapter_title_slug=chunk.chapter_title_slug,
                    filename=chunk.filename,
                    section_level=chunk.section_level,
                    section_title=chunk.section_title,
                    section_path=chunk.section_path,
                    heading_hierarchy=chunk.heading_hierarchy,
                    part_number=chunk.part_number,
                    total_parts=chunk.total_parts,
                    token_count=chunk.token_count,
                    char_count=chunk.char_count,
                )

                search_results.append(
                    SearchResultItem(
                        chunk_id=chunk.chunk_id,
                        content=chunk.content,
                        score=qdrant_result["score"],
                        metadata=metadata,
                    )
                )

        # Step 4: Log the query
        avg_score = (
            sum(r.score for r in search_results) / len(search_results)
            if search_results
            else 0.0
        )

        search_query = SearchQuery(
            query_text=request.query,
            top_k=request.top_k,
            results_count=len(search_results),
            avg_similarity_score=avg_score,
            filters=request.filters.model_dump() if request.filters else None,
        )
        session.add(search_query)
        await session.commit()

        processing_time = time.time() - start_time

        return SearchResponse(
            query=request.query,
            results=search_results,
            total_results=len(search_results),
            processing_time_seconds=round(processing_time, 2),
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")


@router.get("/chunk/{chunk_id}", response_model=SearchResultItem)
async def get_chunk_by_id(
    chunk_id: str,
    session: AsyncSession = Depends(get_db),
):
    """
    Retrieve a specific chunk by its ID.

    Args:
        chunk_id: Unique chunk identifier

    Returns:
        SearchResultItem with full chunk data
    """
    try:
        result = await session.execute(
            select(ChunkMetadata).where(ChunkMetadata.chunk_id == chunk_id)
        )
        chunk = result.scalar_one_or_none()

        if not chunk:
            raise HTTPException(status_code=404, detail=f"Chunk '{chunk_id}' not found")

        metadata = ChunkMetadataSchema(
            chapter_type=chunk.chapter_type,
            chapter_number=chunk.chapter_number,
            chapter_title_slug=chunk.chapter_title_slug,
            filename=chunk.filename,
            section_level=chunk.section_level,
            section_title=chunk.section_title,
            section_path=chunk.section_path,
            heading_hierarchy=chunk.heading_hierarchy,
            part_number=chunk.part_number,
            total_parts=chunk.total_parts,
            token_count=chunk.token_count,
            char_count=chunk.char_count,
        )

        return SearchResultItem(
            chunk_id=chunk.chunk_id,
            content=chunk.content,
            score=1.0,  # Perfect match for direct ID lookup
            metadata=metadata,
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve chunk: {str(e)}")
