"""
Ingestion endpoints for loading chunks into the system.
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import get_db
from app.schemas import ChunkIngestionResponse, StatsResponse
from app.services.chunk_service import get_chunk_service

router = APIRouter(prefix="/ingest", tags=["Ingestion"])


@router.post("/chunks", response_model=ChunkIngestionResponse)
async def ingest_chunks(
    session: AsyncSession = Depends(get_db),
    chunk_service: get_chunk_service = Depends(get_chunk_service),
):
    """
    Ingest all chunks from chunks.json into the system.

    This endpoint:
    1. Loads chunks from rag/chunks.json
    2. Generates embeddings using OpenAI
    3. Stores metadata in Neon Postgres
    4. Stores vectors in Qdrant Cloud

    **Note:** This operation may take several minutes depending on the number of chunks.

    Returns:
        ChunkIngestionResponse with processing statistics and any errors
    """
    try:
        result = await chunk_service.ingest_chunks(session)
        return ChunkIngestionResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


@router.get("/stats", response_model=StatsResponse)
async def get_ingestion_stats(
    session: AsyncSession = Depends(get_db),
    chunk_service: get_chunk_service = Depends(get_chunk_service),
):
    """
    Get statistics about ingested chunks and system status.

    Returns:
        StatsResponse with counts and collection information
    """
    try:
        stats = await chunk_service.get_stats(session)
        return StatsResponse(
            total_chunks=stats["total_chunks"],
            total_queries=stats["total_queries"],
            total_conversations=0,  # TODO: Add conversation count
            avg_query_results=stats["avg_query_results"],
            collection_info=stats["collection_info"],
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get stats: {str(e)}")
