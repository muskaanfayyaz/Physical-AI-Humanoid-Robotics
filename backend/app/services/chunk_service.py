"""
Chunk processing service.
Coordinates reading chunks, generating embeddings, and storing in databases.
"""

import json
import logging
from pathlib import Path
from typing import List, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, func

from app.schemas import ChunkSchema, ChunkMetadataSchema
from app.database import ChunkMetadata, SearchQuery
from app.services.embeddings import get_embedding_service
from app.services.qdrant_service import get_qdrant_service
from app.config import get_settings

logger = logging.getLogger(__name__)


class ChunkService:
    """Service for processing and storing chunks."""

    def __init__(self):
        self.settings = get_settings()
        self.embedding_service = get_embedding_service()
        self.qdrant_service = get_qdrant_service()

    def load_chunks_from_file(self) -> List[ChunkSchema]:
        """
        Load chunks from chunks.json file.

        Returns:
            List of validated chunk schemas

        Raises:
            FileNotFoundError: If chunks.json doesn't exist
            ValueError: If JSON is invalid
        """
        chunks_path = Path(self.settings.chunks_file_path)

        if not chunks_path.exists():
            raise FileNotFoundError(f"Chunks file not found: {chunks_path}")

        try:
            with open(chunks_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            chunks = [ChunkSchema(**chunk) for chunk in data["chunks"]]
            logger.info(f"Loaded {len(chunks)} chunks from {chunks_path}")
            return chunks

        except Exception as e:
            logger.error(f"Failed to load chunks: {e}")
            raise ValueError(f"Invalid chunks.json format: {e}")

    async def store_chunk_metadata(
        self, session: AsyncSession, chunk: ChunkSchema
    ) -> ChunkMetadata:
        """
        Store chunk metadata in Postgres.

        Args:
            session: Database session
            chunk: Chunk schema to store

        Returns:
            Stored ChunkMetadata instance
        """
        metadata = ChunkMetadata(
            chunk_id=chunk.chunk_id,
            chapter_type=chunk.metadata.chapter_type,
            chapter_number=chunk.metadata.chapter_number,
            chapter_title_slug=chunk.metadata.chapter_title_slug,
            filename=chunk.metadata.filename,
            section_level=chunk.metadata.section_level,
            section_title=chunk.metadata.section_title,
            section_path=chunk.metadata.section_path,
            heading_hierarchy=chunk.metadata.heading_hierarchy,
            part_number=chunk.metadata.part_number,
            total_parts=chunk.metadata.total_parts,
            token_count=chunk.metadata.token_count,
            char_count=chunk.metadata.char_count,
            content=chunk.content,
        )

        session.add(metadata)
        return metadata

    async def ingest_chunks(self, session: AsyncSession) -> Dict[str, Any]:
        """
        Complete ingestion pipeline:
        1. Load chunks from file
        2. Generate embeddings
        3. Store in Postgres
        4. Store in Qdrant

        Args:
            session: Database session

        Returns:
            Dictionary with ingestion statistics
        """
        import time

        start_time = time.time()
        errors = []

        try:
            # Step 1: Load chunks
            logger.info("Loading chunks from file...")
            chunks = self.load_chunks_from_file()

            # Step 2: Ensure Qdrant collection exists
            logger.info("Creating Qdrant collection if needed...")
            await self.qdrant_service.create_collection()

            # Step 3: Generate embeddings
            logger.info("Generating embeddings...")
            texts = [chunk.content for chunk in chunks]
            embeddings = await self.embedding_service.generate_embeddings_batch(texts)

            # Step 4: Store in Postgres
            logger.info("Storing metadata in Postgres...")
            postgres_count = 0
            for chunk in chunks:
                try:
                    await self.store_chunk_metadata(session, chunk)
                    postgres_count += 1
                except Exception as e:
                    error_msg = f"Failed to store {chunk.chunk_id} in Postgres: {e}"
                    logger.error(error_msg)
                    errors.append(error_msg)

            await session.commit()

            # Step 5: Store in Qdrant
            logger.info("Storing vectors in Qdrant...")
            chunk_ids = [chunk.chunk_id for chunk in chunks]
            payloads = [
                {
                    "content": chunk.content,
                    "chapter_type": chunk.metadata.chapter_type,
                    "chapter_number": chunk.metadata.chapter_number,
                    "chapter_title_slug": chunk.metadata.chapter_title_slug,
                    "section_title": chunk.metadata.section_title,
                    "heading_hierarchy": chunk.metadata.heading_hierarchy,
                    "section_level": chunk.metadata.section_level,
                    "token_count": chunk.metadata.token_count,
                    "char_count": chunk.metadata.char_count,
                }
                for chunk in chunks
            ]

            await self.qdrant_service.upsert_points(chunk_ids, embeddings, payloads)
            qdrant_count = len(chunks)

            processing_time = time.time() - start_time

            return {
                "success": True,
                "chunks_processed": len(chunks),
                "chunks_stored_postgres": postgres_count,
                "chunks_stored_qdrant": qdrant_count,
                "errors": errors,
                "processing_time_seconds": round(processing_time, 2),
            }

        except Exception as e:
            logger.error(f"Ingestion failed: {e}")
            errors.append(str(e))
            return {
                "success": False,
                "chunks_processed": 0,
                "chunks_stored_postgres": 0,
                "chunks_stored_qdrant": 0,
                "errors": errors,
                "processing_time_seconds": round(time.time() - start_time, 2),
            }

    async def get_chunk_by_id(
        self, session: AsyncSession, chunk_id: str
    ) -> ChunkMetadata | None:
        """
        Retrieve chunk metadata by ID.

        Args:
            session: Database session
            chunk_id: Chunk identifier

        Returns:
            ChunkMetadata if found, None otherwise
        """
        result = await session.execute(
            select(ChunkMetadata).where(ChunkMetadata.chunk_id == chunk_id)
        )
        return result.scalar_one_or_none()

    async def get_stats(self, session: AsyncSession) -> Dict[str, Any]:
        """
        Get database statistics.

        Args:
            session: Database session

        Returns:
            Dictionary with statistics
        """
        # Count chunks
        chunk_count = await session.scalar(select(func.count(ChunkMetadata.id)))

        # Count queries
        query_count = await session.scalar(select(func.count(SearchQuery.id)))

        # Average results per query
        avg_results = await session.scalar(
            select(func.avg(SearchQuery.results_count))
        ) or 0.0

        # Qdrant stats
        collection_info = await self.qdrant_service.get_collection_info()

        return {
            "total_chunks": chunk_count or 0,
            "total_queries": query_count or 0,
            "avg_query_results": round(float(avg_results), 2),
            "collection_info": collection_info,
        }


def get_chunk_service() -> ChunkService:
    """Get chunk service instance."""
    return ChunkService()
