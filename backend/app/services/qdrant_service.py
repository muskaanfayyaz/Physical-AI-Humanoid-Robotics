"""
Qdrant Cloud vector database service.
Handles vector storage, retrieval, and similarity search.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient, AsyncQdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    Range,
    SearchParams,
)
import logging

from app.config import get_settings
from app.schemas import SearchFilters

logger = logging.getLogger(__name__)


class QdrantService:
    """Service for interacting with Qdrant Cloud vector database."""

    def __init__(self):
        settings = get_settings()
        self.client = AsyncQdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=30.0,
        )
        self.collection_name = settings.qdrant_collection_name
        self.vector_size = settings.embedding_dimensions

    async def create_collection(self) -> bool:
        """
        Create collection if it doesn't exist.

        Returns:
            True if collection created or already exists
        """
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create collection with cosine distance
            await self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE,
                ),
            )

            logger.info(f"Created collection '{self.collection_name}'")
            return True

        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            raise

    async def upsert_points(
        self, chunk_ids: List[str], embeddings: List[List[float]], payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        Insert or update points in the collection.

        Args:
            chunk_ids: List of unique chunk identifiers
            embeddings: List of embedding vectors
            payloads: List of metadata dictionaries

        Returns:
            True if successful

        Raises:
            Exception: If upsert operation fails
        """
        try:
            points = [
                PointStruct(
                    id=i,  # Qdrant uses numeric IDs
                    vector=embedding,
                    payload={
                        "chunk_id": chunk_id,
                        **payload,
                    },
                )
                for i, (chunk_id, embedding, payload) in enumerate(
                    zip(chunk_ids, embeddings, payloads)
                )
            ]

            await self.client.upsert(
                collection_name=self.collection_name,
                points=points,
                wait=True,
            )

            logger.info(f"Upserted {len(points)} points to Qdrant")
            return True

        except Exception as e:
            logger.error(f"Failed to upsert points: {e}")
            raise

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        filters: Optional[SearchFilters] = None,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors.

        Args:
            query_vector: Query embedding vector
            top_k: Number of results to return
            filters: Optional filters for metadata
            score_threshold: Minimum similarity score

        Returns:
            List of search results with payload and score
        """
        try:
            # Build filter conditions
            filter_conditions = []

            if filters:
                if filters.chapter_type:
                    filter_conditions.append(
                        FieldCondition(
                            key="chapter_type",
                            match=MatchValue(value=filters.chapter_type),
                        )
                    )

                if filters.chapter_number is not None:
                    filter_conditions.append(
                        FieldCondition(
                            key="chapter_number",
                            match=MatchValue(value=filters.chapter_number),
                        )
                    )

                if filters.section_level is not None:
                    filter_conditions.append(
                        FieldCondition(
                            key="section_level",
                            match=MatchValue(value=filters.section_level),
                        )
                    )

                if filters.min_token_count is not None or filters.max_token_count is not None:
                    filter_conditions.append(
                        FieldCondition(
                            key="token_count",
                            range=Range(
                                gte=filters.min_token_count,
                                lte=filters.max_token_count,
                            ),
                        )
                    )

            # Construct filter
            search_filter = Filter(must=filter_conditions) if filter_conditions else None

            # Perform search using AsyncQdrantClient
            # Try different method names for compatibility across versions
            try:
                # Try new API (qdrant-client 1.9+)
                results = await self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    query_filter=search_filter,
                    score_threshold=score_threshold,
                    with_payload=True,
                )
            except (AttributeError, TypeError):
                # Fallback to alternative method
                results = await self.client.query(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    query_filter=search_filter,
                    score_threshold=score_threshold,
                    with_payload=True,
                )

            # Format results (handle both list and object with .points)
            formatted_results = []
            result_points = results.points if hasattr(results, 'points') else results
            for result in result_points:
                formatted_results.append(
                    {
                        "chunk_id": result.payload.get("chunk_id"),
                        "score": result.score,
                        "payload": result.payload,
                    }
                )

            logger.info(f"Found {len(formatted_results)} results for search")
            return formatted_results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    async def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection stats
        """
        try:
            info = await self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": info.config.params.vectors.size if hasattr(info.config.params, 'vectors') else None,
                "points_count": info.points_count,
                "vectors_count": info.vectors_count if hasattr(info, 'vectors_count') else info.points_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            return {}

    async def test_connection(self) -> bool:
        """
        Test Qdrant connection.

        Returns:
            True if connection successful
        """
        try:
            await self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant connection test failed: {e}")
            return False

    async def delete_collection(self) -> bool:
        """
        Delete the collection (use with caution).

        Returns:
            True if successful
        """
        try:
            await self.client.delete_collection(collection_name=self.collection_name)
            logger.warning(f"Deleted collection '{self.collection_name}'")
            return True
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            return False


# Singleton instance
_qdrant_service: QdrantService | None = None


def get_qdrant_service() -> QdrantService:
    """Get or create the Qdrant service singleton."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
