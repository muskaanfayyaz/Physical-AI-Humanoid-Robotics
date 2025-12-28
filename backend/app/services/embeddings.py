"""
Google Gemini embeddings generation service.
Handles text-to-vector conversion using Gemini's embedding models.
Uses FREE tier: 15 requests/minute, 1500 requests/day.
"""

import asyncio
from typing import List
import google.generativeai as genai
import logging

from app.config import get_settings

logger = logging.getLogger(__name__)


class EmbeddingService:
    """
    Service for generating text embeddings using Google Gemini API.

    Features:
    - 768-dimensional embeddings (vs OpenAI's 1536)
    - FREE tier with generous limits
    - Optimized for retrieval tasks
    """

    def __init__(self):
        """
        Initialize Gemini embedding service.
        Configures API key and model settings.
        """
        settings = get_settings()

        # Configure Gemini API with your API key
        genai.configure(api_key=settings.gemini_api_key)

        self.model_name = settings.gemini_embedding_model  # "models/embedding-001"
        self.dimensions = settings.embedding_dimensions  # 768 for Gemini

        logger.info(f"Initialized Gemini Embedding Service: {self.model_name}")

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string using Gemini.

        Args:
            text: Input text to embed (max 2048 tokens)

        Returns:
            List of 768 floats representing the embedding vector

        Raises:
            Exception: If Gemini API call fails
        """
        try:
            # Use task_type="retrieval_document" for indexing documents
            # Use task_type="retrieval_query" for search queries
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="retrieval_document",  # Optimized for document storage
            )

            # Gemini returns embedding as a list directly
            embedding = result['embedding']

            # Verify dimensions
            if len(embedding) != self.dimensions:
                logger.warning(f"Unexpected embedding dimension: {len(embedding)} (expected {self.dimensions})")

            return embedding

        except Exception as e:
            logger.error(f"Failed to generate Gemini embedding: {e}")
            raise

    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 100
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts with batching.

        Note: Gemini API processes requests sequentially to respect rate limits.
        FREE tier: 15 requests/minute (1 request every 4 seconds).

        Args:
            texts: List of input texts
            batch_size: Number of texts to process per batch (recommended: 100)

        Returns:
            List of 768-dimensional embedding vectors

        Raises:
            Exception: If Gemini API call fails
        """
        embeddings = []

        # Process in batches to manage rate limits and provide progress feedback
        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]

            try:
                # Process each text in the batch
                # Note: Gemini's batch embedding is done via embed_content with list input
                batch_embeddings = []

                for text in batch:
                    # Generate embedding for each text
                    result = genai.embed_content(
                        model=self.model_name,
                        content=text,
                        task_type="retrieval_document",
                    )
                    batch_embeddings.append(result['embedding'])

                embeddings.extend(batch_embeddings)

                logger.info(f"Processed batch {i // batch_size + 1}: {len(batch)} texts")

                # Delay to respect rate limits (FREE tier: 15 req/min = 1 req every 4s)
                # For batch processing, add a small delay every 10 requests
                if i + batch_size < len(texts) and len(batch_embeddings) >= 10:
                    await asyncio.sleep(5)  # 5 second delay every 10 requests
                elif i + batch_size < len(texts):
                    await asyncio.sleep(0.5)  # Small delay between batches

            except Exception as e:
                logger.error(f"Failed to process batch {i // batch_size + 1}: {e}")
                raise

        return embeddings

    async def test_connection(self) -> bool:
        """
        Test Gemini API connection by generating a test embedding.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            await self.generate_embedding("test connection")
            logger.info("Gemini API connection test: SUCCESS")
            return True
        except Exception as e:
            logger.error(f"Gemini API connection test failed: {e}")
            return False


# Singleton instance
_embedding_service: EmbeddingService | None = None


def get_embedding_service() -> EmbeddingService:
    """Get or create the embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
