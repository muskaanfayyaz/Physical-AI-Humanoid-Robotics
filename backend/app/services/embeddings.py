"""
OpenAI embeddings generation service.
Handles text-to-vector conversion using OpenAI's embedding models.
"""

import asyncio
from typing import List
from openai import AsyncOpenAI
import logging

from app.config import get_settings

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for generating text embeddings using OpenAI."""

    def __init__(self):
        settings = get_settings()
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_embedding_model
        self.dimensions = settings.embedding_dimensions

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector

        Raises:
            Exception: If OpenAI API call fails
        """
        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=text,
                dimensions=self.dimensions if "text-embedding-3" in self.model else None,
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            raise

    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 100
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts with batching.

        Args:
            texts: List of input texts
            batch_size: Number of texts to process per batch (max 2048 for OpenAI)

        Returns:
            List of embedding vectors

        Raises:
            Exception: If OpenAI API call fails
        """
        embeddings = []

        # Process in batches to avoid rate limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]

            try:
                response = await self.client.embeddings.create(
                    model=self.model,
                    input=batch,
                    dimensions=self.dimensions if "text-embedding-3" in self.model else None,
                )

                # Extract embeddings in the same order
                batch_embeddings = [item.embedding for item in response.data]
                embeddings.extend(batch_embeddings)

                logger.info(f"Processed batch {i // batch_size + 1}: {len(batch)} texts")

                # Small delay to avoid rate limiting
                if i + batch_size < len(texts):
                    await asyncio.sleep(0.1)

            except Exception as e:
                logger.error(f"Failed to process batch {i // batch_size + 1}: {e}")
                raise

        return embeddings

    async def test_connection(self) -> bool:
        """
        Test OpenAI API connection.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            await self.generate_embedding("test")
            return True
        except Exception as e:
            logger.error(f"OpenAI connection test failed: {e}")
            return False


# Singleton instance
_embedding_service: EmbeddingService | None = None


def get_embedding_service() -> EmbeddingService:
    """Get or create the embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
