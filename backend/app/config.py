"""
Configuration management using Pydantic Settings.
Loads environment variables and provides type-safe configuration.
"""

from functools import lru_cache
from typing import Optional
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "Physical AI RAG Backend"
    app_version: str = "1.0.0"
    debug: bool = False

    # Google Gemini API (FREE tier)
    # Get your API key at: https://aistudio.google.com/app/apikey
    gemini_api_key: str
    gemini_embedding_model: str = "models/text-embedding-004"  # Gemini embedding model
    gemini_chat_model: str = "gemini-2.0-flash-lite"  # Gemini chat model (FREE, fast)
    embedding_dimensions: int = 768  # Gemini embeddings are 768-dimensional

    # Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "physical_ai_textbook"

    # Neon Postgres
    postgres_url: str
    postgres_pool_size: int = 10
    postgres_max_overflow: int = 20

    # API Configuration
    cors_origins: list[str] = ["http://localhost:3000", "https://muskaanfayyaz.github.io"]
    api_prefix: str = "/api/v1"

    # RAG Configuration
    chunks_file_path: str = "chunks.json"  # Changed to backend directory for Render deployment
    top_k_results: int = 5
    similarity_threshold: float = 0.7

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )


@lru_cache()
def get_settings() -> Settings:
    """
    Get cached settings instance.
    Uses lru_cache to instantiate settings only once.
    """
    return Settings()
