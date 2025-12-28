"""
Database connection and session management for Neon Postgres.
Uses SQLAlchemy with async support.
"""

from contextlib import asynccontextmanager
from typing import AsyncGenerator
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    create_async_engine,
    async_sessionmaker,
)
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy import Column, Integer, String, Text, DateTime, JSON, Float
from datetime import datetime

from app.config import get_settings


class Base(DeclarativeBase):
    """Base class for all database models."""
    pass


class ChunkMetadata(Base):
    """
    Stores metadata for each chunk in Postgres.
    The actual vectors are stored in Qdrant.
    """
    __tablename__ = "chunk_metadata"

    id = Column(Integer, primary_key=True, index=True)
    chunk_id = Column(String(255), unique=True, nullable=False, index=True)

    # Chapter information
    chapter_type = Column(String(50), nullable=False, index=True)
    chapter_number = Column(Integer, nullable=False, index=True)
    chapter_title_slug = Column(String(255), nullable=False)
    filename = Column(String(255), nullable=False)

    # Section information
    section_level = Column(Integer, nullable=False)
    section_title = Column(String(500), nullable=False)
    section_path = Column(JSON, nullable=False)
    heading_hierarchy = Column(Text, nullable=False)

    # Part information (for split sections)
    part_number = Column(Integer, nullable=False)
    total_parts = Column(Integer, nullable=False)

    # Content metrics
    token_count = Column(Integer, nullable=False)
    char_count = Column(Integer, nullable=False)

    # Full content (for reference)
    content = Column(Text, nullable=False)

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    def __repr__(self) -> str:
        return f"<ChunkMetadata(chunk_id={self.chunk_id})>"


class SearchQuery(Base):
    """Stores search query history for analytics."""
    __tablename__ = "search_queries"

    id = Column(Integer, primary_key=True, index=True)
    query_text = Column(Text, nullable=False)
    top_k = Column(Integer, nullable=False)
    results_count = Column(Integer, nullable=False)
    avg_similarity_score = Column(Float, nullable=True)
    filters = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    def __repr__(self) -> str:
        return f"<SearchQuery(id={self.id}, query={self.query_text[:50]})>"


class ConversationHistory(Base):
    """Stores conversation history for context."""
    __tablename__ = "conversation_history"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String(255), nullable=False, index=True)
    role = Column(String(50), nullable=False)  # 'user' or 'assistant'
    message = Column(Text, nullable=False)
    chunk_references = Column(JSON, nullable=True)  # List of chunk_ids used
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    def __repr__(self) -> str:
        return f"<ConversationHistory(session_id={self.session_id}, role={self.role})>"


# Database engine and session management
engine: AsyncEngine | None = None
async_session_maker: async_sessionmaker[AsyncSession] | None = None


async def init_db() -> None:
    """Initialize database connection and create tables."""
    global engine, async_session_maker

    settings = get_settings()

    # Convert postgresql:// to postgresql+asyncpg://
    db_url = settings.postgres_url.replace("postgresql://", "postgresql+asyncpg://")

    # Python 3.13 compatibility: Remove sslmode parameter that causes channel_binding error
    # Neon uses sslmode=require by default which triggers channel_binding in Python 3.13
    if "sslmode=" in db_url:
        # Remove sslmode parameter from URL
        import re
        db_url = re.sub(r'[?&]sslmode=[^&]*', '', db_url)
        db_url = re.sub(r'\?&', '?', db_url)  # Fix query string if needed

    # Add ssl=true as query parameter instead (asyncpg compatible)
    if "?" in db_url:
        db_url += "&ssl=true"
    else:
        db_url += "?ssl=true"

    # Connection arguments for asyncpg (Python 3.13 compatibility)
    connect_args = {
        "ssl": "require",  # Use string instead of SSLContext to avoid channel_binding
        "server_settings": {
            "jit": "off",
        },
    }

    engine = create_async_engine(
        db_url,
        echo=settings.debug,
        pool_size=settings.postgres_pool_size,
        max_overflow=settings.postgres_max_overflow,
        pool_pre_ping=True,
        connect_args=connect_args,
    )

    async_session_maker = async_sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )

    # Create tables
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def close_db() -> None:
    """Close database connection."""
    global engine
    if engine:
        await engine.dispose()


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency for getting database sessions.
    Use with FastAPI's Depends().
    """
    if async_session_maker is None:
        raise RuntimeError("Database not initialized. Call init_db() first.")

    async with async_session_maker() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()
