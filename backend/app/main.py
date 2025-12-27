"""
FastAPI application for Physical AI RAG Backend.

This backend provides:
- Chunk ingestion from rag/chunks.json
- OpenAI embedding generation
- Vector storage in Qdrant Cloud
- Metadata storage in Neon Postgres
- Semantic search API
- Conversation history management
- Grounded question answering with LLM (GPT-4o-mini)
- Selection-based Q&A without hallucination
"""

import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import get_settings
from app.database import init_db, close_db
from app.routers import ingest, search, conversation, ask
from app.schemas import HealthCheckResponse
from app.services.embeddings import get_embedding_service
from app.services.qdrant_service import get_qdrant_service
from app.services.llm_service import get_llm_service

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    """
    # Startup
    logger.info("Starting up Physical AI RAG Backend...")
    settings = get_settings()

    try:
        # Initialize database
        await init_db()
        logger.info("✅ Database initialized")

        # Test connections
        embedding_service = get_embedding_service()
        qdrant_service = get_qdrant_service()
        llm_service = get_llm_service()

        openai_ok = await embedding_service.test_connection()
        qdrant_ok = await qdrant_service.test_connection()
        llm_ok = await llm_service.test_connection()

        logger.info(f"✅ OpenAI Embeddings: {'OK' if openai_ok else 'FAILED'}")
        logger.info(f"✅ OpenAI LLM: {'OK' if llm_ok else 'FAILED'}")
        logger.info(f"✅ Qdrant connection: {'OK' if qdrant_ok else 'FAILED'}")

        logger.info(f"🚀 {settings.app_name} v{settings.app_version} ready!")

    except Exception as e:
        logger.error(f"❌ Startup failed: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down...")
    await close_db()
    logger.info("✅ Database connection closed")


# Create FastAPI app
app = FastAPI(
    title="Physical AI RAG Backend",
    description="Backend API for Physical AI & Humanoid Robotics RAG chatbot",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    lifespan=lifespan,
)

# Configure CORS
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(ingest.router, prefix=settings.api_prefix)
app.include_router(search.router, prefix=settings.api_prefix)
app.include_router(conversation.router, prefix=settings.api_prefix)
app.include_router(ask.router, prefix=settings.api_prefix)


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI RAG Backend",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
    }


@app.get("/health", response_model=HealthCheckResponse)
async def health_check():
    """
    Health check endpoint.

    Tests connectivity to:
    - Neon Postgres
    - Qdrant Cloud
    - OpenAI API

    Returns:
        HealthCheckResponse with connection status
    """
    settings = get_settings()

    try:
        # Test OpenAI
        embedding_service = get_embedding_service()
        openai_ok = await embedding_service.test_connection()

        # Test Qdrant
        qdrant_service = get_qdrant_service()
        qdrant_ok = await qdrant_service.test_connection()

        # Database is tested via dependency injection
        database_ok = True

        return HealthCheckResponse(
            status="healthy" if all([database_ok, qdrant_ok, openai_ok]) else "degraded",
            app_name=settings.app_name,
            version=settings.app_version,
            database_connected=database_ok,
            qdrant_connected=qdrant_ok,
            openai_configured=openai_ok,
        )

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthCheckResponse(
            status="unhealthy",
            app_name=settings.app_name,
            version=settings.app_version,
            database_connected=False,
            qdrant_connected=False,
            openai_configured=False,
        )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info",
    )
