"""
Pydantic schemas for request/response validation.
"""

from datetime import datetime
from typing import Any, Optional
from pydantic import BaseModel, Field, ConfigDict


class ChunkMetadataSchema(BaseModel):
    """Schema for chunk metadata."""
    chapter_type: str
    chapter_number: int
    chapter_title_slug: str
    filename: str
    section_level: int
    section_title: str
    section_path: list[str]
    heading_hierarchy: str
    part_number: int
    total_parts: int
    token_count: int
    char_count: int


class ChunkSchema(BaseModel):
    """Schema for a single chunk from chunks.json."""
    chunk_id: str
    content: str
    metadata: ChunkMetadataSchema


class ChunkIngestionResponse(BaseModel):
    """Response for chunk ingestion."""
    success: bool
    chunks_processed: int
    chunks_stored_postgres: int
    chunks_stored_qdrant: int
    errors: list[str] = []
    processing_time_seconds: float


class SearchFilters(BaseModel):
    """Filters for search queries."""
    chapter_type: Optional[str] = None
    chapter_number: Optional[int] = None
    section_level: Optional[int] = None
    min_token_count: Optional[int] = None
    max_token_count: Optional[int] = None


class SearchRequest(BaseModel):
    """Request schema for search endpoint."""
    query: str = Field(..., min_length=1, max_length=1000, description="Search query text")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of results to return")
    filters: Optional[SearchFilters] = None
    similarity_threshold: Optional[float] = Field(default=0.7, ge=0.0, le=1.0)


class SearchResultItem(BaseModel):
    """Single search result item."""
    chunk_id: str
    content: str
    score: float
    metadata: ChunkMetadataSchema

    model_config = ConfigDict(from_attributes=True)


class SearchResponse(BaseModel):
    """Response schema for search endpoint."""
    query: str
    results: list[SearchResultItem]
    total_results: int
    processing_time_seconds: float


class ConversationMessageCreate(BaseModel):
    """Schema for creating a conversation message."""
    session_id: str = Field(..., min_length=1, max_length=255)
    role: str = Field(..., pattern="^(user|assistant)$")
    message: str = Field(..., min_length=1)
    chunk_references: Optional[list[str]] = None


class ConversationMessageResponse(BaseModel):
    """Response schema for conversation message."""
    id: int
    session_id: str
    role: str
    message: str
    chunk_references: Optional[list[str]]
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)


class HealthCheckResponse(BaseModel):
    """Health check response."""
    status: str
    app_name: str
    version: str
    database_connected: bool
    qdrant_connected: bool
    openai_configured: bool


class StatsResponse(BaseModel):
    """Statistics response."""
    total_chunks: int
    total_queries: int
    total_conversations: int
    avg_query_results: float
    collection_info: dict[str, Any]


class SourceReference(BaseModel):
    """Reference to a source chunk used in answer."""
    chunk_id: str
    heading: str
    chapter: str


class AskRequest(BaseModel):
    """Request schema for /ask endpoint with RAG retrieval."""
    query: str = Field(..., min_length=1, max_length=1000, description="User's question")
    top_k: int = Field(default=5, ge=1, le=10, description="Number of chunks to retrieve")
    filters: Optional[SearchFilters] = None
    similarity_threshold: Optional[float] = Field(default=0.7, ge=0.0, le=1.0)
    session_id: Optional[str] = Field(None, description="Session ID for conversation context")
    include_history: bool = Field(default=False, description="Include conversation history in context")


class AskResponse(BaseModel):
    """Response schema for /ask endpoint."""
    query: str
    answer: str
    sources: list[SourceReference]
    model: str
    tokens_used: int
    is_grounded: bool
    processing_time_seconds: float
    chunks_retrieved: int


class AskSelectedRequest(BaseModel):
    """Request schema for /ask-selected endpoint."""
    query: str = Field(..., min_length=1, max_length=1000, description="User's question")
    selected_text: str = Field(..., min_length=1, max_length=10000, description="Text selected by user")
    selection_metadata: Optional[dict[str, Any]] = Field(
        None,
        description="Optional metadata about the selection (chunk_id, heading, etc.)"
    )


class AskSelectedResponse(BaseModel):
    """Response schema for /ask-selected endpoint."""
    query: str
    answer: str
    selection_length: int
    selection_metadata: dict[str, Any]
    model: str
    tokens_used: int
    is_grounded: bool
    processing_time_seconds: float
