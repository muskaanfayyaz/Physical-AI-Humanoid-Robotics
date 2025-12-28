# System Architecture Overview

Complete architecture of the Physical AI RAG system.

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         USER BROWSER                            │
│  https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐         ┌──────────────┐                    │
│  │  Docusaurus  │         │   Chatbot    │                    │
│  │   Textbook   │◄────────│      UI      │                    │
│  │  (18 chapters)│         │   (React)    │                    │
│  └──────────────┘         └───────┬──────┘                    │
│                                    │                            │
└────────────────────────────────────┼────────────────────────────┘
                                     │
                          HTTPS POST /api/v1/ask
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                    FASTAPI BACKEND                              │
│             https://your-backend.railway.app                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    │
│  │    Ingest    │    │    Search    │    │     Ask      │    │
│  │   Router     │    │   Router     │    │   Router     │    │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘    │
│         │                   │                    │             │
│         └───────────────────┼────────────────────┘             │
│                             │                                  │
│         ┌───────────────────┴────────────────────┐            │
│         │          Service Layer                 │            │
│         ├────────────────────────────────────────┤            │
│         │  • Embeddings Service (OpenAI)         │            │
│         │  • Qdrant Service (Vector Search)      │            │
│         │  • LLM Service (GPT-4o-mini)           │            │
│         │  • Chunk Service (Business Logic)      │            │
│         └───────────────────┬────────────────────┘            │
│                             │                                  │
└─────────────────────────────┼──────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│  OpenAI API   │    │ Qdrant Cloud  │    │ Neon Postgres │
│               │    │               │    │               │
│ • Embeddings  │    │ • 632 Vectors │    │ • Metadata    │
│   (1536-dim)  │    │ • Cosine      │    │ • Chunks      │
│ • GPT-4o-mini │    │   Search      │    │ • History     │
│               │    │               │    │ • Queries     │
└───────────────┘    └───────────────┘    └───────────────┘
```

---

## Data Flow: Question Answering

### 1. Normal Mode (RAG)

```
User Question: "What is ROS 2?"
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 1: User Interface                                   │
│  ─────────────────────────────                            │
│  • User types question in chatbot                         │
│  • Chatbot.jsx captures input                             │
│  • POST /api/v1/ask/                                      │
│  • Payload: { query: "What is ROS 2?", top_k: 5 }        │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 2: Generate Query Embedding                         │
│  ───────────────────────────────                          │
│  • EmbeddingService.generate_embedding()                  │
│  • OpenAI API call: text-embedding-3-small                │
│  • Returns: [0.023, -0.145, 0.892, ..., 0.234]           │
│  • Vector length: 1536 dimensions                         │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 3: Vector Search in Qdrant                          │
│  ──────────────────────────────                           │
│  • QdrantService.search()                                 │
│  • Cosine similarity search                               │
│  • Returns top 5 similar chunks                           │
│  • Each with score (0.0-1.0)                              │
│                                                            │
│  Results:                                                  │
│  1. chunk_id: "chapter-03-ros2_chunk_0004" (0.92)        │
│  2. chunk_id: "chapter-03-ros2_chunk_0005" (0.89)        │
│  3. chunk_id: "chapter-03-ros2_chunk_0003" (0.87)        │
│  4. chunk_id: "chapter-04-ros2_chunk_0012" (0.85)        │
│  5. chunk_id: "chapter-03-ros2_chunk_0006" (0.83)        │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 4: Retrieve Full Metadata from Postgres             │
│  ────────────────────────────────────────                 │
│  • Query: SELECT * FROM chunk_metadata                    │
│           WHERE chunk_id IN (...)                         │
│  • Returns: Full content + metadata                       │
│                                                            │
│  For each chunk:                                           │
│  • content: "ROS 2 uses DDS (Data Distribution..."       │
│  • heading: "Chapter 3 > ROS 2 Architecture"              │
│  • chapter: "introduction-to-ros2"                        │
│  • token_count: 345                                       │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 5: Build Context & Generate Answer (LLM)            │
│  ─────────────────────────────────────────                │
│  • LLMService.answer_with_context()                       │
│  • Build formatted context from 5 chunks                  │
│  • Add system prompt (anti-hallucination)                 │
│  • OpenAI ChatCompletion API (GPT-4o-mini)                │
│                                                            │
│  Context format:                                           │
│  """                                                       │
│  [Source 1: Chapter 3 > ROS 2 Architecture]               │
│  ROS 2 uses DDS (Data Distribution Service)...            │
│                                                            │
│  [Source 2: Chapter 3 > DDS Middleware]                   │
│  DDS provides real-time communication...                  │
│  """                                                       │
│                                                            │
│  LLM generates grounded answer:                            │
│  "According to Source 1, ROS 2 uses DDS..."               │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 6: Store Conversation & Return Response             │
│  ────────────────────────────────────────────             │
│  • Store in conversation_history table                    │
│    - User message                                          │
│    - Assistant response                                    │
│    - Chunk references                                      │
│  • Store in search_queries table (analytics)              │
│                                                            │
│  Return JSON:                                              │
│  {                                                         │
│    "query": "What is ROS 2?",                             │
│    "answer": "According to Source 1...",                  │
│    "sources": [...],                                       │
│    "model": "gpt-4o-mini",                                │
│    "tokens_used": 456,                                    │
│    "processing_time_seconds": 1.23                        │
│  }                                                         │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 7: Display in Chatbot UI                            │
│  ─────────────────────────────                            │
│  • Parse response JSON                                     │
│  • Display answer in message bubble                       │
│  • Show sources as clickable links                        │
│  • Show metadata (processing time, etc.)                  │
│  • Add to conversation history                            │
└───────────────────────────────────────────────────────────┘
```

**Total Time:** ~1.5 seconds
- Embedding: ~100ms
- Vector search: ~50ms
- Postgres query: ~50ms
- LLM generation: ~1000ms
- Network overhead: ~300ms

---

### 2. Selected Text Mode

```
User Selects Text: "ROS 2 uses DDS as middleware..."
User Asks: "What does this say about real-time?"
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 1: Text Selection Detected                          │
│  ──────────────────────────                               │
│  • window.getSelection() captures text                    │
│  • setSelectedText(text)                                  │
│  • setMode('selected')                                    │
│  • Chatbot auto-opens                                     │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 2: Send to Backend                                  │
│  ─────────────────────                                    │
│  • POST /api/v1/ask/selected                              │
│  • Payload: {                                              │
│      query: "What does this say about real-time?",        │
│      selected_text: "ROS 2 uses DDS as middleware..."     │
│    }                                                       │
│  • NO database retrieval                                  │
│  • NO embedding generation                                │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 3: Direct LLM Processing                             │
│  ────────────────────────                                 │
│  • LLMService.answer_from_selection()                     │
│  • System prompt: "ONLY use selected text"                │
│  • User prompt:                                            │
│    """                                                     │
│    Selected Text:                                          │
│    ROS 2 uses DDS as middleware...                        │
│                                                            │
│    Question: What does this say about real-time?          │
│                                                            │
│    Answer ONLY from the selected text.                    │
│    """                                                     │
│  • GPT-4o-mini generates answer                           │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 4: Return Response                                   │
│  ──────────────────                                       │
│  {                                                         │
│    "query": "What does this say about real-time?",       │
│    "answer": "The selected text mentions that DDS...",   │
│    "selection_length": 150,                               │
│    "model": "gpt-4o-mini",                                │
│    "tokens_used": 234,                                    │
│    "processing_time_seconds": 0.45                        │
│  }                                                         │
└───────────────────────────────────────────────────────────┘
```

**Total Time:** ~0.5 seconds (faster - no retrieval)

---

## Data Ingestion Flow

```
chunks.json (632 chunks)
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  POST /api/v1/ingest/chunks                                │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 1: Load & Validate                                   │
│  ────────────────────                                      │
│  • ChunkService.load_chunks_from_file()                   │
│  • Parse JSON                                              │
│  • Validate schema                                         │
│  • 632 chunks loaded                                       │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 2: Generate Embeddings (Batch Processing)            │
│  ──────────────────────────────────────────                │
│  • EmbeddingService.generate_embeddings_batch()           │
│  • Process in batches of 100 chunks                       │
│  • OpenAI API: text-embedding-3-small                     │
│                                                            │
│  Batch 1: chunks 1-100   → 100 embeddings (8s)           │
│  Batch 2: chunks 101-200 → 100 embeddings (8s)           │
│  Batch 3: chunks 201-300 → 100 embeddings (8s)           │
│  Batch 4: chunks 301-400 → 100 embeddings (8s)           │
│  Batch 5: chunks 401-500 → 100 embeddings (8s)           │
│  Batch 6: chunks 501-600 → 100 embeddings (8s)           │
│  Batch 7: chunks 601-632 →  32 embeddings (3s)           │
│                                                            │
│  Total time: ~51 seconds                                   │
│  Total cost: ~$0.50                                       │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 3: Store in Postgres                                 │
│  ────────────────────────                                 │
│  • For each chunk:                                         │
│    INSERT INTO chunk_metadata (                           │
│      chunk_id, content, chapter_type,                     │
│      heading_hierarchy, token_count, ...                  │
│    )                                                       │
│                                                            │
│  • 632 rows inserted                                       │
│  • Indexes created on chunk_id, chapter_type              │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 4: Store in Qdrant                                   │
│  ──────────────────────                                   │
│  • Create collection (if not exists)                      │
│  • QdrantService.upsert_points()                          │
│  • For each chunk:                                         │
│    - ID: numeric (0-631)                                  │
│    - Vector: 1536-dimensional embedding                   │
│    - Payload: {                                            │
│        chunk_id, content, heading, chapter,               │
│        token_count, ...                                    │
│      }                                                     │
│                                                            │
│  • 632 vectors stored                                      │
│  • Cosine similarity index built                          │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  STEP 5: Return Response                                   │
│  ──────────────────────                                   │
│  {                                                         │
│    "success": true,                                        │
│    "chunks_processed": 632,                               │
│    "chunks_stored_postgres": 632,                         │
│    "chunks_stored_qdrant": 632,                           │
│    "errors": [],                                           │
│    "processing_time_seconds": 52.3                        │
│  }                                                         │
└───────────────────────────────────────────────────────────┘
```

---

## Database Schemas

### Neon Postgres

**chunk_metadata table:**
```sql
CREATE TABLE chunk_metadata (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(255) UNIQUE NOT NULL,

    -- Chapter info
    chapter_type VARCHAR(50) NOT NULL,
    chapter_number INTEGER NOT NULL,
    chapter_title_slug VARCHAR(255) NOT NULL,

    -- Section info
    section_title VARCHAR(500) NOT NULL,
    heading_hierarchy TEXT NOT NULL,
    section_path JSON NOT NULL,

    -- Content
    content TEXT NOT NULL,
    token_count INTEGER NOT NULL,
    char_count INTEGER NOT NULL,

    -- Timestamps
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chunk_id ON chunk_metadata(chunk_id);
CREATE INDEX idx_chapter_type ON chunk_metadata(chapter_type);
```

**conversation_history table:**
```sql
CREATE TABLE conversation_history (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255) NOT NULL,
    role VARCHAR(50) NOT NULL,
    message TEXT NOT NULL,
    chunk_references JSON,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_session_id ON conversation_history(session_id);
```

**search_queries table:**
```sql
CREATE TABLE search_queries (
    id SERIAL PRIMARY KEY,
    query_text TEXT NOT NULL,
    top_k INTEGER NOT NULL,
    results_count INTEGER NOT NULL,
    avg_similarity_score FLOAT,
    filters JSON,
    created_at TIMESTAMP DEFAULT NOW()
);
```

### Qdrant Collection

**Collection: physical_ai_textbook**
```json
{
  "name": "physical_ai_textbook",
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "payload_schema": {
    "chunk_id": "keyword",
    "content": "text",
    "chapter_type": "keyword",
    "chapter_number": "integer",
    "heading_hierarchy": "text",
    "token_count": "integer"
  }
}
```

---

## Technology Stack

### Frontend
- **Framework:** Docusaurus 3.x
- **UI Library:** React 18
- **Styling:** CSS Modules
- **Hosting:** GitHub Pages
- **Build Tool:** Webpack (via Docusaurus)

### Backend
- **Framework:** FastAPI 0.109.0
- **Server:** Uvicorn (ASGI)
- **Python:** 3.11+
- **ORM:** SQLAlchemy 2.0 (async)
- **Hosting:** Railway / Render / Fly.io

### Databases
- **Vector DB:** Qdrant Cloud (1GB free tier)
- **Relational DB:** Neon Postgres (10GB free tier)

### AI Services
- **Embeddings:** OpenAI text-embedding-3-small (1536-dim)
- **LLM:** OpenAI GPT-4o-mini (chat completions)

---

## Security Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  FRONTEND (GitHub Pages)                                    │
│  ───────────────────────                                    │
│  • Static files only (no secrets)                           │
│  • HTTPS enforced                                            │
│  • CSP headers                                               │
│  • XSS protection (React escaping)                          │
└────────────────────┬────────────────────────────────────────┘
                     │
                     │ HTTPS (TLS 1.3)
                     │
┌────────────────────▼────────────────────────────────────────┐
│  BACKEND (Railway)                                           │
│  ────────────────                                            │
│  • CORS validation                                           │
│  • Rate limiting (optional)                                  │
│  • Input validation (Pydantic)                               │
│  • Environment variables (secrets)                           │
│  • SQL injection protection (ORM)                            │
└────────────────────┬────────────────────────────────────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
        │ TLS        │ TLS        │ TLS
        ▼            ▼            ▼
┌────────────┐  ┌────────────┐  ┌────────────┐
│  OpenAI    │  │  Qdrant    │  │   Neon     │
│    API     │  │   Cloud    │  │ Postgres   │
│            │  │            │  │            │
│ • API key  │  │ • API key  │  │ • SSL      │
│ • HTTPS    │  │ • HTTPS    │  │ • Password │
└────────────┘  └────────────┘  └────────────┘
```

---

## Performance Metrics

### Latency Breakdown

**Normal Mode (/ask):**
```
Total: 1.5s
├─ Embedding generation: 100ms
├─ Vector search: 50ms
├─ Postgres query: 50ms
├─ LLM generation: 1000ms
└─ Network overhead: 300ms
```

**Selected Mode (/ask/selected):**
```
Total: 0.5s
├─ LLM generation: 400ms
└─ Network overhead: 100ms
```

### Throughput

- **Concurrent requests:** 10-20 req/s (single worker)
- **Database connections:** Pool of 10
- **OpenAI rate limit:** 500 req/min (tier 1)

### Resource Usage

**Backend (per request):**
- CPU: ~50ms
- Memory: ~50MB
- Network: ~5KB request, ~2KB response

**Storage:**
- Qdrant: ~10MB (632 vectors × 1536 dims × 4 bytes)
- Postgres: ~100MB (metadata + history)

---

## Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  DEVELOPMENT                                                 │
├─────────────────────────────────────────────────────────────┤
│  Localhost:3000 (Docusaurus)                                │
│        ↓                                                     │
│  Localhost:8000 (FastAPI)                                   │
│        ↓                                                     │
│  [OpenAI, Qdrant, Neon]                                     │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  PRODUCTION                                                  │
├─────────────────────────────────────────────────────────────┤
│  GitHub Pages (Static CDN)                                  │
│        ↓                                                     │
│  Railway (FastAPI + Uvicorn)                                │
│        ↓                                                     │
│  [OpenAI, Qdrant Cloud, Neon Serverless]                   │
└─────────────────────────────────────────────────────────────┘
```

---

**Architecture Version:** 1.0
**Last Updated:** December 2024
**Status:** ✅ Production Ready
