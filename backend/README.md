# Physical AI RAG Backend

Production-ready FastAPI backend for the Physical AI & Humanoid Robotics textbook RAG system.

## Features

✅ **Chunk Ingestion** - Load and process 632 semantic chunks from `chunks.json`
✅ **OpenAI Embeddings** - Generate 1536-dimensional vectors using `text-embedding-3-small`
✅ **Qdrant Cloud** - Vector storage and similarity search
✅ **Neon Postgres** - Metadata storage and conversation history
✅ **Semantic Search** - Query textbook content with filters
✅ **Grounded Q&A (NEW)** - LLM-powered answers with anti-hallucination
✅ **Selection-based Q&A (NEW)** - Answer questions from user-selected text
✅ **Conversation Tracking** - Store and retrieve chat history
✅ **Health Monitoring** - Connection status and system stats

---

## Architecture

```
┌─────────────────┐
│   FastAPI App   │
└────────┬────────┘
         │
    ┌────┴─────┬──────────┬────────────┐
    │          │          │            │
┌───▼───┐ ┌───▼────┐ ┌──▼─────┐ ┌────▼────┐
│Ingest │ │ Search │ │  Chat  │ │ Health  │
│Router │ │ Router │ │ Router │ │  Check  │
└───┬───┘ └───┬────┘ └──┬─────┘ └────┬────┘
    │         │          │            │
    └─────────┴──────────┴────────────┘
              │
    ┌─────────┴─────────┐
    │                   │
┌───▼────────┐    ┌────▼────────┐
│  Services  │    │  Database   │
├────────────┤    ├─────────────┤
│ Embeddings │    │ PostgreSQL  │
│  Qdrant    │    │ (SQLAlchemy)│
│   Chunks   │    └─────────────┘
└────────────┘
```

---

## Quick Start

### 1. Prerequisites

- Python 3.11+
- PostgreSQL database (Neon recommended)
- Qdrant Cloud account
- OpenAI API key

### 2. Installation

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Configuration

Create `.env` file:

```bash
cp .env.example .env
```

Update with your credentials:

```env
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant Cloud
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo

# Neon Postgres
POSTGRES_URL=postgresql://username:password@your-host.neon.tech/dbname?sslmode=require
```

### 4. Run the Server

```bash
# Development mode (auto-reload)
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Production mode
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

Server runs at: **http://localhost:8000**

---

## API Documentation

### Interactive Docs

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

### Core Endpoints

#### **POST /api/v1/ingest/chunks**
Ingest all chunks from `chunks.json`

```bash
curl -X POST http://localhost:8000/api/v1/ingest/chunks
```

Response:
```json
{
  "success": true,
  "chunks_processed": 632,
  "chunks_stored_postgres": 632,
  "chunks_stored_qdrant": 632,
  "errors": [],
  "processing_time_seconds": 45.2
}
```

#### **POST /api/v1/search/**
Semantic search with filters

```bash
curl -X POST http://localhost:8000/api/v1/search/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How does ROS 2 handle communication between nodes?",
    "top_k": 5,
    "similarity_threshold": 0.7,
    "filters": {
      "chapter_type": "chapter",
      "chapter_number": 3
    }
  }'
```

Response:
```json
{
  "query": "How does ROS 2 handle communication?",
  "results": [
    {
      "chunk_id": "chapter-03-introduction-to-ros2_chunk_0004",
      "content": "ROS 2 uses DDS (Data Distribution Service)...",
      "score": 0.892,
      "metadata": {
        "chapter_type": "chapter",
        "chapter_number": 3,
        "section_title": "Communication in ROS 2",
        "heading_hierarchy": "Chapter 3 > ROS 2 Architecture > DDS Layer",
        "token_count": 345
      }
    }
  ],
  "total_results": 5,
  "processing_time_seconds": 0.23
}
```

#### **GET /api/v1/search/chunk/{chunk_id}**
Get specific chunk by ID

```bash
curl http://localhost:8000/api/v1/search/chunk/chapter-01-introduction-to-physical-ai_chunk_0001
```

#### **POST /api/v1/conversation/message**
Store conversation message

```bash
curl -X POST http://localhost:8000/api/v1/conversation/message \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "user-123-session",
    "role": "user",
    "message": "What is physical AI?",
    "chunk_references": ["chapter-01-introduction-to-physical-ai_chunk_0001"]
  }'
```

#### **GET /api/v1/conversation/session/{session_id}**
Retrieve conversation history

```bash
curl http://localhost:8000/api/v1/conversation/session/user-123-session?limit=50
```

#### **POST /api/v1/ask/** ⭐ NEW
Ask questions with grounded LLM answers (RAG-based)

```bash
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How does ROS 2 work?",
    "top_k": 5
  }'
```

Response:
```json
{
  "query": "How does ROS 2 work?",
  "answer": "According to Source 1, ROS 2 uses DDS (Data Distribution Service) as its middleware...",
  "sources": [
    {
      "chunk_id": "chapter-03-introduction-to-ros2_chunk_0004",
      "heading": "Chapter 3 > ROS 2 Architecture",
      "chapter": "introduction-to-ros2"
    }
  ],
  "model": "gpt-4o-mini",
  "tokens_used": 456,
  "is_grounded": true,
  "chunks_retrieved": 5
}
```

**Features:**
- Retrieves relevant chunks from Qdrant
- Answers ONLY using retrieved context
- Refuses to hallucinate if info not present
- Provides source attribution

See [ASK_ENDPOINTS.md](ASK_ENDPOINTS.md) for complete documentation.

#### **POST /api/v1/ask/selected** ⭐ NEW
Ask questions about user-selected text (No RAG)

```bash
curl -X POST http://localhost:8000/api/v1/ask/selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this say about DDS?",
    "selected_text": "ROS 2 uses DDS (Data Distribution Service)..."
  }'
```

Response:
```json
{
  "query": "What does this say about DDS?",
  "answer": "The selected text explains that DDS is used as ROS 2's middleware for real-time communication...",
  "model": "gpt-4o-mini",
  "tokens_used": 234,
  "is_grounded": true
}
```

**Features:**
- No database retrieval
- Answers ONLY from selected text
- Says "Not found in selection" if answer not present
- Fast response (~0.5s)

See [ASK_ENDPOINTS.md](ASK_ENDPOINTS.md) for complete documentation.

#### **GET /health**
System health check

```bash
curl http://localhost:8000/health
```

Response:
```json
{
  "status": "healthy",
  "app_name": "Physical AI RAG Backend",
  "version": "1.0.0",
  "database_connected": true,
  "qdrant_connected": true,
  "openai_configured": true
}
```

#### **GET /api/v1/ingest/stats**
System statistics

```bash
curl http://localhost:8000/api/v1/ingest/stats
```

---

## Database Schema

### Postgres Tables

#### **chunk_metadata**
Stores metadata for each chunk

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| chunk_id | String | Unique identifier |
| chapter_type | String | chapter/appendix |
| chapter_number | Integer | Chapter number |
| section_title | String | Section name |
| heading_hierarchy | Text | Full path |
| content | Text | Full chunk text |
| token_count | Integer | Token count |
| created_at | DateTime | Timestamp |

#### **search_queries**
Logs all search queries

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| query_text | Text | Search query |
| top_k | Integer | Results requested |
| results_count | Integer | Results returned |
| avg_similarity_score | Float | Average score |
| created_at | DateTime | Timestamp |

#### **conversation_history**
Stores chat messages

| Column | Type | Description |
|--------|------|-------------|
| id | Integer | Primary key |
| session_id | String | Session identifier |
| role | String | user/assistant |
| message | Text | Message content |
| chunk_references | JSON | Referenced chunks |
| created_at | DateTime | Timestamp |

---

## Development

### Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI app
│   ├── config.py            # Settings management
│   ├── database.py          # SQLAlchemy models
│   ├── schemas.py           # Pydantic schemas
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── ingest.py        # Ingestion endpoints
│   │   ├── search.py        # Search endpoints
│   │   └── conversation.py  # Chat endpoints
│   └── services/
│       ├── __init__.py
│       ├── embeddings.py    # OpenAI service
│       ├── qdrant_service.py # Vector DB
│       └── chunk_service.py  # Business logic
├── .env                     # Environment variables (git-ignored)
├── .env.example            # Template
├── requirements.txt        # Dependencies
└── README.md              # This file
```

### Running Tests

```bash
pytest
```

### Code Quality

```bash
# Format code
black app/

# Lint
ruff check app/

# Type checking
mypy app/
```

---

## Deployment

### Option 1: Railway

```bash
railway init
railway add
railway up
```

### Option 2: Render

1. Create new Web Service
2. Connect GitHub repo
3. Build command: `pip install -r requirements.txt`
4. Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Option 3: Docker

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app/ ./app/

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

---

## Environment Variables Reference

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `OPENAI_API_KEY` | ✅ | - | OpenAI API key |
| `OPENAI_EMBEDDING_MODEL` | ❌ | text-embedding-3-small | Embedding model |
| `EMBEDDING_DIMENSIONS` | ❌ | 1536 | Vector dimensions |
| `QDRANT_URL` | ✅ | - | Qdrant Cloud URL |
| `QDRANT_API_KEY` | ✅ | - | Qdrant API key |
| `QDRANT_COLLECTION_NAME` | ❌ | physical_ai_textbook | Collection name |
| `POSTGRES_URL` | ✅ | - | Postgres connection string |
| `POSTGRES_POOL_SIZE` | ❌ | 10 | Connection pool size |
| `DEBUG` | ❌ | False | Enable debug mode |
| `CORS_ORIGINS` | ❌ | localhost:3000 | Allowed origins |

---

## Best Practices

✅ **Security**
- Never commit `.env` file
- Use environment variables for all secrets
- Enable CORS only for trusted origins
- Use HTTPS in production

✅ **Performance**
- Use connection pooling (default: 10)
- Batch embedding generation (100 chunks/batch)
- Cache embedding service instances
- Use async/await throughout

✅ **Monitoring**
- Check `/health` endpoint regularly
- Monitor search query logs
- Track embedding API usage
- Set up error alerting

✅ **Scaling**
- Run multiple workers: `--workers 4`
- Use Redis for caching (optional)
- Implement rate limiting (optional)
- Monitor Qdrant collection size

---

## Troubleshooting

### Database Connection Failed

```bash
# Test connection
psql "postgresql://username:password@host/db?sslmode=require"

# Check environment variable
echo $POSTGRES_URL
```

### Qdrant Connection Failed

```bash
# Verify API key and URL
curl -H "api-key: YOUR_KEY" https://your-qdrant-url/collections
```

### OpenAI Rate Limit

- Reduce batch size in `embeddings.py`
- Add delays between batches
- Upgrade OpenAI tier

### Ingestion Slow

- Expected: ~45-60 seconds for 632 chunks
- Bottleneck: OpenAI API calls
- Solution: Use batching (already implemented)

---

## License

MIT License - See main repository

---

## Support

- **GitHub Issues**: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/issues
- **API Docs**: http://localhost:8000/docs
- **Hackathon**: GIAIC Hackathon I

---

**Built with:** FastAPI • OpenAI • Qdrant • Neon Postgres • SQLAlchemy
**Status:** ✅ Production Ready
**Version:** 1.0.0
