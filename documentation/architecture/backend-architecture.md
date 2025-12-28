# Backend Architecture Documentation

## System Overview

The Physical AI RAG Backend is a production-grade FastAPI application implementing a complete Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics textbook.

---

## Technology Stack

### Core Framework
- **FastAPI 0.109.0** - Modern async web framework
- **Uvicorn** - ASGI server with auto-reload
- **Pydantic v2** - Data validation and settings management

### Databases
- **Neon Postgres** - Metadata and conversation storage
  - Async driver: `asyncpg`
  - ORM: SQLAlchemy 2.0 with async support
- **Qdrant Cloud** - Vector storage and similarity search
  - Client: `qdrant-client 1.7.3`
  - Distance: Cosine similarity

### AI Services
- **OpenAI API** - Embedding generation
  - Model: `text-embedding-3-small`
  - Dimensions: 1536
  - Batch processing: Up to 100 texts/batch

---

## Architecture Layers

```
┌─────────────────────────────────────────────────────────────┐
│                      API Layer (FastAPI)                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐ │
│  │  Ingest  │  │  Search  │  │   Chat   │  │    Health   │ │
│  │  Router  │  │  Router  │  │  Router  │  │    Check    │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘ │
└───────┼────────────┼─────────────┼────────────────┼────────┘
        │            │              │                │
┌───────┼────────────┼──────────────┼────────────────┼────────┐
│       │            │              │                │         │
│  ┌────▼────────────▼──────────────▼────────────────▼──────┐ │
│  │              Service Layer (Business Logic)            │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐  │ │
│  │  │   Chunk      │  │  Embeddings  │  │   Qdrant    │  │ │
│  │  │   Service    │  │   Service    │  │   Service   │  │ │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬──────┘  │ │
│  └─────────┼──────────────────┼──────────────────┼─────────┘ │
│            │                  │                  │           │
└────────────┼──────────────────┼──────────────────┼───────────┘
             │                  │                  │
┌────────────┼──────────────────┼──────────────────┼───────────┐
│            │                  │                  │           │
│  ┌─────────▼──────────┐  ┌───▼──────────┐  ┌───▼─────────┐ │
│  │  PostgreSQL DB     │  │  OpenAI API  │  │  Qdrant DB  │ │
│  │  (Neon)            │  │              │  │  (Cloud)    │ │
│  │                    │  │              │  │             │ │
│  │ • chunk_metadata   │  │ • Embeddings │  │ • Vectors   │ │
│  │ • search_queries   │  │ • 1536-dim   │  │ • Cosine    │ │
│  │ • conversations    │  │ • Batching   │  │ • Filters   │ │
│  └────────────────────┘  └──────────────┘  └─────────────┘ │
│                     Data Layer                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Data Flow

### 1. Ingestion Pipeline

```
chunks.json (632 chunks)
    │
    ▼
┌─────────────────────┐
│  Load & Validate    │ ← chunk_service.load_chunks_from_file()
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│ Generate Embeddings │ ← embedding_service.generate_embeddings_batch()
│  (100 chunks/batch) │
└─────────┬───────────┘
          │
          ├─────────────────────┬──────────────────────┐
          │                     │                      │
          ▼                     ▼                      ▼
┌──────────────────┐  ┌──────────────────┐  ┌─────────────────┐
│ Store Metadata   │  │  Store Vectors   │  │  Create Index   │
│  in Postgres     │  │  in Qdrant       │  │   (automatic)   │
└──────────────────┘  └──────────────────┘  └─────────────────┘
```

### 2. Search Query Flow

```
User Query: "How does ROS 2 work?"
    │
    ▼
┌─────────────────────┐
│ Generate Query      │ ← embedding_service.generate_embedding()
│   Embedding         │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│ Vector Search       │ ← qdrant_service.search()
│  (Cosine Similarity)│    • top_k: 5
│  Apply Filters      │    • threshold: 0.7
└─────────┬───────────┘    • chapter filter
          │
          ▼
┌─────────────────────┐
│ Retrieve Metadata   │ ← database query
│  from Postgres      │    • Join by chunk_id
└─────────┬───────────┘    • Full content
          │
          ▼
┌─────────────────────┐
│ Rank & Format       │ ← Sorted by score
│   Results           │    • Top 5 chunks
└─────────┬───────────┘    • Metadata included
          │
          ▼
┌─────────────────────┐
│  Log Query          │ ← search_queries table
│  Return Response    │    • Analytics
└─────────────────────┘
```

### 3. Conversation Storage

```
User Message
    │
    ▼
┌─────────────────────┐
│ Store in Postgres   │ ← conversation_history table
│  session_id         │
│  role: "user"       │
│  message            │
│  chunk_references   │
└─────────────────────┘
    │
    ▼
Assistant processes...
    │
    ▼
┌─────────────────────┐
│ Store Response      │ ← conversation_history table
│  same session_id    │
│  role: "assistant"  │
│  message            │
│  chunk_references   │
└─────────────────────┘
```

---

## Database Schema

### PostgreSQL Tables

#### chunk_metadata
```sql
CREATE TABLE chunk_metadata (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(255) UNIQUE NOT NULL,

    -- Chapter info
    chapter_type VARCHAR(50) NOT NULL,
    chapter_number INTEGER NOT NULL,
    chapter_title_slug VARCHAR(255) NOT NULL,
    filename VARCHAR(255) NOT NULL,

    -- Section info
    section_level INTEGER NOT NULL,
    section_title VARCHAR(500) NOT NULL,
    section_path JSON NOT NULL,
    heading_hierarchy TEXT NOT NULL,

    -- Content
    part_number INTEGER NOT NULL,
    total_parts INTEGER NOT NULL,
    token_count INTEGER NOT NULL,
    char_count INTEGER NOT NULL,
    content TEXT NOT NULL,

    -- Timestamps
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chunk_id ON chunk_metadata(chunk_id);
CREATE INDEX idx_chapter_type ON chunk_metadata(chapter_type);
CREATE INDEX idx_chapter_number ON chunk_metadata(chapter_number);
```

#### search_queries
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

#### conversation_history
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

### Qdrant Collection

```python
{
    "name": "physical_ai_textbook",
    "vectors": {
        "size": 1536,
        "distance": "Cosine"
    },
    "payload": {
        "chunk_id": "string",
        "content": "string",
        "chapter_type": "string",
        "chapter_number": "integer",
        "section_title": "string",
        "heading_hierarchy": "string",
        "token_count": "integer"
    }
}
```

---

## API Endpoints

### Ingestion
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/ingest/chunks` | Ingest all chunks |
| GET | `/api/v1/ingest/stats` | System statistics |

### Search
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/search/` | Semantic search |
| GET | `/api/v1/search/chunk/{id}` | Get chunk by ID |

### Conversation
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/v1/conversation/message` | Store message |
| GET | `/api/v1/conversation/session/{id}` | Get history |
| DELETE | `/api/v1/conversation/session/{id}` | Delete session |

### Health
| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/` | Root info |

---

## Performance Characteristics

### Ingestion
- **Total Chunks**: 632
- **Processing Time**: ~45-60 seconds
- **Bottleneck**: OpenAI API rate limits
- **Optimization**: Batch processing (100 chunks/batch)

### Search
- **Query Latency**: 200-300ms
  - Embedding generation: ~100ms
  - Vector search: ~50ms
  - Postgres query: ~50ms
- **Throughput**: ~10-20 queries/second (single worker)

### Scalability
- **Database Pool**: 10 connections (configurable)
- **Workers**: 1 (development), 4+ (production)
- **Qdrant**: Auto-scales with collection size
- **OpenAI**: Rate-limited by API tier

---

## Security Considerations

### API Keys
- All keys stored in environment variables
- Never logged or exposed in responses
- `.env` file git-ignored

### CORS
- Configurable allowed origins
- Default: localhost + GitHub Pages
- Production: Restrict to specific domains

### Database
- SSL/TLS required for Postgres (Neon)
- Connection pooling with limits
- SQL injection protected (SQLAlchemy ORM)

### Input Validation
- Pydantic schemas for all endpoints
- Query length limits (1000 chars)
- top_k limits (max 20)
- Session ID length limits

---

## Error Handling

### Structured Errors
```python
{
    "detail": "Error message",
    "status_code": 500
}
```

### Common Errors
| Status | Cause | Solution |
|--------|-------|----------|
| 404 | Chunk not found | Check chunk_id |
| 500 | Database connection | Check POSTGRES_URL |
| 500 | Qdrant connection | Check QDRANT_URL/KEY |
| 429 | OpenAI rate limit | Reduce batch size |

### Logging
```python
# Log levels
DEBUG: Detailed info (development)
INFO: General operations
WARNING: Potential issues
ERROR: Failed operations
```

---

## Deployment Strategies

### Development
```bash
uvicorn app.main:app --reload
```

### Production
```bash
uvicorn app.main:app --workers 4 --host 0.0.0.0 --port 8000
```

### Docker
```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY app/ ./app/
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0"]
```

### Platform-as-a-Service
- **Railway**: Auto-deploy from GitHub
- **Render**: Web service with auto-scaling
- **Fly.io**: Edge deployment

---

## Monitoring & Observability

### Health Endpoints
```bash
# Quick check
curl http://localhost:8000/health

# Detailed stats
curl http://localhost:8000/api/v1/ingest/stats
```

### Metrics to Track
- Request latency (P50, P95, P99)
- Error rate
- Database connection pool usage
- OpenAI API usage/costs
- Qdrant collection size

### Logging Strategy
- Structured JSON logs
- Request/response logging
- Error stack traces
- Performance timing

---

## Future Enhancements

### Planned Features
- [ ] Redis caching for frequent queries
- [ ] Rate limiting per IP/session
- [ ] Streaming responses
- [ ] Multi-language support
- [ ] Advanced filters (date ranges, tags)
- [ ] Query rewriting/expansion
- [ ] User authentication (optional)

### Optimizations
- [ ] Embedding caching
- [ ] Hybrid search (vector + keyword)
- [ ] Reranking with cross-encoder
- [ ] Query result caching

---

## Dependencies Overview

### Critical
- `fastapi` - Web framework
- `sqlalchemy[asyncio]` - ORM
- `asyncpg` - Postgres driver
- `qdrant-client` - Vector DB
- `openai` - Embeddings

### Supporting
- `pydantic-settings` - Config
- `uvicorn` - Server
- `python-dotenv` - Env vars

### Development
- `pytest` - Testing
- `black` - Formatting
- `ruff` - Linting

---

## Best Practices Implemented

✅ **Async/Await** - All I/O operations
✅ **Dependency Injection** - FastAPI Depends()
✅ **Type Hints** - Full typing throughout
✅ **Pydantic Validation** - Request/response schemas
✅ **Singleton Services** - Reuse connections
✅ **Connection Pooling** - Database efficiency
✅ **Batch Processing** - Embedding generation
✅ **Error Handling** - Try/catch with logging
✅ **Environment Config** - 12-factor app
✅ **API Documentation** - Auto-generated Swagger

---

## Version History

### v1.0.0 (Current)
- Initial production release
- 632 chunks support
- OpenAI embeddings
- Qdrant + Postgres integration
- Complete CRUD operations
- Health monitoring

---

**Maintained by**: GIAIC Hackathon I Team
**Documentation**: README.md, ARCHITECTURE.md
**API Docs**: /docs, /redoc
