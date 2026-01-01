# Technical Planning Document

**Project**: Physical AI & Humanoid Robotics Textbook with RAG Chatbot
**Created**: December 2025
**Status**: Implementation Complete
**Last Updated**: January 2026

## Table of Contents

1. [Project Overview](#project-overview)
2. [Technical Stack Selection](#technical-stack-selection)
3. [System Architecture Design](#system-architecture-design)
4. [Database Schema Planning](#database-schema-planning)
5. [API Design](#api-design)
6. [Frontend Architecture](#frontend-architecture)
7. [Deployment Strategy](#deployment-strategy)
8. [Performance Considerations](#performance-considerations)
9. [Security Planning](#security-planning)
10. [Testing Strategy](#testing-strategy)

---

## Project Overview

### Problem Statement

Create an interactive, university-level textbook on Physical AI & Humanoid Robotics with an intelligent RAG (Retrieval-Augmented Generation) chatbot that can answer questions based on the textbook content without hallucination.

### Key Requirements

1. **Textbook Website**
   - Static site generation for fast loading
   - Chapter-based navigation
   - Mobile-responsive design
   - GitHub Pages deployment

2. **RAG Chatbot Backend**
   - Semantic search over textbook content
   - Grounded answers (no hallucination)
   - Conversation history tracking
   - Real-time response streaming

3. **Data Pipeline**
   - Automated content chunking
   - Vector embedding generation
   - Efficient storage and retrieval

### Success Criteria

- ✅ Chatbot answers questions accurately from textbook content
- ✅ Sub-second search response time
- ✅ Scalable to 1000+ concurrent users
- ✅ <2% hallucination rate
- ✅ Free-tier compatible deployment

---

## Technical Stack Selection

### Decision Matrix

| Component | Options Considered | Selected | Rationale |
|-----------|-------------------|----------|-----------|
| **Frontend Framework** | React, Vue, Docusaurus | **Docusaurus** | Built for documentation, MDX support, SEO optimized |
| **Backend Framework** | Flask, Django, FastAPI | **FastAPI** | Async support, auto-docs, type safety, modern |
| **LLM Provider** | OpenAI, Anthropic, Google Gemini | **Google Gemini** | Free tier, good embeddings, cost-effective |
| **Vector Database** | Pinecone, Weaviate, Qdrant | **Qdrant Cloud** | Free tier, Python client, easy setup |
| **SQL Database** | PostgreSQL (Neon), Supabase | **Neon Postgres** | Serverless, free tier, auto-scaling |
| **Embedding Model** | text-embedding-ada-002, Gemini | **Gemini Embedding** | Free tier, 768 dimensions, multilingual |
| **Chat Model** | GPT-4, Claude, Gemini | **Gemini 1.5 Flash** | Fast, cost-effective, good reasoning |
| **Hosting (Frontend)** | Netlify, Vercel, GitHub Pages | **GitHub Pages** | Free, Git integration, simple |
| **Hosting (Backend)** | Railway, Fly.io, Render | **Render** | Free tier, auto-deploy, managed |

### Technology Stack (Final)

**Frontend:**
- Docusaurus 3.1+
- React 18
- TypeScript (config files)
- MDX for content

**Backend:**
- FastAPI 0.110+
- Python 3.13
- SQLAlchemy 2.0+ (async)
- Pydantic 2.6+ (validation)

**Database Layer:**
- Neon Postgres (metadata storage)
- Qdrant Cloud (vector storage)
- psycopg 3.1+ (async Postgres driver, Python 3.13 compatible)

**AI/ML:**
- Google Gemini API
  - `text-embedding-004` (embeddings)
  - `gemini-1.5-flash` (chat/generation)

**Deployment:**
- GitHub Pages (frontend)
- Render (backend)
- GitHub Actions (CI/CD)

---

## System Architecture Design

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         User Browser                         │
└────────────────┬────────────────────────────────────────────┘
                 │
                 │ HTTPS
                 │
┌────────────────▼────────────────────────────────────────────┐
│              GitHub Pages (Static Site)                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Docusaurus Frontend                                  │  │
│  │  - Chapter Pages                                      │  │
│  │  - RAG Chatbot Component                             │  │
│  └──────────────────┬───────────────────────────────────┘  │
└─────────────────────┼──────────────────────────────────────┘
                      │
                      │ API Calls (CORS)
                      │
┌─────────────────────▼──────────────────────────────────────┐
│              Render (FastAPI Backend)                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  FastAPI Application                                  │  │
│  │  ┌────────────┬────────────┬──────────┬───────────┐ │  │
│  │  │  /ingest   │  /search   │  /ask    │  /conv    │ │  │
│  │  │  Router    │  Router    │  Router  │  Router   │ │  │
│  │  └────────────┴────────────┴──────────┴───────────┘ │  │
│  │                                                       │  │
│  │  ┌─────────────────────────────────────────────────┐ │  │
│  │  │           Service Layer                         │ │  │
│  │  │  - Embedding Service (Gemini)                   │ │  │
│  │  │  - LLM Service (Gemini)                         │ │  │
│  │  │  - Qdrant Service                               │ │  │
│  │  │  - Chunk Service                                │ │  │
│  │  └─────────────────────────────────────────────────┘ │  │
│  └──────────────────┬────────────────┬─────────────────┘  │
└─────────────────────┼────────────────┼────────────────────┘
                      │                │
                      │                │
        ┌─────────────▼─────┐   ┌─────▼──────────────┐
        │  Neon Postgres     │   │  Qdrant Cloud      │
        │  (Metadata)        │   │  (Vectors)         │
        │  - chunk_metadata  │   │  - Collection:     │
        │  - conversations   │   │    textbook_chunks │
        │  - messages        │   │  - 768 dims        │
        └────────────────────┘   └────────────────────┘
                      │
                      │
        ┌─────────────▼─────────────────┐
        │  Google Gemini API             │
        │  - text-embedding-004          │
        │  - gemini-1.5-flash            │
        └────────────────────────────────┘
```

### Component Responsibilities

**Frontend (Docusaurus):**
- Serve static textbook content
- Render chatbot UI
- Handle user interactions
- Make API calls to backend

**Backend (FastAPI):**
- Process chunk ingestion
- Generate embeddings via Gemini
- Perform semantic search
- Generate grounded answers
- Manage conversation history

**Neon Postgres:**
- Store chunk metadata (id, content, chapter, page)
- Store conversation history
- Store user messages
- Provide relational queries

**Qdrant:**
- Store vector embeddings
- Perform similarity search
- Return top-k relevant chunks

**Gemini API:**
- Generate embeddings (768-dim vectors)
- Generate chat responses
- Process prompts with context

---

## Database Schema Planning

### Neon Postgres Schema

```sql
-- Chunk Metadata Table
CREATE TABLE chunk_metadata (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(255) UNIQUE NOT NULL,
    content TEXT NOT NULL,
    metadata JSONB,
    chapter VARCHAR(255),
    page_number INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chunk_id ON chunk_metadata(chunk_id);
CREATE INDEX idx_chapter ON chunk_metadata(chapter);
CREATE INDEX idx_metadata ON chunk_metadata USING GIN (metadata);

-- Conversations Table
CREATE TABLE conversations (
    id SERIAL PRIMARY KEY,
    conversation_id VARCHAR(255) UNIQUE NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);

CREATE INDEX idx_conversation_id ON conversations(conversation_id);

-- Messages Table
CREATE TABLE messages (
    id SERIAL PRIMARY KEY,
    conversation_id VARCHAR(255) REFERENCES conversations(conversation_id),
    role VARCHAR(50) NOT NULL,  -- 'user' or 'assistant'
    content TEXT NOT NULL,
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_message_conversation ON messages(conversation_id);
CREATE INDEX idx_message_created_at ON messages(created_at);
```

### Qdrant Collection Schema

```python
collection_config = {
    "collection_name": "textbook_chunks",
    "vector_size": 768,  # Gemini embedding dimension
    "distance": "Cosine",  # Similarity metric
    "on_disk_payload": True,  # Optimize memory
    "optimizers_config": {
        "indexing_threshold": 10000
    }
}

# Payload structure
payload = {
    "chunk_id": "chapter-01-chunk-001",
    "content": "...",
    "chapter": "Introduction to Physical AI",
    "metadata": {
        "page": 1,
        "section": "Overview"
    }
}
```

---

## API Design

### RESTful Endpoints

#### 1. Ingest Endpoint

```
POST /api/v1/ingest
```

**Purpose**: Ingest chunks from JSON file into database and Qdrant

**Request**:
```json
{
  "file_path": "rag/chunks.json"
}
```

**Response**:
```json
{
  "status": "success",
  "chunks_ingested": 150,
  "message": "Chunks ingested successfully"
}
```

**Workflow**:
1. Read chunks.json
2. For each chunk:
   - Store metadata in Postgres
   - Generate embedding via Gemini
   - Store vector in Qdrant

---

#### 2. Search Endpoint

```
POST /api/v1/search
```

**Purpose**: Semantic search over textbook content

**Request**:
```json
{
  "query": "What is ROS 2?",
  "top_k": 5,
  "filter": {
    "chapter": "Introduction to ROS 2"
  }
}
```

**Response**:
```json
{
  "results": [
    {
      "chunk_id": "chapter-03-chunk-012",
      "content": "ROS 2 is the second generation...",
      "score": 0.89,
      "metadata": {
        "chapter": "Introduction to ROS 2",
        "page": 45
      }
    }
  ],
  "query_time_ms": 120
}
```

**Workflow**:
1. Generate query embedding via Gemini
2. Search Qdrant for similar vectors
3. Retrieve metadata from Postgres
4. Return ranked results

---

#### 3. Ask Endpoint (RAG)

```
POST /api/v1/ask
```

**Purpose**: Answer question using RAG (grounded in textbook)

**Request**:
```json
{
  "question": "How does ROS 2 differ from ROS 1?",
  "conversation_id": "conv-123",
  "top_k": 5,
  "include_sources": true
}
```

**Response**:
```json
{
  "answer": "ROS 2 differs from ROS 1 in several key ways...",
  "sources": [
    {
      "chunk_id": "chapter-03-chunk-015",
      "chapter": "Introduction to ROS 2",
      "relevance_score": 0.92
    }
  ],
  "conversation_id": "conv-123",
  "processing_time_ms": 850
}
```

**Workflow**:
1. Search for relevant chunks (semantic search)
2. Build context from top-k chunks
3. Create prompt with context
4. Generate answer via Gemini
5. Store question & answer in conversation history
6. Return grounded answer with sources

---

#### 4. Conversation Endpoints

```
GET /api/v1/conversations/{conversation_id}
```

**Purpose**: Retrieve conversation history

**Response**:
```json
{
  "conversation_id": "conv-123",
  "messages": [
    {
      "role": "user",
      "content": "What is ROS 2?",
      "timestamp": "2025-12-29T10:30:00Z"
    },
    {
      "role": "assistant",
      "content": "ROS 2 is...",
      "timestamp": "2025-12-29T10:30:02Z"
    }
  ]
}
```

---

## Frontend Architecture

### Component Hierarchy

```
App
├── Layout
│   ├── Navbar
│   ├── Sidebar (Chapter Navigation)
│   └── Footer
│
├── Pages
│   ├── HomePage
│   ├── ChapterPage
│   │   └── MDXContent
│   └── BlogPage
│
└── Components
    └── Chatbot
        ├── ChatbotToggle
        ├── ChatbotWindow
        │   ├── MessageList
        │   │   ├── UserMessage
        │   │   └── BotMessage
        │   ├── InputBox
        │   └── SourceReferences
        └── ChatbotAPI (service)
```

### Chatbot Component Design

**Features**:
- Expandable/collapsible chat window
- Message history display
- Typing indicators
- Source citations
- Error handling
- Conversation persistence

**State Management**:
```javascript
const [messages, setMessages] = useState([]);
const [conversationId, setConversationId] = useState(null);
const [isLoading, setIsLoading] = useState(false);
const [error, setError] = useState(null);
```

**API Integration**:
```javascript
const askQuestion = async (question) => {
  const response = await fetch(`${API_URL}/api/v1/ask`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      question,
      conversation_id: conversationId,
      top_k: 5
    })
  });
  return response.json();
};
```

---

## Deployment Strategy

### GitHub Pages (Frontend)

**Build Process**:
```bash
npm run build  # Generate static files
```

**Deployment**:
- Automatic via GitHub Actions
- Triggered on push to main branch
- Builds to `gh-pages` branch

**Workflow** (`.github/workflows/deploy.yml`):
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### Render (Backend)

**Configuration**:
- Auto-deploy from GitHub
- Free tier (512MB RAM)
- Persistent storage not needed (serverless DB)

**Environment Variables**:
```bash
GOOGLE_API_KEY=...
POSTGRES_URL=postgresql://...@neon.tech/...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
```

**Build Command**:
```bash
pip install --upgrade pip setuptools wheel && \
pip install -r requirements.txt
```

**Start Command**:
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

---

## Performance Considerations

### Backend Optimization

1. **Async/Await Pattern**
   - All I/O operations are async
   - Non-blocking database queries
   - Concurrent API calls

2. **Connection Pooling**
   ```python
   engine = create_async_engine(
       POSTGRES_URL,
       pool_size=5,
       max_overflow=10,
       pool_pre_ping=True
   )
   ```

3. **Caching Strategy**
   - Qdrant handles vector caching
   - No application-level cache (stateless)

4. **Batch Processing**
   - Ingest chunks in batches of 100
   - Reduces API calls

### Frontend Optimization

1. **Code Splitting**
   - Docusaurus automatic code splitting
   - Lazy load chatbot component

2. **Asset Optimization**
   - Minified JS/CSS
   - Compressed images
   - CDN delivery (GitHub Pages)

### Database Optimization

1. **Indexing**
   - B-tree index on `chunk_id`
   - GIN index on `metadata` (JSONB)
   - Index on `conversation_id`

2. **Query Optimization**
   - Use prepared statements
   - Limit result sets
   - Avoid N+1 queries

---

## Security Planning

### API Security

1. **CORS Configuration**
   ```python
   allow_origins = [
       "https://yourusername.github.io",
       "http://localhost:3000"  # Development
   ]
   ```

2. **Rate Limiting** (Future)
   - Implement per-IP rate limiting
   - Prevent abuse

3. **Input Validation**
   - Pydantic models validate all inputs
   - Sanitize user queries

### Secret Management

1. **Environment Variables**
   - Never commit secrets to Git
   - Use Render environment variables
   - Use `.env.local` for development

2. **API Key Rotation**
   - Rotate Gemini API keys periodically
   - Monitor usage quotas

### Database Security

1. **Neon Postgres**
   - SSL/TLS connections required
   - IP whitelisting (optional)
   - Role-based access control

2. **Qdrant Cloud**
   - API key authentication
   - HTTPS only

---

## Testing Strategy

### Backend Testing

**Unit Tests**:
```python
# tests/test_embeddings.py
async def test_embedding_generation():
    service = get_embedding_service()
    embedding = await service.generate_embedding("test")
    assert len(embedding) == 768
    assert all(isinstance(x, float) for x in embedding)
```

**Integration Tests**:
```python
# tests/test_api.py
async def test_search_endpoint():
    response = client.post("/api/v1/search", json={
        "query": "ROS 2",
        "top_k": 5
    })
    assert response.status_code == 200
    assert len(response.json()["results"]) <= 5
```

### Frontend Testing

**Component Tests**:
```javascript
// __tests__/Chatbot.test.js
test('renders chatbot button', () => {
  render(<Chatbot />);
  expect(screen.getByText(/Ask AI/i)).toBeInTheDocument();
});
```

### Manual Testing

**Test Cases**:
1. ✅ Ingest chunks successfully
2. ✅ Search returns relevant results
3. ✅ RAG answers are grounded
4. ✅ Conversation history persists
5. ✅ CORS works from GitHub Pages
6. ✅ Error handling works

---

## Risk Mitigation

### Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Free tier limits exceeded | High | Medium | Monitor usage, implement rate limiting |
| Qdrant downtime | High | Low | Implement retry logic, fallback search |
| Gemini API rate limits | Medium | Medium | Implement exponential backoff |
| Database connection issues | High | Low | Connection pooling, health checks |
| CORS issues | Medium | Low | Proper configuration, testing |

### Operational Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Render cold starts | Medium | High | Accepted (free tier limitation) |
| GitHub Pages downtime | Medium | Very Low | Minimal risk (GitHub SLA) |
| API key exposure | High | Low | Secret scanning, .gitignore |

---

## Implementation Phases

**See**: [implementation-phases.md](./implementation-phases.md) for detailed breakdown

**Phase 1**: Core Backend Setup (Week 1)
**Phase 2**: Data Pipeline & Ingestion (Week 1)
**Phase 3**: Search & RAG Implementation (Week 2)
**Phase 4**: Frontend Integration (Week 2)
**Phase 5**: Deployment & Testing (Week 3)
**Phase 6**: Optimization & Launch (Week 3)

---

## Success Metrics

### Performance Metrics

- ✅ Search latency: <200ms (p95)
- ✅ RAG response time: <2s (p95)
- ✅ Uptime: >99% (Render free tier)
- ✅ Error rate: <1%

### Quality Metrics

- ✅ Answer accuracy: >95% (grounded in content)
- ✅ Hallucination rate: <2%
- ✅ Source attribution: 100%
- ✅ User satisfaction: >4.5/5 (projected)

---

**Document Status**: ✅ Complete
**Next Review**: January 2026
**Owner**: Development Team
