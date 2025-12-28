# Quick Start Guide

Get the Physical AI RAG Backend running in 5 minutes.

---

## Prerequisites

- Python 3.11+
- OpenAI API key
- Qdrant Cloud credentials (provided)
- Neon Postgres database

---

## 1. Install

```bash
cd backend
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

---

## 2. Configure

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant (provided)
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo

# Neon Postgres
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
```

---

## 3. Run

```bash
uvicorn app.main:app --reload
```

Server runs at: **http://localhost:8000**

---

## 4. Test

```bash
# Health check
curl http://localhost:8000/health

# Expected: {"status": "healthy", ...}
```

---

## 5. Ingest Data

```bash
curl -X POST http://localhost:8000/api/v1/ingest/chunks
```

Wait ~60 seconds. Expected:
```json
{
  "success": true,
  "chunks_processed": 632,
  "chunks_stored_postgres": 632,
  "chunks_stored_qdrant": 632
}
```

---

## 6. Search

```bash
curl -X POST http://localhost:8000/api/v1/search/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "top_k": 3
  }'
```

---

## 🎉 Done!

Your RAG backend is ready.

### Next Steps

- **API Docs**: http://localhost:8000/docs
- **Full README**: [README.md](README.md)
- **Deployment**: [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)
- **Architecture**: [ARCHITECTURE.md](ARCHITECTURE.md)

---

## Troubleshooting

### "Database connection failed"
- Check `POSTGRES_URL` format
- Verify SSL mode: `?sslmode=require`

### "Qdrant connection failed"
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Test: `curl -H "api-key: YOUR_KEY" YOUR_URL/collections`

### "OpenAI error"
- Check API key is valid
- Verify billing is enabled
- Check rate limits

---

**Time to Complete**: 5-10 minutes
**Status**: ✅ Production Ready
