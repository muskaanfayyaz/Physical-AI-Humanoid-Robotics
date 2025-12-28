# 🚀 Quick Start with Gemini API

**Your backend now uses Google Gemini API (100% FREE!)**

---

## ⚡ 3-Step Setup

### Step 1: Install Dependencies (30 seconds)

```bash
cd backend
pip install -r requirements.txt
```

### Step 2: Configure Environment (Already Done!)

Your `.env` file is already configured with:

```env
GEMINI_API_KEY=AIzaSyAqgHNgBERQC1ORCvB7KsZf3TyyXqb29cM
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
```

**Only missing:** Add your Neon Postgres URL (get it from https://neon.tech)

```env
POSTGRES_URL=postgresql://user:pass@ep-xxx.us-west-2.aws.neon.tech/neondb?sslmode=require
```

### Step 3: Run Backend (10 seconds)

```bash
uvicorn app.main:app --reload
```

**Expected output:**
```
INFO:     Initialized Gemini Embedding Service: models/embedding-001
INFO:     Initialized Gemini LLM Service: gemini-1.5-flash
INFO:     Uvicorn running on http://127.0.0.1:8000
```

✅ **You're running on Gemini!**

---

## 🧪 Test It

### 1. Health Check

```bash
curl http://localhost:8000/health
```

### 2. Ingest Chunks (REQUIRED - First Time Only)

```bash
curl -X POST http://localhost:8000/api/v1/ingest/chunks
```

**This will:**
- Generate 768-dim embeddings using Gemini
- Store 632 chunks in Qdrant
- Store metadata in Postgres
- Takes ~10-15 minutes (Gemini rate limits)

### 3. Ask a Question

```bash
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

**Expected response:**
```json
{
  "answer": "ROS 2 is...",
  "model": "gemini-1.5-flash",
  "sources": [...]
}
```

---

## 📊 What Changed?

| Component | Before (OpenAI) | After (Gemini) |
|-----------|----------------|----------------|
| Embeddings | text-embedding-3-small (1536-dim) | models/embedding-001 (768-dim) |
| Chat | gpt-4o-mini | gemini-1.5-flash |
| Cost | $1-2/month | **$0/month (FREE!)** |
| Rate Limit | Pay-per-use | 15 req/min, 1500 req/day |

---

## 🔧 Files Modified

✅ **Updated for Gemini:**
- `app/config.py` - Gemini API key configuration
- `app/services/embeddings.py` - Gemini embeddings (768-dim)
- `app/services/llm_service.py` - Gemini chat (gemini-1.5-flash)
- `requirements.txt` - Added google-generativeai==0.3.2
- `.env` - Your Gemini API key
- `.env.example` - Template with Gemini variables

---

## 💰 Cost: $0/month

**100% FREE stack:**
```
✅ Gemini API (embeddings + chat)     → $0/month
✅ Qdrant Cloud (1GB vectors)         → $0/month
✅ Neon Postgres (10GB)                → $0/month
✅ Render (750 hrs/month)              → $0/month
✅ GitHub Pages (frontend)             → $0/month
────────────────────────────────────────────────
   TOTAL COST                          → $0/month 🎉
```

---

## 📚 Documentation

- **Full migration guide:** See `GEMINI_MIGRATION_COMPLETE.md`
- **Neon Postgres setup:** See `NEON_POSTGRES_SETUP.md`
- **API endpoints:** See `README.md`
- **Deployment:** See `../RENDER_QUICK_START.md`

---

## 🚨 Common Issues

**Issue:** "Invalid API key"
- **Fix:** Check `GEMINI_API_KEY` in `.env`
- Get new key: https://aistudio.google.com/app/apikey

**Issue:** "Dimension mismatch (1536 vs 768)"
- **Fix:** Delete Qdrant collection and re-ingest
- Old collection had OpenAI embeddings (1536-dim)
- New collection needs Gemini embeddings (768-dim)

**Issue:** "Rate limit exceeded"
- **Normal!** Gemini FREE tier: 15 requests/minute
- Wait 1 minute and retry
- Ingestion takes ~10-15 minutes (expected)

---

## ✅ Success Criteria

Your setup is working if:
- ✅ Backend starts without errors
- ✅ Logs show "Initialized Gemini Embedding Service"
- ✅ Logs show "Initialized Gemini LLM Service"
- ✅ Ingest returns 632 chunks with 768 dimensions
- ✅ /ask endpoint returns answers
- ✅ Response includes `"model": "gemini-1.5-flash"`

---

**You're now running a 100% FREE RAG backend! 🚀**

Questions? See full docs in `GEMINI_MIGRATION_COMPLETE.md`
