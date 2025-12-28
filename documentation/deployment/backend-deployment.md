# Backend Deployment Guide

Step-by-step guide to deploy the Physical AI RAG Backend.

---

## Prerequisites Checklist

Before deploying, ensure you have:

- [ ] OpenAI API key
- [ ] Qdrant Cloud account (Free tier is sufficient)
- [ ] Neon Postgres database (Free tier is sufficient)
- [ ] GitHub repository (for platform deployments)
- [ ] Python 3.11+ (for local development)

---

## Step 1: Set Up Databases

### Qdrant Cloud

1. **Create Account**
   - Visit: https://cloud.qdrant.io/
   - Sign up for free tier

2. **Create Cluster**
   - Click "Create Cluster"
   - Select free tier (1GB storage)
   - Choose region: `europe-west3` (or closest to you)

3. **Get Credentials**
   ```
   QDRANT_URL: https://xxx.gcp.cloud.qdrant.io
   QDRANT_API_KEY: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
   ```

### Neon Postgres

1. **Create Account**
   - Visit: https://neon.tech/
   - Sign up for free tier

2. **Create Project**
   - Name: `physical-ai-rag`
   - Region: Select closest to your users

3. **Get Connection String**
   ```
   POSTGRES_URL: postgresql://username:password@host.neon.tech/dbname?sslmode=require
   ```

---

## Step 2: Local Development Setup

### Clone & Install

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Configure Environment

```bash
# Copy example
cp .env.example .env

# Edit with your credentials
nano .env  # or use any text editor
```

Required `.env` variables:
```env
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.gcp.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-key
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
```

### Test Locally

```bash
# Start server
uvicorn app.main:app --reload

# In another terminal, test health
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "database_connected": true,
  "qdrant_connected": true,
  "openai_configured": true
}
```

---

## Step 3: Ingest Data

### Load Chunks into System

```bash
# Make sure server is running
curl -X POST http://localhost:8000/api/v1/ingest/chunks
```

This will:
1. Load 632 chunks from `../rag/chunks.json`
2. Generate embeddings via OpenAI (~45-60 seconds)
3. Store metadata in Postgres
4. Store vectors in Qdrant

Expected output:
```json
{
  "success": true,
  "chunks_processed": 632,
  "chunks_stored_postgres": 632,
  "chunks_stored_qdrant": 632,
  "processing_time_seconds": 52.3
}
```

### Verify Ingestion

```bash
# Check stats
curl http://localhost:8000/api/v1/ingest/stats

# Test search
curl -X POST http://localhost:8000/api/v1/search/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "top_k": 3}'
```

---

## Step 4: Production Deployment

### Option 1: Railway (Recommended)

**Advantages**: Auto-deploy from GitHub, free tier, easy setup

1. **Install Railway CLI**
   ```bash
   npm install -g @railway/cli
   railway login
   ```

2. **Initialize Project**
   ```bash
   cd backend
   railway init
   ```

3. **Add Environment Variables**
   ```bash
   railway variables set OPENAI_API_KEY=sk-...
   railway variables set QDRANT_URL=https://...
   railway variables set QDRANT_API_KEY=...
   railway variables set POSTGRES_URL=postgresql://...
   ```

4. **Deploy**
   ```bash
   railway up
   ```

5. **Get URL**
   ```bash
   railway domain
   # Returns: https://your-app.railway.app
   ```

---

### Option 2: Render

**Advantages**: Free tier, GitHub auto-deploy, PostgreSQL included

1. **Create Account**
   - Visit: https://render.com/
   - Connect GitHub

2. **Create Web Service**
   - Click "New +" → "Web Service"
   - Select your repository
   - Root directory: `backend`

3. **Configure Build**
   ```
   Build Command: pip install -r requirements.txt
   Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```

4. **Add Environment Variables**
   - Go to "Environment" tab
   - Add all variables from `.env`

5. **Deploy**
   - Click "Create Web Service"
   - Wait 2-3 minutes
   - URL: `https://your-app.onrender.com`

---

### Option 3: Fly.io

**Advantages**: Edge deployment, global distribution

1. **Install Flyctl**
   ```bash
   curl -L https://fly.io/install.sh | sh
   fly auth login
   ```

2. **Create `fly.toml`**
   ```toml
   app = "physical-ai-rag"

   [build]
     builder = "paketobuildpacks/builder:base"

   [[services]]
     internal_port = 8000
     protocol = "tcp"

     [[services.ports]]
       port = 80
       handlers = ["http"]

     [[services.ports]]
       port = 443
       handlers = ["tls", "http"]
   ```

3. **Deploy**
   ```bash
   fly launch
   fly secrets set OPENAI_API_KEY=sk-...
   fly secrets set QDRANT_URL=https://...
   fly deploy
   ```

---

### Option 4: Docker + Cloud Run

**Advantages**: Containerized, serverless, auto-scaling

1. **Create `Dockerfile`**
   ```dockerfile
   FROM python:3.11-slim

   WORKDIR /app

   COPY requirements.txt .
   RUN pip install --no-cache-dir -r requirements.txt

   COPY app/ ./app/
   COPY ../rag/chunks.json ../rag/chunks.json

   EXPOSE 8000

   CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

2. **Build Image**
   ```bash
   docker build -t physical-ai-rag .
   docker run -p 8000:8000 --env-file .env physical-ai-rag
   ```

3. **Deploy to Cloud Run**
   ```bash
   gcloud builds submit --tag gcr.io/PROJECT_ID/physical-ai-rag
   gcloud run deploy physical-ai-rag \
     --image gcr.io/PROJECT_ID/physical-ai-rag \
     --platform managed \
     --set-env-vars OPENAI_API_KEY=sk-...,...
   ```

---

## Step 5: Post-Deployment

### Update CORS Origins

In `.env` or platform environment variables:
```env
CORS_ORIGINS=["https://your-frontend.vercel.app","https://muskaanfayyaz.github.io"]
```

### Run Ingestion on Production

```bash
# Get your production URL
BACKEND_URL=https://your-app.railway.app

# Ingest chunks
curl -X POST $BACKEND_URL/api/v1/ingest/chunks
```

### Test Production API

```bash
# Health check
curl $BACKEND_URL/health

# Search test
curl -X POST $BACKEND_URL/api/v1/search/ \
  -H "Content-Type: application/json" \
  -d '{"query": "humanoid robotics", "top_k": 5}'
```

---

## Step 6: Frontend Integration

### Update Frontend Config

In your Docusaurus/React app:

```javascript
// config.js
export const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend.railway.app/api/v1'
  : 'http://localhost:8000/api/v1';
```

### Example Search Call

```javascript
async function searchTextbook(query) {
  const response = await fetch(`${API_BASE_URL}/search/`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query: query,
      top_k: 5,
      similarity_threshold: 0.7
    })
  });

  const data = await response.json();
  return data.results;
}
```

---

## Monitoring & Maintenance

### Health Checks

Set up monitoring to ping `/health` every 5 minutes:
- **UptimeRobot**: Free tier, email alerts
- **Cronitor**: Cron job monitoring
- **Better Uptime**: Status pages

### Cost Estimation (Free Tiers)

| Service | Free Tier | Usage |
|---------|-----------|-------|
| **Qdrant** | 1GB storage | ~10MB for 632 chunks |
| **Neon Postgres** | 10GB storage | ~100MB |
| **OpenAI** | Pay-per-use | ~$0.50 for ingestion |
| **Railway/Render** | 500 hrs/month | Enough for demo |

### Scaling Considerations

When you outgrow free tiers:
1. **OpenAI**: Upgrade to tier 2+ for higher rate limits
2. **Qdrant**: Scale to 4GB ($25/month)
3. **Neon**: Upgrade for more connections ($19/month)
4. **Hosting**: Scale to multiple workers ($7-20/month)

---

## Troubleshooting

### Issue: Ingestion Fails

**Symptoms**: 500 error on `/ingest/chunks`

**Solutions**:
1. Check OpenAI API key is valid
2. Verify `chunks.json` path is correct
3. Check OpenAI rate limits
4. Review server logs

### Issue: Search Returns No Results

**Symptoms**: Empty results array

**Solutions**:
1. Verify ingestion completed successfully
2. Check Qdrant collection exists
3. Try lowering `similarity_threshold`
4. Test with simpler queries

### Issue: Database Connection Error

**Symptoms**: `database_connected: false` in health check

**Solutions**:
1. Verify `POSTGRES_URL` format
2. Check SSL mode is `require`
3. Test connection with `psql`
4. Check Neon project is active

---

## Security Checklist

Before going live:

- [ ] All secrets in environment variables (not hardcoded)
- [ ] `.env` file is git-ignored
- [ ] CORS origins restricted to your domains
- [ ] HTTPS enabled (automatic on Railway/Render)
- [ ] Database uses SSL/TLS
- [ ] Rate limiting configured (optional)
- [ ] API key rotation plan (quarterly)

---

## Rollback Procedure

If deployment fails:

1. **Check Logs**
   ```bash
   railway logs  # or render logs, fly logs
   ```

2. **Rollback Version**
   ```bash
   railway rollback
   ```

3. **Verify Health**
   ```bash
   curl https://your-app.railway.app/health
   ```

---

## Next Steps

After successful deployment:

1. **Test all endpoints** with Postman or curl
2. **Ingest production data** via `/ingest/chunks`
3. **Update frontend** to use production URL
4. **Set up monitoring** for uptime
5. **Document API** for your team
6. **Test search quality** with real queries

---

## Support Resources

- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Neon Docs**: https://neon.tech/docs/introduction
- **Railway Docs**: https://docs.railway.app/
- **OpenAI Docs**: https://platform.openai.com/docs

---

**Deployment Status**: ✅ Production Ready

**Estimated Setup Time**: 30-45 minutes
**Required Skills**: Basic terminal, API knowledge
**Cost**: Free tier for demo/hackathon

---

🎉 **Congratulations!** Your RAG backend is now live and ready for the hackathon!
