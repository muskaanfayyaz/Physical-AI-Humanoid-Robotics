# Quick Deploy Guide

Ultra-condensed deployment steps for experienced developers.

---

## Prerequisites

```bash
# Install tools
npm install -g @railway/cli
curl -fsSL https://railway.app/install.sh | sh

# Get credentials
# 1. OpenAI API key: https://platform.openai.com/api-keys
# 2. Neon Postgres: https://neon.tech (create database)
# 3. Qdrant: Already provided below
```

---

## 1. Deploy Backend (5 minutes)

```bash
# Navigate to backend
cd backend

# Login to Railway
railway login

# Initialize project
railway init
# Name: physical-ai-backend

# Set environment variables
railway variables set \
  OPENAI_API_KEY="sk-your-key-here" \
  QDRANT_URL="https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io" \
  QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo" \
  POSTGRES_URL="postgresql://user:pass@host.neon.tech/db?sslmode=require" \
  CORS_ORIGINS='["https://muskaanfayyaz.github.io","http://localhost:3000"]'

# Create Railway config
cat > railway.json << 'EOF'
{
  "build": { "builder": "NIXPACKS" },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE"
  }
}
EOF

# Deploy
railway up

# Get URL
BACKEND_URL=$(railway domain)
echo "Backend URL: $BACKEND_URL"

# Ingest chunks
curl -X POST $BACKEND_URL/api/v1/ingest/chunks

# Test
curl $BACKEND_URL/health
curl -X POST $BACKEND_URL/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

---

## 2. Configure Frontend (2 minutes)

```bash
cd ../book

# Edit docusaurus.config.js
# Add this to module.exports:
```

```javascript
customFields: {
  chatbotApiUrl: 'https://your-backend.railway.app/api/v1',
}
```

Add chatbot to chapter:

```mdx
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

---

## 3. Deploy Frontend (3 minutes)

```bash
# Build
npm run build

# Test locally
npm run serve

# Deploy to GitHub Pages
git add .
git commit -m "Deploy with chatbot"
git push origin main

# GitHub Actions will auto-deploy
# Or manual deploy:
GIT_USER=muskaanfayyaz npm run deploy
```

---

## 4. Verify

```bash
# Backend
curl https://your-backend.railway.app/health
curl https://your-backend.railway.app/api/v1/ingest/stats

# Frontend
# Visit: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/
# Click chatbot (💬)
# Ask: "What is Physical AI?"
```

---

## Environment Variables Cheat Sheet

**Backend (.env or Railway):**
```env
OPENAI_API_KEY=sk-proj-xxx
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]
```

**Frontend (docusaurus.config.js):**
```javascript
customFields: {
  chatbotApiUrl: 'https://your-backend.railway.app/api/v1',
}
```

---

## Troubleshooting

**CORS Error:**
```bash
railway variables set CORS_ORIGINS='["https://muskaanfayyaz.github.io"]'
railway restart
```

**Backend Not Responding:**
```bash
railway logs
railway restart
```

**Chunks Not Ingested:**
```bash
curl -X POST $(railway domain)/api/v1/ingest/chunks
```

**Frontend Not Updating:**
```bash
npm run clear
npm run build
git push origin main
```

---

## One-Liner Deployment (All-in-One)

```bash
# Backend
cd backend && \
railway login && \
railway init && \
railway variables set \
  OPENAI_API_KEY="sk-xxx" \
  QDRANT_URL="https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io" \
  QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo" \
  POSTGRES_URL="postgresql://user:pass@host.neon.tech/db?sslmode=require" \
  CORS_ORIGINS='["https://muskaanfayyaz.github.io"]' && \
railway up && \
curl -X POST $(railway domain)/api/v1/ingest/chunks

# Frontend (update config first!)
cd ../book && \
npm run build && \
git add . && \
git commit -m "Deploy" && \
git push origin main
```

---

**Total Time:** 10-15 minutes
**Cost:** FREE (except OpenAI ~$1/month)
**Status:** ✅ Production Ready
