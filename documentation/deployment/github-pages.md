# Complete Deployment Guide

Step-by-step guide to deploy the Physical AI RAG system (Backend + Frontend).

---

## Overview

**What We're Deploying:**
1. FastAPI Backend → Railway (or Render/Fly.io)
2. Docusaurus Frontend → GitHub Pages
3. Connect Chatbot → Backend API

**Time Required:** 30-45 minutes

---

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] GitHub account
- [ ] OpenAI API key ([Get here](https://platform.openai.com/api-keys))
- [ ] Neon Postgres database ([Create here](https://neon.tech))
- [ ] Qdrant Cloud account (already provided in credentials)
- [ ] Git installed locally
- [ ] Node.js 18+ installed
- [ ] Python 3.11+ installed

---

## Part 1: Deploy FastAPI Backend

### Option A: Railway (Recommended - Easiest)

#### Step 1.1: Create Railway Account

1. Visit: https://railway.app/
2. Click **"Start a New Project"**
3. Sign up with GitHub

#### Step 1.2: Install Railway CLI

```bash
# macOS/Linux
curl -fsSL https://railway.app/install.sh | sh

# Windows (PowerShell)
iwr https://railway.app/install.ps1 | iex

# Verify installation
railway --version
```

#### Step 1.3: Login to Railway

```bash
railway login
```

This opens a browser window to authenticate.

#### Step 1.4: Initialize Railway Project

```bash
# Navigate to backend directory
cd /mnt/d/physical-ai-textbook/backend

# Initialize Railway
railway init

# You'll be prompted:
# ✔ Enter project name: physical-ai-backend
# ✔ Select environment: production
```

#### Step 1.5: Set Environment Variables

```bash
# Set OpenAI API Key
railway variables set OPENAI_API_KEY=sk-your-actual-openai-key-here

# Set Qdrant credentials (already provided)
railway variables set QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io

railway variables set QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo

# Set Neon Postgres URL (get from Neon dashboard)
railway variables set POSTGRES_URL=postgresql://username:password@ep-xxx.neon.tech/dbname?sslmode=require

# Set CORS origins (update with your GitHub Pages URL)
railway variables set CORS_ORIGINS='["https://muskaanfayyaz.github.io","http://localhost:3000"]'
```

**Important:** Replace with your actual values:
- `OPENAI_API_KEY`: Your OpenAI API key
- `POSTGRES_URL`: Your Neon database connection string

#### Step 1.6: Create Railway Configuration

Create `railway.json` in the backend directory:

```bash
cat > railway.json << 'EOF'
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
EOF
```

#### Step 1.7: Create Nixpacks Configuration

Create `nixpacks.toml`:

```bash
cat > nixpacks.toml << 'EOF'
[phases.setup]
nixPkgs = ["python311"]

[phases.install]
cmds = ["pip install -r requirements.txt"]

[start]
cmd = "uvicorn app.main:app --host 0.0.0.0 --port $PORT"
EOF
```

#### Step 1.8: Deploy to Railway

```bash
# Deploy
railway up

# This will:
# 1. Upload your code
# 2. Install dependencies
# 3. Start the server
```

#### Step 1.9: Get Your Deployment URL

```bash
# Generate a public domain
railway domain

# Output will be something like:
# https://physical-ai-backend-production.up.railway.app
```

**Copy this URL!** You'll need it for the frontend.

#### Step 1.10: Verify Backend Deployment

```bash
# Get your Railway URL
BACKEND_URL=$(railway domain)

# Test health endpoint
curl $BACKEND_URL/health

# Expected output:
# {
#   "status": "healthy",
#   "app_name": "Physical AI RAG Backend",
#   "version": "1.0.0",
#   "database_connected": true,
#   "qdrant_connected": true,
#   "openai_configured": true
# }
```

#### Step 1.11: Ingest Chunks (IMPORTANT!)

```bash
# Get your Railway URL
BACKEND_URL=$(railway domain)

# Ingest the 632 chunks into the system
curl -X POST $BACKEND_URL/api/v1/ingest/chunks

# This will take ~60 seconds
# Expected output:
# {
#   "success": true,
#   "chunks_processed": 632,
#   "chunks_stored_postgres": 632,
#   "chunks_stored_qdrant": 632,
#   "processing_time_seconds": 52.3
# }
```

#### Step 1.12: Test the Ask Endpoint

```bash
# Test RAG Q&A
curl -X POST $BACKEND_URL/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Should return an AI-generated answer with sources
```

✅ **Backend deployment complete!**

---

### Option B: Render (Alternative)

#### Step B.1: Create Render Account

1. Visit: https://render.com/
2. Sign up with GitHub

#### Step B.2: Create New Web Service

1. Click **"New +"** → **"Web Service"**
2. Connect your GitHub repository
3. Select the repository: `Physical-AI-Humanoid-Robotics`
4. Configure:
   - **Name:** `physical-ai-backend`
   - **Region:** Oregon (US West)
   - **Root Directory:** `backend`
   - **Runtime:** Python 3
   - **Build Command:** `pip install -r requirements.txt`
   - **Start Command:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

#### Step B.3: Add Environment Variables

In Render dashboard, go to **"Environment"** tab and add:

```
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]
```

#### Step B.4: Deploy

1. Click **"Create Web Service"**
2. Wait 2-3 minutes for deployment
3. Copy your URL: `https://physical-ai-backend.onrender.com`

#### Step B.5: Ingest Chunks

```bash
# Replace with your Render URL
BACKEND_URL=https://physical-ai-backend.onrender.com

curl -X POST $BACKEND_URL/api/v1/ingest/chunks
```

---

### Option C: Fly.io (Advanced)

#### Step C.1: Install Flyctl

```bash
# macOS/Linux
curl -L https://fly.io/install.sh | sh

# Windows
iwr https://fly.io/install.ps1 -useb | iex
```

#### Step C.2: Login

```bash
fly auth login
```

#### Step C.3: Launch App

```bash
cd backend

fly launch \
  --name physical-ai-backend \
  --region sjc \
  --no-deploy
```

#### Step C.4: Set Secrets

```bash
fly secrets set \
  OPENAI_API_KEY=sk-your-key-here \
  QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io \
  QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo \
  POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
```

#### Step C.5: Deploy

```bash
fly deploy
```

---

## Part 2: Get Database Credentials

### Step 2.1: Set Up Neon Postgres

1. Visit: https://neon.tech/
2. Click **"Sign up"** (or login)
3. Create new project:
   - **Name:** `physical-ai-db`
   - **Region:** Select closest to your backend
   - **Postgres Version:** 16

4. **Copy connection string:**
   ```
   postgresql://username:password@ep-xxx-123.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

5. Update backend environment variable:
   ```bash
   railway variables set POSTGRES_URL="postgresql://username:password@ep-xxx.neon.tech/neondb?sslmode=require"
   ```

### Step 2.2: Verify Qdrant Cloud

Qdrant credentials are already provided:
- **URL:** `https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io`
- **API Key:** `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo`

You can verify at: https://cloud.qdrant.io/

---

## Part 3: Connect Frontend to Backend

### Step 3.1: Update Docusaurus Configuration

Edit `book/docusaurus.config.js`:

```javascript
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  favicon: 'img/favicon.ico',

  url: 'https://muskaanfayyaz.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  organizationName: 'muskaanfayyaz',
  projectName: 'Physical-AI-Humanoid-Robotics',

  // ... other config

  // ADD THIS:
  customFields: {
    // Replace with your actual Railway/Render URL
    chatbotApiUrl: process.env.NODE_ENV === 'production'
      ? 'https://your-backend.railway.app/api/v1'  // ← UPDATE THIS
      : 'http://localhost:8000/api/v1',
  },

  // ... rest of config
};
```

**IMPORTANT:** Replace `https://your-backend.railway.app/api/v1` with your actual backend URL from Step 1.9.

Examples:
- Railway: `https://physical-ai-backend-production.up.railway.app/api/v1`
- Render: `https://physical-ai-backend.onrender.com/api/v1`
- Fly.io: `https://physical-ai-backend.fly.dev/api/v1`

### Step 3.2: Add Chatbot to a Chapter

Edit any chapter file, e.g., `book/docs/intro.md`:

```mdx
# Welcome to Physical AI & Humanoid Robotics

This textbook covers...

---

import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

### Step 3.3: Test Locally First

```bash
# Terminal 1: Backend (if testing locally)
cd backend
uvicorn app.main:app --reload

# Terminal 2: Frontend
cd book
npm start
```

Visit: http://localhost:3000

**Test:**
1. Click chatbot button (💬)
2. Ask: "What is Physical AI?"
3. Verify answer appears

### Step 3.4: Build for Production

```bash
cd book

# Clean previous builds
npm run clear

# Build
npm run build

# Test production build locally
npm run serve
```

Visit: http://localhost:3000

**Test the production build:**
1. Open browser console (F12)
2. Check for errors
3. Test chatbot with backend URL

---

## Part 4: Deploy Frontend to GitHub Pages

### Step 4.1: Update Backend CORS

**CRITICAL:** Add your GitHub Pages URL to CORS origins.

```bash
# Railway
railway variables set CORS_ORIGINS='["https://muskaanfayyaz.github.io","http://localhost:3000"]'

# Or edit backend/.env and redeploy
```

Backend `.env`:
```env
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]
```

### Step 4.2: Commit Changes

```bash
# Add all changes
git add .

# Commit
git commit -m "Add chatbot integration with deployed backend

- Configure chatbot API URL
- Add ChatbotWrapper to chapters
- Update CORS settings"

# Push to main branch
git push origin main
```

### Step 4.3: Deploy to GitHub Pages

#### Option 1: Manual Deploy

```bash
cd book

# Set Git user (for deployment)
GIT_USER=muskaanfayyaz npm run deploy
```

#### Option 2: GitHub Actions (Automatic)

Already configured in `.github/workflows/deploy.yml`!

Just push to main:
```bash
git push origin main
```

GitHub Actions will automatically:
1. Build the site
2. Deploy to `gh-pages` branch
3. Update GitHub Pages

### Step 4.4: Enable GitHub Pages

1. Go to: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/settings/pages
2. Under **"Source"**, select:
   - Branch: `gh-pages`
   - Folder: `/` (root)
3. Click **"Save"**
4. Wait 2-3 minutes

### Step 4.5: Get Your Site URL

GitHub Pages URL:
```
https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/
```

Visit this URL to see your deployed site!

---

## Part 5: Verification & Testing

### Step 5.1: Test Backend Endpoints

```bash
# Set your backend URL
BACKEND_URL=https://your-backend.railway.app

# 1. Health check
curl $BACKEND_URL/health

# 2. Check ingestion stats
curl $BACKEND_URL/api/v1/ingest/stats

# 3. Test search
curl -X POST $BACKEND_URL/api/v1/search/ \
  -H "Content-Type: application/json" \
  -d '{"query": "ROS 2", "top_k": 3}'

# 4. Test ask endpoint
curl -X POST $BACKEND_URL/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?"}'
```

### Step 5.2: Test Frontend

Visit: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/

**Manual Tests:**

1. **Chatbot Appears**
   - [ ] Floating button (💬) visible in bottom-right

2. **Normal Mode**
   - [ ] Click button → chat opens
   - [ ] Type: "What is ROS 2?" → send
   - [ ] Answer appears with sources
   - [ ] Sources are clickable

3. **Selected Text Mode**
   - [ ] Select text on page → chat auto-opens
   - [ ] Badge shows "Selected Text Mode"
   - [ ] Ask question → answer from selection

4. **Error Handling**
   - [ ] If backend offline → error message shows
   - [ ] Can clear conversation
   - [ ] Can close/reopen chatbot

5. **Mobile Test**
   - [ ] Open on phone
   - [ ] Chatbot responsive
   - [ ] Full-screen on mobile

6. **Dark Mode**
   - [ ] Toggle dark mode (top-right)
   - [ ] Chatbot styling looks good

### Step 5.3: Check Browser Console

Open browser DevTools (F12):

**Look for:**
- ✅ No errors in console
- ✅ API requests succeed (Network tab)
- ✅ Responses return in 1-2 seconds

**Common Issues:**
- ❌ CORS error → Update backend CORS_ORIGINS
- ❌ 404 error → Check API URL in config
- ❌ 500 error → Check backend logs

---

## Part 6: Troubleshooting

### Issue 1: CORS Error

**Symptom:**
```
Access to fetch at 'https://backend.railway.app/api/v1/ask/' from origin 'https://muskaanfayyaz.github.io' has been blocked by CORS policy
```

**Solution:**
```bash
# Update backend CORS
railway variables set CORS_ORIGINS='["https://muskaanfayyaz.github.io","http://localhost:3000"]'

# Or in backend/.env:
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]

# Restart backend
railway up --detach
```

### Issue 2: Backend Not Responding

**Check backend logs:**
```bash
# Railway
railway logs

# Render
# Go to dashboard → Logs tab
```

**Common fixes:**
- Restart: `railway restart`
- Redeploy: `railway up`
- Check environment variables: `railway variables`

### Issue 3: Chatbot Shows "Backend connection failed"

**Verify API URL:**
```bash
# Check what's in config
cat book/docusaurus.config.js | grep chatbotApiUrl

# Should be:
# chatbotApiUrl: 'https://your-backend.railway.app/api/v1'
```

**Test manually:**
```bash
curl https://your-backend.railway.app/health
```

### Issue 4: No Chunks Found

**Symptom:** Search returns empty, ask gives generic error

**Solution:** Ingest chunks again
```bash
BACKEND_URL=https://your-backend.railway.app

curl -X POST $BACKEND_URL/api/v1/ingest/chunks
```

### Issue 5: GitHub Pages Not Updating

**Solutions:**
1. Check GitHub Actions: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/actions
2. Look for failed workflows
3. Re-run failed jobs
4. Clear browser cache (Ctrl+Shift+R)

---

## Part 7: Environment Variables Summary

### Backend Environment Variables

Required for Railway/Render/Fly.io:

```bash
# OpenAI (REQUIRED)
OPENAI_API_KEY=sk-proj-xxx...  # Get from platform.openai.com

# Qdrant Cloud (PROVIDED)
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo

# Neon Postgres (REQUIRED - get from neon.tech)
POSTGRES_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname?sslmode=require

# CORS (REQUIRED)
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]

# Optional
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
EMBEDDING_DIMENSIONS=1536
DEBUG=False
```

### Frontend Configuration

In `book/docusaurus.config.js`:

```javascript
customFields: {
  chatbotApiUrl: process.env.NODE_ENV === 'production'
    ? 'https://your-backend.railway.app/api/v1'  // Production URL
    : 'http://localhost:8000/api/v1',            // Local dev
}
```

---

## Part 8: Deployment Checklist

### Pre-Deployment

- [ ] OpenAI API key obtained
- [ ] Neon Postgres database created
- [ ] Qdrant credentials verified
- [ ] Backend tested locally
- [ ] Frontend tested locally
- [ ] Chatbot works with local backend

### Backend Deployment

- [ ] Railway/Render account created
- [ ] Backend repository connected
- [ ] Environment variables set
- [ ] Backend deployed successfully
- [ ] Health check returns "healthy"
- [ ] Chunks ingested (632 chunks)
- [ ] Ask endpoint tested
- [ ] Backend URL copied

### Frontend Deployment

- [ ] Docusaurus config updated with backend URL
- [ ] Chatbot added to chapters
- [ ] CORS configured in backend
- [ ] Production build tested locally
- [ ] Changes committed to Git
- [ ] Pushed to GitHub
- [ ] GitHub Pages enabled
- [ ] Site deployed successfully
- [ ] Chatbot works on live site

### Post-Deployment

- [ ] All chatbot modes tested
- [ ] Sources link correctly
- [ ] Mobile responsive works
- [ ] Dark mode looks good
- [ ] No console errors
- [ ] Performance acceptable (<2s responses)

---

## Part 9: Cost Estimation

### Backend Hosting

**Railway (Free Tier):**
- 500 hours/month
- $5 credit/month
- **Cost:** FREE for demo

**Render (Free Tier):**
- 750 hours/month
- Sleeps after 15min inactivity
- **Cost:** FREE

### Database

**Neon Postgres (Free Tier):**
- 10GB storage
- 100 hours compute/month
- **Cost:** FREE

**Qdrant Cloud (Free Tier):**
- 1GB storage
- **Cost:** FREE

### API Usage

**OpenAI:**
- Embeddings: ~$0.50 for initial ingestion (632 chunks)
- GPT-4o-mini: ~$0.001 per question
- **Estimated:** $1-2/month for demo usage

**Total Monthly Cost:** $1-2 (OpenAI only)

---

## Part 10: Monitoring

### Check Backend Health

```bash
# Scheduled health check
curl https://your-backend.railway.app/health

# Expected:
# {"status":"healthy","database_connected":true,...}
```

### Monitor Logs

**Railway:**
```bash
railway logs --tail
```

**Render:**
- Dashboard → Logs tab → Enable live tails

### Track Usage

Check stats endpoint:
```bash
curl https://your-backend.railway.app/api/v1/ingest/stats
```

Returns:
```json
{
  "total_chunks": 632,
  "total_queries": 123,
  "avg_query_results": 4.5,
  "collection_info": {...}
}
```

---

## Quick Commands Reference

```bash
# ========================================
# BACKEND DEPLOYMENT (Railway)
# ========================================

# Login
railway login

# Initialize
cd backend
railway init

# Set variables
railway variables set OPENAI_API_KEY=sk-xxx
railway variables set POSTGRES_URL=postgresql://...
railway variables set CORS_ORIGINS='["https://muskaanfayyaz.github.io"]'

# Deploy
railway up

# Get URL
railway domain

# View logs
railway logs

# Ingest chunks
curl -X POST $(railway domain)/api/v1/ingest/chunks

# ========================================
# FRONTEND DEPLOYMENT
# ========================================

# Build
cd book
npm run build

# Deploy to GitHub Pages
GIT_USER=muskaanfayyaz npm run deploy

# Or with GitHub Actions (automatic)
git push origin main

# ========================================
# TESTING
# ========================================

# Backend health
curl https://your-backend.railway.app/health

# Test ask endpoint
curl -X POST https://your-backend.railway.app/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Check stats
curl https://your-backend.railway.app/api/v1/ingest/stats
```

---

## Success Criteria

Your deployment is successful when:

✅ Backend health check returns "healthy"
✅ 632 chunks ingested
✅ Ask endpoint returns answers with sources
✅ Frontend loads without errors
✅ Chatbot button appears
✅ Normal mode works (ask questions)
✅ Selected text mode works (highlight text)
✅ Sources link correctly
✅ Mobile responsive
✅ Dark mode works
✅ No CORS errors in console

---

## Next Steps After Deployment

1. **Share with users**
   - Post on social media
   - Share GitHub repo
   - Submit to hackathon

2. **Monitor usage**
   - Check Railway/Render logs
   - Track OpenAI costs
   - Gather feedback

3. **Optimize**
   - Add caching
   - Implement rate limiting
   - Optimize prompts

4. **Scale** (if needed)
   - Upgrade Railway plan
   - Add Redis caching
   - Implement CDN

---

**Deployment Status:** ✅ Ready
**Estimated Time:** 30-45 minutes
**Difficulty:** Medium
**Support:** Check logs, test endpoints, verify environment variables

🎉 **Congratulations!** Your Physical AI RAG system is now live!
