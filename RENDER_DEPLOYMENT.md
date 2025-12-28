# Render Free Tier Deployment Guide

Complete guide to deploy the Physical AI RAG system on Render's FREE tier.

---

## ✅ Free Tier Resources

**What You Get FREE on Render:**
- 750 hours/month of web service runtime
- Services sleep after 15 minutes of inactivity
- Services wake up automatically on request (cold start ~30s)
- Free SSL certificates
- Unlimited bandwidth
- Auto-deploy from GitHub

**Perfect for:** Demos, portfolios, hackathons, small projects

---

## Prerequisites

- [ ] GitHub account with your repository
- [ ] OpenAI API key ([Get here](https://platform.openai.com/api-keys))
- [ ] Neon Postgres database (FREE tier - [Create here](https://neon.tech))
- [ ] Qdrant Cloud account (already provided)

---

## Part 1: Deploy Backend to Render (10 minutes)

### Step 1.1: Create Render Account

1. Visit: https://render.com/
2. Click **"Get Started for Free"**
3. Sign up with GitHub (recommended)
4. Verify your email

### Step 1.2: Connect GitHub Repository

1. In Render Dashboard, click **"New +"** → **"Web Service"**
2. Click **"Connect account"** to link GitHub
3. Select repository: `Physical-AI-Humanoid-Robotics`
4. Click **"Connect"**

### Step 1.3: Configure Web Service

**Basic Settings:**
```
Name: physical-ai-backend
Region: Oregon (US West) or Frankfurt (EU)
Branch: main
Root Directory: backend
Runtime: Python 3
```

**Build Settings:**
```
Build Command: pip install -r requirements.txt
Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Instance Type:**
```
Instance Type: Free
```

⚠️ **IMPORTANT:** Make sure "Free" is selected!

### Step 1.4: Add Environment Variables

Scroll to **"Environment"** section and add these variables:

Click **"Add Environment Variable"** for each:

**Required Variables:**

1. **OPENAI_API_KEY**
   ```
   Value: sk-proj-your-actual-openai-key-here
   ```

2. **QDRANT_URL**
   ```
   Value: https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
   ```

3. **QDRANT_API_KEY**
   ```
   Value: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
   ```

4. **POSTGRES_URL**
   ```
   Value: postgresql://username:password@ep-xxx.neon.tech/dbname?sslmode=require
   ```
   *(Get this from Neon - see Step 2)*

5. **CORS_ORIGINS**
   ```
   Value: ["https://muskaanfayyaz.github.io","http://localhost:3000"]
   ```

**Optional (Recommended):**

6. **OPENAI_EMBEDDING_MODEL**
   ```
   Value: text-embedding-3-small
   ```

7. **EMBEDDING_DIMENSIONS**
   ```
   Value: 1536
   ```

8. **DEBUG**
   ```
   Value: False
   ```

### Step 1.5: Deploy

1. Click **"Create Web Service"** at the bottom
2. Render will start building and deploying
3. Wait 2-3 minutes for first deployment

**You'll see:**
```
==> Installing dependencies...
==> Building application...
==> Starting service...
==> Your service is live 🎉
```

### Step 1.6: Get Your Backend URL

After deployment succeeds:

1. Your URL will be: `https://physical-ai-backend.onrender.com`
2. **Copy this URL** - you'll need it for frontend!

### Step 1.7: Test Backend Deployment

```bash
# Set your Render URL
BACKEND_URL=https://physical-ai-backend.onrender.com

# Test health endpoint
curl $BACKEND_URL/health
```

**Expected Response:**
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

⚠️ **First request may take 30-60 seconds** (cold start from sleep)

### Step 1.8: Ingest Chunks (CRITICAL!)

```bash
# Ingest all 632 chunks into the system
curl -X POST $BACKEND_URL/api/v1/ingest/chunks

# This takes ~60 seconds
```

**Expected Response:**
```json
{
  "success": true,
  "chunks_processed": 632,
  "chunks_stored_postgres": 632,
  "chunks_stored_qdrant": 632,
  "processing_time_seconds": 52.3
}
```

### Step 1.9: Verify Ingestion

```bash
# Check stats
curl $BACKEND_URL/api/v1/ingest/stats
```

**Expected:**
```json
{
  "total_chunks": 632,
  "total_queries": 0,
  "total_conversations": 0,
  "avg_query_results": 0.0,
  "collection_info": {...}
}
```

### Step 1.10: Test Ask Endpoint

```bash
# Test RAG Q&A
curl -X POST $BACKEND_URL/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "top_k": 5
  }'
```

**Should return:** AI-generated answer with sources!

✅ **Backend deployment complete!**

---

## Part 2: Set Up Neon Postgres (5 minutes)

### Step 2.1: Create Neon Account

1. Visit: https://neon.tech/
2. Click **"Sign up"** (use GitHub for easy login)
3. Verify email

### Step 2.2: Create Database

1. Click **"Create a project"**
2. Configure:
   ```
   Project name: physical-ai-db
   Region: US East (Ohio) or Europe (Frankfurt)
   Postgres version: 16
   ```
3. Click **"Create project"**

### Step 2.3: Get Connection String

1. In project dashboard, find **"Connection string"**
2. Select **"Pooled connection"**
3. Copy the connection string:
   ```
   postgresql://username:password@ep-xxx-123.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### Step 2.4: Update Render Environment Variable

1. Go back to Render Dashboard
2. Click your service: `physical-ai-backend`
3. Go to **"Environment"** tab
4. Find `POSTGRES_URL` variable
5. Click **"Edit"**
6. Paste your Neon connection string
7. Click **"Save Changes"**

Render will automatically redeploy with new settings.

### Step 2.5: Re-ingest Chunks (After Postgres Update)

```bash
# Wait for redeploy to finish (~2 min)
# Then ingest again
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

---

## Part 3: Configure Frontend (5 minutes)

### Step 3.1: Update Docusaurus Configuration

**Edit:** `book/docusaurus.config.js`

Find the `customFields` section and update:

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

  // ADD THIS SECTION:
  customFields: {
    // Your Render backend URL
    chatbotApiUrl: process.env.NODE_ENV === 'production'
      ? 'https://physical-ai-backend.onrender.com/api/v1'  // ← UPDATE THIS
      : 'http://localhost:8000/api/v1',
  },

  // ... rest of config
};

module.exports = config;
```

**IMPORTANT:** Replace `https://physical-ai-backend.onrender.com/api/v1` with YOUR actual Render URL from Step 1.6.

### Step 3.2: Add Chatbot to Chapters

Edit any chapter file, for example: `book/docs/intro.md`

**Add at the bottom:**

```mdx
---

import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

**Example for all chapters:**

1. `docs/intro.md`
2. `docs/chapters/chapter-01-introduction-to-physical-ai.mdx`
3. `docs/chapters/chapter-02-sensor-systems.mdx`
4. ... etc

Add the same import and component at the end of each file.

### Step 3.3: Test Locally First

```bash
cd book

# Install dependencies (if not done)
npm install

# Start dev server
npm start
```

Visit: http://localhost:3000

**Test:**
1. Click chatbot button (💬)
2. Ask: "What is Physical AI?"
3. Wait ~30s for first request (Render cold start)
4. Verify answer appears with sources

⚠️ **Note:** First request to Render backend is slow (cold start). Subsequent requests are fast.

### Step 3.4: Build for Production

```bash
# Clean previous builds
npm run clear

# Build
npm run build

# Test production build locally
npm run serve
```

Visit: http://localhost:3000

**Verify:**
- No errors in console
- Chatbot works
- Sources link correctly

---

## Part 4: Deploy Frontend to GitHub Pages (5 minutes)

### Step 4.1: Update Backend CORS (CRITICAL!)

Go to Render Dashboard:

1. Click your service: `physical-ai-backend`
2. Go to **"Environment"** tab
3. Find `CORS_ORIGINS`
4. Edit to include your GitHub Pages URL:
   ```
   ["https://muskaanfayyaz.github.io","http://localhost:3000"]
   ```
5. Click **"Save Changes"**

Render will redeploy automatically.

### Step 4.2: Commit Changes

```bash
# Add all changes
git add .

# Commit
git commit -m "Configure chatbot with Render backend

- Add Render API URL to config
- Add ChatbotWrapper to chapters
- Configure CORS for GitHub Pages"

# Push to main
git push origin main
```

### Step 4.3: Deploy to GitHub Pages

GitHub Actions will automatically deploy!

**Check deployment:**
1. Go to: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/actions
2. Wait for workflow to complete (~3-5 minutes)
3. Look for ✅ green checkmark

**Or manual deploy:**
```bash
cd book

# Deploy to gh-pages branch
GIT_USER=muskaanfayyaz npm run deploy
```

### Step 4.4: Enable GitHub Pages (If Not Already Enabled)

1. Go to: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/settings/pages
2. Under **"Source"**:
   - Branch: `gh-pages`
   - Folder: `/` (root)
3. Click **"Save"**
4. Wait 2-3 minutes for deployment

### Step 4.5: Verify Deployment

Visit: **https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/**

**Test:**
1. Page loads correctly
2. Chatbot button appears (💬)
3. Click chatbot
4. Ask: "What is ROS 2?"
5. Wait ~30s for first request (Render cold start)
6. Answer appears with sources

✅ **Full deployment complete!**

---

## Part 5: Understanding Render Free Tier

### How Free Tier Works

**Service Behavior:**
- ✅ Service runs for 750 hours/month (plenty for demo)
- ⏸️ **Sleeps after 15 minutes** of inactivity
- 🔄 **Wakes automatically** on incoming request
- ⏱️ Cold start time: 20-60 seconds
- ⚡ After wake: Normal speed (1-2s response)

**What This Means:**
- First chatbot request after inactivity: **Slow (~30-60s)**
- Subsequent requests: **Fast (~1-2s)**
- If no activity for 15 min: Service sleeps again

**Perfect For:**
- Hackathon demos
- Portfolio projects
- Low-traffic applications
- Personal projects

**Not Ideal For:**
- Production apps with 24/7 users
- High-traffic services
- Real-time requirements

### How to Handle Cold Starts

**Option 1: Accept It (Recommended for Free)**
- First request is slow, that's okay
- Add loading message in chatbot
- Subsequent requests are fast

**Option 2: Keep-Alive Ping (Uses More Hours)**
```bash
# Run this script to ping every 14 minutes
while true; do
  curl https://physical-ai-backend.onrender.com/health
  sleep 840  # 14 minutes
done
```

⚠️ This uses your 750 hour quota faster.

**Option 3: Upgrade to Paid ($7/month)**
- Service never sleeps
- Always fast responses
- More resources

---

## Part 6: Verification & Testing

### Backend Tests

```bash
BACKEND_URL=https://physical-ai-backend.onrender.com

# 1. Health check
curl $BACKEND_URL/health

# 2. Check stats
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

### Frontend Tests

Visit: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/

**Manual Checklist:**

- [ ] Page loads without errors
- [ ] Chatbot button (💬) visible
- [ ] Clicking button opens chat
- [ ] Can type and send message
- [ ] First request takes 30-60s (cold start - normal!)
- [ ] Answer appears with sources
- [ ] Sources are clickable
- [ ] Text selection mode works
- [ ] Mobile responsive (test on phone)
- [ ] Dark mode works

### Browser Console Check

Press **F12** to open DevTools:

**Look for:**
- ✅ No errors in Console tab
- ✅ API requests succeed (Network tab)
- ✅ Response status: 200 OK

**If you see errors:**
- ❌ CORS error → Update CORS_ORIGINS in Render
- ❌ 404 error → Check API URL in config
- ❌ 500 error → Check Render logs

---

## Part 7: Troubleshooting

### Issue 1: Chatbot Shows "Backend connection failed"

**Symptoms:**
- Error banner in chatbot
- No responses

**Solutions:**

1. **Check if backend is running:**
   ```bash
   curl https://physical-ai-backend.onrender.com/health
   ```

2. **Check Render logs:**
   - Go to Render Dashboard
   - Click your service
   - Click **"Logs"** tab
   - Look for errors

3. **Verify environment variables:**
   - Go to **"Environment"** tab
   - Check all variables are set correctly
   - Click **"Save Changes"** to redeploy

4. **Check API URL in config:**
   ```bash
   cat book/docusaurus.config.js | grep chatbotApiUrl
   ```
   Should show your Render URL.

### Issue 2: CORS Error in Browser Console

**Symptoms:**
```
Access to fetch at 'https://physical-ai-backend.onrender.com/api/v1/ask/'
from origin 'https://muskaanfayyaz.github.io' has been blocked by CORS policy
```

**Solution:**

1. Go to Render Dashboard
2. Click your service
3. Go to **"Environment"** tab
4. Edit `CORS_ORIGINS`:
   ```
   ["https://muskaanfayyaz.github.io","http://localhost:3000"]
   ```
5. Save (service will redeploy)

### Issue 3: Service Sleeping / Slow First Request

**Symptoms:**
- First chatbot request takes 30-60 seconds
- Subsequent requests are fast

**This is NORMAL for Render Free Tier!**

**Solutions:**

1. **Accept it** - Free tier limitation
2. **Add loading message** in chatbot UI (already implemented)
3. **Upgrade to paid** ($7/month for always-on)

### Issue 4: Chunks Not Found / Empty Results

**Symptoms:**
- Chatbot says "cannot find information"
- Search returns no results

**Solution:**

Re-ingest chunks:
```bash
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

Wait ~60 seconds, then test again.

### Issue 5: Render Service Won't Start

**Check Render logs:**

Common issues:
- Missing environment variables
- Incorrect Python version
- Dependency installation failed

**Fix:**

1. Go to **"Environment"** tab
2. Verify all required variables exist
3. Go to **"Logs"** tab
4. Read error messages
5. Fix and click **"Manual Deploy"**

### Issue 6: OpenAI API Error

**Symptoms:**
```
"OpenAI API error: 401 Unauthorized"
```

**Solution:**

1. Check API key is valid
2. Check billing is enabled on OpenAI
3. Update `OPENAI_API_KEY` in Render Environment

---

## Part 8: Monitoring Your Deployment

### Check Service Status

**Render Dashboard:**
1. Go to https://dashboard.render.com/
2. Click your service
3. Check **Status**: Should show "Live"

**Quick Health Check:**
```bash
curl https://physical-ai-backend.onrender.com/health
```

### View Logs

**Real-time logs:**
1. Render Dashboard → Your service
2. Click **"Logs"** tab
3. Enable **"Live tail"**

**Check for errors:**
- Look for `ERROR` lines
- Look for `Exception` messages
- Look for HTTP error codes (500, 404)

### Monitor Usage

**Free Tier Limits:**
- 750 hours/month runtime
- Unlimited bandwidth
- Unlimited requests

**Check usage:**
1. Render Dashboard
2. Click your service
3. See "Usage" section

### Track API Costs (OpenAI)

**Monitor costs:**
1. Go to: https://platform.openai.com/usage
2. Check daily usage
3. Set spending limits

**Estimated costs:**
- Initial ingestion: ~$0.50 (one-time)
- Per query: ~$0.001
- 100 queries: ~$0.10
- 1000 queries: ~$1.00

---

## Part 9: Deployment Checklist

### Pre-Deployment

- [ ] Render account created
- [ ] GitHub repository accessible
- [ ] OpenAI API key obtained
- [ ] Neon Postgres database created
- [ ] Qdrant credentials verified

### Backend Deployment (Render)

- [ ] Web service created on Render
- [ ] Repository connected
- [ ] Build/start commands configured
- [ ] **Free tier instance selected**
- [ ] All environment variables set:
  - [ ] OPENAI_API_KEY
  - [ ] QDRANT_URL
  - [ ] QDRANT_API_KEY
  - [ ] POSTGRES_URL
  - [ ] CORS_ORIGINS
- [ ] Service deployed successfully
- [ ] Health check returns "healthy"
- [ ] Chunks ingested (632 chunks)
- [ ] Ask endpoint tested
- [ ] Backend URL copied

### Frontend Configuration

- [ ] `docusaurus.config.js` updated with Render URL
- [ ] Chatbot added to chapters
- [ ] Production build tested locally
- [ ] No errors in browser console

### Frontend Deployment (GitHub Pages)

- [ ] Changes committed to Git
- [ ] Pushed to GitHub
- [ ] GitHub Actions workflow completed
- [ ] GitHub Pages enabled
- [ ] Site accessible at GitHub Pages URL
- [ ] Chatbot works on live site

### Post-Deployment Verification

- [ ] All chatbot modes tested
- [ ] Sources link correctly
- [ ] Mobile responsive works
- [ ] Dark mode works
- [ ] Cold start time acceptable (<60s)
- [ ] Subsequent requests fast (<2s)

---

## Part 10: Cost Summary

### Completely FREE Services

| Service | Free Tier | Usage | Cost |
|---------|-----------|-------|------|
| **Render** | 750 hrs/month | Backend hosting | **FREE** ✅ |
| **Neon Postgres** | 10GB storage | Metadata storage | **FREE** ✅ |
| **Qdrant Cloud** | 1GB storage | Vector storage | **FREE** ✅ |
| **GitHub Pages** | Unlimited | Frontend hosting | **FREE** ✅ |

### Paid Service (Required)

| Service | Cost | Usage | Monthly |
|---------|------|-------|---------|
| **OpenAI** | Pay-per-use | Embeddings + LLM | **~$1-2** 💵 |

**Total Monthly Cost: $1-2** (OpenAI only)

### OpenAI Cost Breakdown

**One-time:**
- Initial ingestion (632 chunks): ~$0.50

**Ongoing:**
- Per query (ask endpoint): ~$0.001
- 100 queries/month: ~$0.10
- 500 queries/month: ~$0.50
- 1000 queries/month: ~$1.00

**For hackathon demo:** Expect $1-2 total

---

## Part 11: Optimization Tips

### Reduce Cold Start Time

**In `backend/app/main.py`**, add health endpoint optimization:

```python
@app.get("/")
async def root():
    """Root endpoint - fast response for keep-alive pings."""
    return {"status": "ok"}
```

This endpoint responds faster than `/health` for keep-alive pings.

### Add Loading Message

Already implemented in chatbot! First request shows:
```
"⏳ Connecting to backend... (may take 30s on first request)"
```

### Optimize for Mobile

Chatbot is already responsive:
- Full-screen on mobile
- Touch-friendly buttons
- Smooth scrolling

---

## Quick Commands Reference

```bash
# ========================================
# BACKEND DEPLOYMENT (Render)
# ========================================

# Test health
curl https://physical-ai-backend.onrender.com/health

# Ingest chunks
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks

# Test ask endpoint
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Check stats
curl https://physical-ai-backend.onrender.com/api/v1/ingest/stats

# ========================================
# FRONTEND DEPLOYMENT
# ========================================

# Build
cd book
npm run build

# Deploy to GitHub Pages
git add .
git commit -m "Deploy with chatbot"
git push origin main

# Or manual deploy
GIT_USER=muskaanfayyaz npm run deploy

# ========================================
# MONITORING
# ========================================

# Check Render logs
# Dashboard → Service → Logs tab

# Check GitHub Actions
# Repository → Actions tab

# Check OpenAI usage
# platform.openai.com/usage
```

---

## Success Criteria

Your deployment is successful when:

✅ Render service shows "Live" status
✅ Health check returns `{"status": "healthy"}`
✅ 632 chunks ingested successfully
✅ Ask endpoint returns answers with sources
✅ GitHub Pages site loads without errors
✅ Chatbot button appears on chapters
✅ Normal mode works (ask questions)
✅ Selected text mode works (highlight text)
✅ Sources link correctly
✅ Mobile responsive
✅ Dark mode works
✅ No CORS errors in console
✅ First request takes 30-60s (cold start - expected!)
✅ Subsequent requests fast (1-2s)

---

## Next Steps After Deployment

1. **Test thoroughly**
   - Ask various questions
   - Test on mobile
   - Share with friends for feedback

2. **Monitor costs**
   - Check OpenAI usage daily
   - Set spending alerts

3. **Improve performance** (optional)
   - Cache common queries
   - Optimize prompts
   - Add rate limiting

4. **Submit to hackathon**
   - Include live demo link
   - Mention FREE deployment
   - Highlight features

---

## Support & Resources

**Render Documentation:**
- https://render.com/docs

**Neon Documentation:**
- https://neon.tech/docs/introduction

**OpenAI API:**
- https://platform.openai.com/docs

**Qdrant Cloud:**
- https://qdrant.tech/documentation/

**Troubleshooting:**
- Check Render logs first
- Verify environment variables
- Test endpoints with curl
- Check browser console

---

**Deployment Status:** ✅ 100% FREE (except OpenAI $1-2/month)
**Total Time:** 25-30 minutes
**Difficulty:** Easy (step-by-step guide)
**Best For:** Hackathons, demos, portfolios

🎉 **Your Physical AI RAG system is deployed on Render FREE tier!**
