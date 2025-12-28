# Render Free Tier - Quick Start

Ultra-fast deployment guide for Render's FREE tier (25 minutes total).

---

## 🎯 What You'll Deploy (100% FREE*)

```
Physical AI Textbook (GitHub Pages) ─────► FREE ✅
         │
         ├─► FastAPI Backend (Render) ────► FREE ✅ (750 hrs/month)
         │
         ├─► Neon Postgres ───────────────► FREE ✅ (10GB)
         │
         ├─► Qdrant Cloud ────────────────► FREE ✅ (1GB)
         │
         └─► OpenAI API ──────────────────► ~$1-2/month 💵
```

\* Only OpenAI costs ~$1-2/month

---

## ⚡ Quick Deploy (4 Steps)

### Step 1: Get Credentials (5 min)

**You need:**

1. **OpenAI API Key**
   - Go to: https://platform.openai.com/api-keys
   - Click "Create new secret key"
   - Copy: `sk-proj-xxx...`

2. **Neon Postgres**
   - Go to: https://neon.tech
   - Sign up → Create project
   - Copy connection string:
     ```
     postgresql://user:pass@host.neon.tech/db?sslmode=require
     ```

3. **Qdrant** (Already provided!)
   ```
   URL: https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
   API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
   ```

---

### Step 2: Deploy Backend to Render (10 min)

**2.1 Create Service**

1. Go to: https://render.com/
2. Sign up with GitHub
3. Click **"New +"** → **"Web Service"**
4. Connect repository: `Physical-AI-Humanoid-Robotics`

**2.2 Configure**

```
Name: physical-ai-backend
Region: Oregon (US West)
Branch: main
Root Directory: backend
Runtime: Python 3

Build Command: pip install -r requirements.txt
Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT

Instance Type: Free ⚠️ (Important!)
```

**2.3 Add Environment Variables**

Click "Add Environment Variable" for each:

```env
OPENAI_API_KEY=sk-proj-your-key-here
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]
```

**2.4 Deploy**

- Click **"Create Web Service"**
- Wait 2-3 minutes
- Copy URL: `https://physical-ai-backend.onrender.com`

**2.5 Ingest Chunks**

```bash
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

Wait ~60 seconds. Should return:
```json
{"success": true, "chunks_processed": 632, ...}
```

---

### Step 3: Configure Frontend (5 min)

**3.1 Update Config**

Edit `book/docusaurus.config.js`:

```javascript
customFields: {
  chatbotApiUrl: 'https://physical-ai-backend.onrender.com/api/v1',
}
```

**3.2 Add Chatbot to Chapters**

Add to bottom of each chapter file:

```mdx
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

---

### Step 4: Deploy Frontend (5 min)

```bash
# Commit changes
git add .
git commit -m "Add chatbot with Render backend"
git push origin main

# GitHub Actions will auto-deploy!
# Or manual:
cd book
GIT_USER=muskaanfayyaz npm run deploy
```

**Visit:** https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/

✅ **Done!**

---

## 🧪 Test Your Deployment

```bash
# Backend health
curl https://physical-ai-backend.onrender.com/health

# Backend stats
curl https://physical-ai-backend.onrender.com/api/v1/ingest/stats

# Test chatbot
# Visit: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/
# Click 💬 button
# Ask: "What is ROS 2?"
# Wait 30s for first request (cold start - normal!)
```

---

## ⚠️ Important Notes

### Render Free Tier Behavior

**Service sleeps after 15 minutes of inactivity**
- First request: **30-60 seconds** (cold start)
- Subsequent requests: **1-2 seconds** (normal)
- This is EXPECTED and NORMAL for free tier!

**What to tell users:**
> "First chatbot request may take 30 seconds as the server wakes up. This is normal for free hosting!"

### Monthly Limits

- ✅ 750 hours/month (plenty!)
- ✅ Unlimited bandwidth
- ✅ Unlimited requests

**You're fine for:**
- Hackathon demos
- Portfolio projects
- Low-traffic sites

---

## 🔧 Troubleshooting

### "Backend connection failed"

**Check if service is running:**
```bash
curl https://physical-ai-backend.onrender.com/health
```

**If timeout or error:**
1. Go to Render Dashboard
2. Check service status
3. Check logs for errors
4. Verify environment variables

### CORS Error

Update `CORS_ORIGINS` in Render:
```
["https://muskaanfayyaz.github.io","http://localhost:3000"]
```

### Slow First Request

**This is normal!** Render free tier sleeps after 15min.
- First request: ~30-60s (waking up)
- After that: ~1-2s (normal speed)

### No Chunks Found

Re-ingest:
```bash
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

---

## 💰 Cost Breakdown

| Service | Cost | Why |
|---------|------|-----|
| Render | **FREE** | 750 hrs/month |
| Neon Postgres | **FREE** | 10GB included |
| Qdrant Cloud | **FREE** | 1GB included |
| GitHub Pages | **FREE** | Unlimited static hosting |
| OpenAI API | **$1-2/month** | Pay per use |

**Total: $1-2/month** (OpenAI only)

### OpenAI Costs

- Initial ingestion: $0.50 (one-time)
- Per question: $0.001
- 100 questions: $0.10
- 1000 questions: $1.00

**For hackathon:** Expect $1-2 total

---

## 📋 Deployment Checklist

**Before starting:**
- [ ] GitHub repo accessible
- [ ] OpenAI API key ready
- [ ] Neon database created
- [ ] Connection strings copied

**Backend (Render):**
- [ ] Service created on Render
- [ ] **Free tier selected** ⚠️
- [ ] All env variables set
- [ ] Service deployed successfully
- [ ] Health check returns "healthy"
- [ ] Chunks ingested (632)

**Frontend (GitHub Pages):**
- [ ] Config updated with Render URL
- [ ] Chatbot added to chapters
- [ ] Changes committed and pushed
- [ ] GitHub Actions completed
- [ ] Site accessible

**Verification:**
- [ ] Chatbot button appears
- [ ] Can ask questions
- [ ] Answers appear with sources
- [ ] Works on mobile
- [ ] Dark mode works

---

## 🎯 Success Criteria

Your deployment works if:

✅ Render shows "Live" status
✅ Health check returns `{"status": "healthy"}`
✅ Chatbot opens when clicked
✅ Questions get answered (first one takes 30-60s)
✅ Sources appear and link correctly
✅ Works on mobile
✅ No CORS errors in console

---

## 🚀 Post-Deployment

**Share your project:**
```
Demo: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/
GitHub: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics
Backend: https://physical-ai-backend.onrender.com/docs
```

**Monitor usage:**
- Render: Check dashboard for uptime
- OpenAI: Check usage at platform.openai.com/usage
- Set spending alerts

**Get feedback:**
- Ask users to try the chatbot
- Note which questions work well
- Improve prompts if needed

---

## 📞 Need Help?

**Check:**
1. Render logs (Dashboard → Service → Logs)
2. Browser console (F12)
3. GitHub Actions (if frontend deploy fails)

**Common issues:**
- Slow first request → Normal for free tier
- CORS error → Update CORS_ORIGINS
- 404 error → Check API URL in config
- No chunks → Run ingest command

---

**Deployment Time:** 25 minutes
**Cost:** FREE (except OpenAI ~$1-2)
**Best For:** Hackathons, demos, portfolios

🎉 **You're live on Render FREE tier!**

---

## All-in-One Commands

```bash
# Test backend
curl https://physical-ai-backend.onrender.com/health
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?"}'

# Deploy frontend
cd book
npm run build
git add . && git commit -m "Deploy" && git push origin main
```

**Your URLs:**
- Frontend: https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/
- Backend: https://physical-ai-backend.onrender.com
- API Docs: https://physical-ai-backend.onrender.com/docs
