# Neon Postgres Setup Guide

Complete guide to creating and configuring a FREE Neon Postgres database for the Physical AI Textbook backend.

---

## 🎯 What is Neon?

**Neon** is a serverless Postgres database with a generous FREE tier:

```
✅ FREE Tier Includes:
- 10 GB storage
- Unlimited queries
- Auto-scaling compute
- Branching support
- Point-in-time restore
- Connection pooling
```

**Perfect for:**
- Hackathon projects
- Portfolio apps
- Low-traffic production apps
- Development & testing

---

## 📋 Prerequisites

Before starting:
- ✅ Email account (Gmail works)
- ✅ GitHub account (for OAuth login)
- ✅ 5 minutes of time

---

## 🚀 Step-by-Step Setup

### Step 1: Create Neon Account (2 minutes)

**1.1 Visit Neon**

Go to: **https://neon.tech**

**1.2 Sign Up**

Click **"Sign Up"** button (top-right corner)

Choose one of:
- **Sign up with GitHub** (recommended - faster)
- **Sign up with Google**
- **Sign up with Email**

**1.3 Verify Email**

If using email signup:
- Check your inbox
- Click verification link
- You'll be redirected to Neon Console

---

### Step 2: Create Your First Project (3 minutes)

**2.1 Create Project**

After login, you'll see the Neon Console dashboard.

Click **"Create a project"** button

**2.2 Configure Project**

Fill in the following:

```
Project Name: physical-ai-backend
(or any name you prefer)

Region: Choose closest to your users
  - US East (Ohio) - aws-us-east-2
  - US West (Oregon) - aws-us-west-2
  - Europe (Frankfurt) - aws-eu-central-1
  - Asia Pacific (Singapore) - aws-ap-southeast-1

Postgres Version: 16 (latest - recommended)
```

**Recommended region selection:**
- **Render deployed in Oregon** → Choose **US West (Oregon)**
- **Render deployed in Ohio** → Choose **US East (Ohio)**
- **European users** → Choose **Europe (Frankfurt)**

**2.3 Click "Create Project"**

Wait 10-15 seconds for provisioning.

---

### Step 3: Get Connection String (1 minute)

**3.1 Find Connection Details**

Once project is created, you'll see the **Connection Details** panel.

**3.2 Select Connection Type**

In the "Connection Details" section:

1. **Connection type:** Select **"Pooled connection"** (recommended for serverless)
2. **Database:** `neondb` (default)
3. **Role:** Your username (auto-generated, like `physical-ai-backend-owner`)

**3.3 Copy Connection String**

You'll see a connection string like:

```
postgresql://username:password@ep-cool-meadow-12345678.us-west-2.aws.neon.tech/neondb?sslmode=require
```

**Format breakdown:**
```
postgresql://          ← Protocol
username:password      ← Credentials (auto-generated)
@ep-cool-meadow-12345678.us-west-2.aws.neon.tech  ← Host
/neondb                ← Database name
?sslmode=require       ← SSL mode (required)
```

**3.4 Save Connection String**

Click **"Copy"** button and save it somewhere secure.

⚠️ **IMPORTANT:** This contains your password. Keep it secret!

---

## 🔧 Integration with Backend

### Option 1: Using .env File (Local Development)

**Create `.env` file in `backend/` directory:**

```bash
cd backend
cat > .env << 'EOF'
# Neon Postgres Database
POSTGRES_URL=postgresql://username:password@ep-cool-meadow-12345678.us-west-2.aws.neon.tech/neondb?sslmode=require

# Other environment variables
OPENAI_API_KEY=sk-proj-your-key-here
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
EOF
```

**Replace:**
- `username:password@ep-cool-meadow-12345678.us-west-2.aws.neon.tech` with YOUR actual connection string
- `sk-proj-your-key-here` with YOUR OpenAI or Gemini API key

---

### Option 2: Render Deployment

**In Render Dashboard:**

1. Go to your service: https://dashboard.render.com/
2. Click your service (e.g., `physical-ai-backend`)
3. Click **"Environment"** tab
4. Click **"Add Environment Variable"**
5. Add:

```
Key: POSTGRES_URL
Value: postgresql://username:password@ep-cool-meadow-12345678.us-west-2.aws.neon.tech/neondb?sslmode=require
```

6. Click **"Save Changes"**
7. Service will automatically redeploy

---

## ✅ Verify Connection

### Test 1: Local Connection

**Install psycopg2 (if not already installed):**

```bash
cd backend
pip install psycopg2-binary
```

**Test connection script:**

```python
# test_neon.py
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from app.config import get_settings

async def test_connection():
    settings = get_settings()

    # Convert postgresql:// to postgresql+asyncpg://
    db_url = settings.postgres_url.replace(
        "postgresql://",
        "postgresql+asyncpg://"
    )

    engine = create_async_engine(db_url, echo=True)

    try:
        async with engine.begin() as conn:
            result = await conn.execute("SELECT version();")
            version = result.fetchone()
            print(f"✅ Connected to Neon Postgres!")
            print(f"   Version: {version[0]}")
            return True
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False
    finally:
        await engine.dispose()

if __name__ == "__main__":
    asyncio.run(test_connection())
```

**Run test:**

```bash
python test_neon.py
```

**Expected output:**

```
✅ Connected to Neon Postgres!
   Version: PostgreSQL 16.x on x86_64-pc-linux-gnu, compiled by gcc...
```

---

### Test 2: Create Tables

**Initialize database schema:**

```bash
cd backend

# Make sure your .env is configured
# Run the FastAPI app (it will auto-create tables on startup)
uvicorn app.main:app --reload
```

**Expected logs:**

```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Creating database tables...
INFO:     Database tables created successfully
INFO:     Application startup complete.
INFO:     Uvicorn running on http://127.0.0.1:8000
```

**Verify tables in Neon Console:**

1. Go to https://console.neon.tech/
2. Click your project
3. Click **"SQL Editor"** tab
4. Run query:

```sql
SELECT table_name
FROM information_schema.tables
WHERE table_schema = 'public';
```

**Expected result:**

```
table_name
-----------------------
chunk_metadata
search_queries
conversation_history
```

✅ **Success!** Your database is set up correctly.

---

## 📊 Monitor Your Database

### Check Storage Usage

**In Neon Console:**

1. Click your project
2. Look at **"Storage"** section in dashboard
3. Shows: `X MB / 10 GB used`

**FREE tier limit:** 10 GB (plenty for this project)

**Estimated usage for Physical AI Textbook:**
- 632 chunks × ~500 bytes = ~300 KB
- Conversation history (1000 queries) = ~500 KB
- Total: **< 1 MB** (0.01% of free tier)

### Check Active Connections

**Run in SQL Editor:**

```sql
SELECT count(*) as active_connections
FROM pg_stat_activity
WHERE state = 'active';
```

**Check compute hours:**

Neon FREE tier includes **191 hours/month** of active compute.

- **Serverless auto-suspend:** Inactive databases sleep after 5 minutes
- **Wake time:** < 1 second on first query
- **Your usage:** Likely < 10 hours/month for low traffic

---

## 🔒 Security Best Practices

### 1. Never Commit Connection String

**Add to `.gitignore`:**

```bash
echo ".env" >> .gitignore
echo "*.env" >> .gitignore
```

### 2. Use Environment Variables

**❌ Bad:**
```python
DATABASE_URL = "postgresql://user:pass@host/db"
```

**✅ Good:**
```python
from app.config import get_settings
settings = get_settings()
db_url = settings.postgres_url
```

### 3. Rotate Passwords Regularly

**In Neon Console:**

1. Go to your project
2. Click **"Settings"** tab
3. Click **"Reset password"**
4. Copy new connection string
5. Update environment variables

### 4. Use IP Allowlist (Optional)

**In Neon Console:**

1. Click **"Settings"** → **"IP Allow"**
2. Add IP ranges:
   - Your Render IP (check Render docs)
   - Your local IP (for development)

---

## 🐛 Troubleshooting

### Error: "connection refused"

**Cause:** Invalid connection string or wrong region

**Solution:**
1. Copy connection string again from Neon Console
2. Make sure you're using **Pooled connection** (not Direct)
3. Verify `sslmode=require` is in the URL

**Test:**
```bash
curl -I https://ep-cool-meadow-12345678.us-west-2.aws.neon.tech
```

Should return: `HTTP/1.1 404 Not Found` (this is OK - means host is reachable)

---

### Error: "too many connections"

**Cause:** Connection pooling not configured

**Solution:** Use **Pooled connection string** from Neon Console

**Pooled vs Direct:**
- **Pooled:** `ep-cool-meadow-12345678-pooler.us-west-2.aws.neon.tech` ← Use this!
- **Direct:** `ep-cool-meadow-12345678.us-west-2.aws.neon.tech`

---

### Error: "SSL connection required"

**Cause:** Missing `?sslmode=require` in connection string

**Solution:** Add SSL parameter:

```
postgresql://user:pass@host/db?sslmode=require
                                ↑ Must include this
```

---

### Error: "password authentication failed"

**Cause:** Password changed or incorrect

**Solution:**
1. Go to Neon Console
2. Click **"Connection Details"**
3. Click **"Reset password"**
4. Copy NEW connection string
5. Update `.env` or Render environment variables

---

### Database is slow

**Cause:** Neon serverless auto-suspends after inactivity

**Behavior:**
- First query after suspension: **~1 second** (wake-up time)
- Subsequent queries: **< 100ms**

**This is normal and EXPECTED for FREE tier!**

**Solutions:**
- Accept 1s wake-up time (fine for low-traffic apps)
- Upgrade to paid plan for always-on compute
- Use connection pooler (already recommended above)

---

## 💰 Free Tier Limits

```
Storage:               10 GB
Compute hours:         191 hours/month
Active time:           Unlimited queries during active time
Branches:              10
Projects:              1
Auto-suspend:          After 5 minutes inactivity
```

**What happens if you exceed limits?**

- **Storage > 10 GB:** Upgrade required ($0.000164/GB-hour ≈ $1.20/month for 15GB)
- **Compute > 191 hours:** Upgrade required ($0.16/hour ≈ $10/month for always-on)

**For Physical AI Textbook:**
- ✅ Storage: < 1 MB (0.01% of limit)
- ✅ Compute: < 10 hours/month (5% of limit)
- ✅ You're safe on FREE tier!

---

## 📋 Quick Reference

### Connection String Format

```
postgresql://USERNAME:PASSWORD@HOST:PORT/DATABASE?sslmode=require
```

**Example:**
```
postgresql://alex:AbCdEf123456@ep-cool-meadow-12345678-pooler.us-west-2.aws.neon.tech/neondb?sslmode=require
```

### Environment Variable

```bash
export POSTGRES_URL="postgresql://user:pass@host/db?sslmode=require"
```

### Backend Config

```python
# app/config.py
class Settings(BaseSettings):
    postgres_url: str = Field(..., env="POSTGRES_URL")
```

### Test Commands

```bash
# Test connection
python test_neon.py

# Start backend (creates tables automatically)
uvicorn app.main:app --reload

# Check tables via psql
psql "postgresql://user:pass@host/db?sslmode=require" -c "\dt"
```

---

## 🎯 Integration Checklist

**Before deploying:**
- [ ] Neon account created
- [ ] Project created in correct region
- [ ] Connection string copied (Pooled connection)
- [ ] Connection string saved securely
- [ ] Added to `.env` file OR Render environment variables
- [ ] `.env` added to `.gitignore`
- [ ] Tested connection locally
- [ ] Tables created successfully (3 tables: chunk_metadata, search_queries, conversation_history)
- [ ] Backend can read/write to database

**After deploying:**
- [ ] Backend /health endpoint returns healthy
- [ ] POST /api/v1/ingest/chunks succeeds (creates 632 chunk_metadata rows)
- [ ] GET /api/v1/ingest/stats shows chunk count
- [ ] POST /api/v1/ask/ returns answers (creates conversation_history rows)
- [ ] Monitor storage usage in Neon Console

---

## 🚀 Next Steps

**1. Ingest Chunks**

```bash
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

**2. Verify in Neon SQL Editor**

```sql
-- Check chunk count
SELECT COUNT(*) FROM chunk_metadata;
-- Expected: 632

-- View sample chunks
SELECT chunk_id, heading_hierarchy, LEFT(content, 100)
FROM chunk_metadata
LIMIT 5;
```

**3. Test Q&A**

```bash
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

**4. Check conversation history**

```sql
SELECT session_id, role, LEFT(message, 50), created_at
FROM conversation_history
ORDER BY created_at DESC
LIMIT 10;
```

---

## 📞 Support

**Neon Documentation:**
- Quickstart: https://neon.tech/docs/get-started-with-neon/signing-up
- Connection strings: https://neon.tech/docs/connect/connect-from-any-app
- Pooling: https://neon.tech/docs/connect/connection-pooling

**Common Issues:**
- https://neon.tech/docs/connect/connectivity-issues

**Community:**
- Discord: https://discord.gg/neon
- GitHub: https://github.com/neondatabase/neon

---

## 📊 Cost Comparison

| Provider | Storage | Compute | Price |
|----------|---------|---------|-------|
| **Neon (FREE)** | 10 GB | 191 hrs/month | **$0** |
| Heroku Postgres | 1 GB | Shared | $0 |
| Heroku Postgres | 10 GB | Standard | $50/month |
| AWS RDS (smallest) | 20 GB | t3.micro | ~$15/month |
| Supabase FREE | 500 MB | Paused after 1 week | $0 |
| **Neon Pro** | 10 GB | Always-on | $19/month |

**Winner:** Neon FREE tier (10 GB, auto-suspend, perfect for low-traffic apps)

---

## ✅ Summary

**What you did:**
1. ✅ Created FREE Neon Postgres account
2. ✅ Created project in optimal region
3. ✅ Got pooled connection string
4. ✅ Configured backend environment variables
5. ✅ Tested connection
6. ✅ Created database tables (chunk_metadata, search_queries, conversation_history)

**Total cost:** **$0/month** (FREE tier)

**Your database:**
- 🌍 URL: `ep-your-id.us-west-2.aws.neon.tech`
- 💾 Storage: 10 GB available
- ⚡ Compute: 191 hours/month available
- 🔒 SSL: Enabled
- 📊 Tables: 3 (auto-created by FastAPI)

**Next:** Ingest 632 chunks and start using the chatbot!

---

## 🎉 You're Ready!

Your Neon Postgres database is now:
- ✅ Created and configured
- ✅ Connected to backend
- ✅ Ready to store chunks, queries, and conversations
- ✅ 100% FREE (within generous limits)
- ✅ Auto-scaling and serverless

**Total setup time:** 5 minutes
**Total cost:** $0/month

🚀 **Deploy your chatbot and start answering questions!**
