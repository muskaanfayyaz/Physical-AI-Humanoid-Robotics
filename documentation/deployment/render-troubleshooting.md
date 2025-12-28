# Render Deployment Troubleshooting Guide

Quick fixes for common Render deployment errors.

---

## ❌ Error: "metadata-generation-failed"

### Problem
```
error: metadata-generation-failed
× Encountered error while generating package metadata.
```

### Root Cause
This error typically occurs due to:
1. **`psycopg2-binary` build issues** - Requires compilation on Render
2. **Missing system dependencies**
3. **Python version mismatch**
4. **Corrupted package metadata**

### ✅ Solutions

#### Solution 1: Remove psycopg2-binary (Recommended)

Since the backend uses `asyncpg` for async database connections, `psycopg2-binary` is not needed.

**File:** `backend/requirements.txt`

```diff
# Database - PostgreSQL with async support
sqlalchemy==2.0.27
asyncpg==0.29.0
- psycopg2-binary==2.9.9
+ # psycopg2-binary removed - asyncpg handles connections
```

**Why this works:** Removes the problematic package that requires compilation.

#### Solution 2: Use psycopg2 instead of psycopg2-binary

If you absolutely need `psycopg2`, use the source version:

```diff
- psycopg2-binary==2.9.9
+ psycopg2==2.9.9
```

**Note:** This is slower to build but works on Render.

#### Solution 3: Specify Python Version

Create `backend/runtime.txt`:
```
python-3.11.9
```

**Why this works:** Ensures consistent Python version across environments.

#### Solution 4: Pin all dependencies

Run locally:
```bash
cd backend
pip freeze > requirements-frozen.txt
```

Then use `requirements-frozen.txt` for deployment.

---

## ❌ Error: "Application failed to respond"

### Problem
Build succeeds but service doesn't start.

### Solutions

1. **Check Start Command:**
   ```
   uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```

2. **Verify Environment Variables:**
   - `GEMINI_API_KEY` is set
   - `QDRANT_URL` is correct
   - `QDRANT_API_KEY` is set
   - `POSTGRES_URL` is valid

3. **Check Logs:**
   - Go to Render Dashboard → Your Service → Logs
   - Look for startup errors

---

## ❌ Error: "Module not found"

### Problem
```
ModuleNotFoundError: No module named 'app'
```

### Solution

**Verify Root Directory in Render:**
```
Root Directory: backend
```

**Verify Start Command:**
```
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

Not: `uvicorn main:app` ❌

---

## ❌ Error: "Database connection failed"

### Problem
Backend starts but can't connect to Neon Postgres.

### Solutions

1. **Check POSTGRES_URL format:**
   ```
   postgresql://user:password@ep-xxx.neon.tech/dbname?sslmode=require
   ```

2. **Verify Neon database is running:**
   - Log into Neon console
   - Check if database is active

3. **Check IP allowlist:**
   - Neon Free tier allows all IPs by default
   - If restricted, add Render's IPs

---

## ❌ Error: "Qdrant connection failed"

### Problem
Backend starts but can't connect to Qdrant.

### Solutions

1. **Verify QDRANT_URL:**
   ```
   https://your-cluster.gcp.cloud.qdrant.io
   ```

2. **Verify QDRANT_API_KEY:**
   - Should be a JWT token
   - Get from Qdrant Cloud console

3. **Check Qdrant cluster status:**
   - Log into Qdrant Cloud
   - Verify cluster is running

---

## ❌ Error: "Build timeout"

### Problem
Build takes too long and times out.

### Solutions

1. **Use build script:**

   **File:** `backend/build.sh`
   ```bash
   #!/usr/bin/env bash
   set -o errexit
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

2. **In Render, set Build Command:**
   ```
   ./build.sh
   ```

3. **Reduce dependencies:**
   - Remove unused packages from requirements.txt
   - Use lighter alternatives

---

## ❌ Error: "Port already in use"

### Problem
```
Address already in use
```

### Solution

**Always use `$PORT` environment variable:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Not hardcoded:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000  ❌
```

---

## 🔍 General Debugging Steps

### 1. Check Render Logs

```
Render Dashboard → Your Service → Logs
```

Look for:
- Build errors
- Startup errors
- Runtime errors

### 2. Test Locally First

```bash
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload
```

If it works locally, issue is deployment-specific.

### 3. Verify Environment Variables

In Render Dashboard:
- Go to Environment section
- Verify all required variables are set
- Check for typos in variable names

### 4. Check Health Endpoint

After deployment:
```bash
curl https://your-service.onrender.com/health
```

Should return:
```json
{
  "status": "healthy",
  "database_connected": true,
  "qdrant_connected": true,
  "openai_configured": true
}
```

---

## 📋 Pre-Deployment Checklist

Before deploying to Render:

- [ ] `backend/requirements.txt` has no `psycopg2-binary`
- [ ] `backend/runtime.txt` specifies Python 3.11.9
- [ ] `backend/build.sh` exists and is executable
- [ ] All environment variables ready:
  - [ ] `GEMINI_API_KEY`
  - [ ] `QDRANT_URL`
  - [ ] `QDRANT_API_KEY`
  - [ ] `POSTGRES_URL`
  - [ ] `CORS_ORIGINS`
- [ ] Tested locally and works
- [ ] Committed and pushed to GitHub

---

## 🆘 Still Having Issues?

1. **Check Render Status:** https://status.render.com/
2. **Review Render Docs:** https://render.com/docs
3. **Check backend logs** in Render Dashboard
4. **Test dependencies locally:**
   ```bash
   python -m pip install -r requirements.txt
   ```

---

## ✅ Successful Deployment Indicators

After deployment, you should see:

1. **Build Logs:**
   ```
   ✅ Build completed successfully
   ```

2. **Deploy Logs:**
   ```
   Starting service...
   Application startup complete
   Uvicorn running on http://0.0.0.0:10000
   ```

3. **Health Check:**
   ```bash
   curl https://your-service.onrender.com/health
   # Returns: {"status": "healthy"}
   ```

4. **API Docs Accessible:**
   ```
   https://your-service.onrender.com/docs
   ```

---

## 📚 Related Documentation

- [Render Deployment Guide](./render-deployment.md)
- [Backend Architecture](../architecture/backend-architecture.md)
- [Backend Setup](../setup/backend-quickstart.md)
- [Gemini API Setup](../setup/gemini-setup.md)

---

**Last Updated:** 2025-12-29
**For:** Physical AI & Humanoid Robotics Backend
