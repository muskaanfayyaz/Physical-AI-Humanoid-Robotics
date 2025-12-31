# 🚨 RENDER DEPLOYMENT - QUICK FIX GUIDE

**Problem:** `metadata-generation-failed` error on Render

**Status:** Multiple fixes applied - Try these in order

---

## ⚡ QUICK FIX - Try This FIRST

### Step 1: Update Your Render Build Command

Go to Render Dashboard → Your Service → Settings

**Change Build Command to:**
```bash
python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt
```

### Step 2: Save and Redeploy

Click **"Save Changes"** then **"Manual Deploy"** → **"Deploy latest commit"**

---

## ✅ What I Fixed

1. ✅ **Removed `psycopg2-binary`** - causes build errors
2. ✅ **Updated to flexible versions** - allows compatible package selection
3. ✅ **Enhanced build script** - upgrades pip/setuptools first
4. ✅ **Updated runtime** - uses Python 3.11 (stable)
5. ✅ **Created fallback** - minimal requirements file

---

## 🔄 IF STILL FAILING - Try These

### Option A: Use Minimal Requirements

**Build Command:**
```bash
pip install --no-cache-dir -r requirements-minimal.txt
```

**Location:** `backend/requirements-minimal.txt` (already created)

### Option B: Install Without Build Script

**Build Command:**
```bash
pip install fastapi uvicorn sqlalchemy asyncpg qdrant-client google-generativeai python-dotenv pydantic pydantic-settings httpx python-multipart python-json-logger
```

### Option C: Debug Mode - Find Problem Package

**Build Command:**
```bash
pip install --upgrade pip setuptools wheel && \
pip install fastapi && echo "✅ fastapi" && \
pip install uvicorn && echo "✅ uvicorn" && \
pip install sqlalchemy && echo "✅ sqlalchemy" && \
pip install asyncpg && echo "✅ asyncpg" && \
pip install qdrant-client && echo "✅ qdrant" && \
pip install google-generativeai && echo "✅ gemini" && \
pip install python-dotenv pydantic pydantic-settings httpx python-multipart python-json-logger
```

Check logs to see which package fails.

---

## 📊 Your Updated Files

### backend/requirements.txt
```python
# Flexible version ranges (allows compatible updates)
fastapi>=0.110.0,<1.0.0
uvicorn>=0.29.0,<1.0.0
sqlalchemy[asyncio]>=2.0.27,<3.0.0
asyncpg>=0.29.0,<1.0.0
qdrant-client>=1.8.0,<2.0.0
google-generativeai>=0.4.0,<1.0.0
python-dotenv>=1.0.0,<2.0.0
pydantic>=2.6.0,<3.0.0
pydantic-settings>=2.2.0,<3.0.0
httpx>=0.26.0,<1.0.0
python-multipart>=0.0.9,<1.0.0
python-json-logger>=2.0.0,<3.0.0
```

### backend/runtime.txt
```
python-3.11
```

### backend/build.sh
```bash
#!/usr/bin/env bash
set -o errexit
python -m pip install --upgrade pip setuptools wheel
pip install --no-cache-dir -r requirements.txt
```

---

## 🎯 Render Configuration

**Root Directory:**
```
backend
```

**Build Command (Option 1 - Recommended):**
```bash
python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt
```

**Build Command (Option 2 - If Option 1 fails):**
```bash
./build.sh
```

**Start Command:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Instance Type:**
```
Free
```

---

## 🔍 Need to See Error Details?

If you're still getting errors, I need to see:

1. **Full error message** from Render build logs
2. **Which package** is failing (look for "Building wheel for...")
3. **Python version** being used (check logs for "Python 3.x.x")

**How to get logs:**
- Render Dashboard → Your Service → Logs tab
- Copy the entire build output
- Share the lines around the error

---

## 📚 Detailed Documentation

- **[Render Build Commands](backend/RENDER_BUILD_COMMANDS.md)** - All build options
- **[Render Configuration](backend/RENDER_CONFIG.md)** - Complete config reference
- **[Troubleshooting Guide](documentation/deployment/render-troubleshooting.md)** - Detailed troubleshooting

---

## ✅ Commit These Changes

```bash
git add .
git commit -m "fix: apply comprehensive Render deployment fixes"
git push origin main
```

Then try redeploying on Render.

---

**Updated:** 2025-12-29
**Quick Help:** Try the Quick Fix first, then options A, B, C if needed
