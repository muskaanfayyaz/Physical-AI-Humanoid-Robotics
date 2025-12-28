# 🔍 RENDER DEBUGGING - Step by Step

**Error:** `ERROR: Error loading ASGI app. Could not import module "main".`

This means Render **cannot find your app**. Let's debug exactly why.

---

## 🚨 MOST COMMON ISSUE: Root Directory Wrong

### Check This FIRST

1. Go to **Render Dashboard**
2. Click on your service
3. Go to **"Settings"** tab
4. Scroll to **"Build & Deploy"** section
5. Look at **"Root Directory"**

**What it should be:**
```
backend
```

**Common mistakes:**
- ❌ Empty (blank)
- ❌ `/backend`
- ❌ `./backend`
- ❌ `/app/backend`
- ✅ `backend` (CORRECT!)

**If it's NOT exactly `backend`, that's your problem!**

---

## 📋 EXACT Settings You Need

Go through these settings **one by one** and verify:

### 1. Root Directory
```
backend
```
(Just the word "backend", nothing else)

### 2. Build Command
```bash
python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt
```

### 3. Start Command - TRY THESE IN ORDER:

**Option A (Try this FIRST):**
```bash
./start-render.sh
```

**Option B (If Option A doesn't work):**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Option C (If Options A & B don't work):**
```bash
python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

### 4. Auto-Deploy
```
Yes (enabled)
```

---

## 🔍 Step-by-Step Verification

### Step 1: Screenshot Your Settings

Take a screenshot of your Render **"Build & Deploy"** section and check:

- [ ] Root Directory = `backend`
- [ ] Build Command is correct
- [ ] Start Command is correct

### Step 2: Check Environment Variables

Go to **"Environment"** tab and verify:

- [ ] `GEMINI_API_KEY` is set (and not empty)
- [ ] `QDRANT_URL` is set
- [ ] `QDRANT_API_KEY` is set
- [ ] `POSTGRES_URL` is set

### Step 3: Force Rebuild

1. Go to **"Manual Deploy"**
2. Click **"Clear build cache & deploy"**
3. Wait for build to complete

---

## 🔧 Alternative: Add Debugging to Start Command

Replace your **Start Command** with this debugging version:

```bash
pwd && ls -la && ls -la app/ && python -c "from app.main import app; print('Import OK')" && uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

This will show you:
1. Current directory
2. Files in current directory
3. Files in app/ directory
4. If import works
5. Then start the server

**Check the logs** to see what it prints before the error.

---

## 📊 Read Your Logs Carefully

Go to **Logs** tab in Render and look for these clues:

### Clue 1: Current Directory
Look for lines like:
```
Current directory: /opt/render/project/src
```

**Should show:** `/opt/render/project/src` (if Root Directory = `backend`)

### Clue 2: Directory Contents
Look for:
```
app/
requirements.txt
build.sh
start-render.sh
```

**If you DON'T see `app/`**, your Root Directory is wrong!

### Clue 3: Error Location
```
ERROR: Error loading ASGI app. Could not import module "main".
```

**If it says "main" not "app.main"**, your Start Command is wrong!

---

## 🎯 The Correct File Structure

On Render, after build, the structure should be:

```
/opt/render/project/src/    ← You should be HERE (Root Directory = backend)
├── app/
│   ├── __init__.py
│   ├── main.py             ← Your FastAPI app
│   ├── config.py
│   ├── database.py
│   ├── routers/
│   ├── services/
│   └── schemas/
├── requirements.txt
├── build.sh
├── start-render.sh
└── runtime.txt
```

---

## 🔄 Try This Exact Configuration

Copy and paste these **EXACTLY** into Render:

### Settings → Build & Deploy

**Root Directory:**
```
backend
```

**Build Command:**
```
python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt
```

**Start Command:**
```
./start-render.sh
```

Click **"Save Changes"**

### Then Deploy

1. Go to **"Manual Deploy"**
2. Click **"Clear build cache & deploy"**
3. Watch the logs

---

## 🧪 What You Should See in Logs

### During Build:
```
📦 Upgrading build tools...
Successfully installed pip-24.x setuptools-xx.x wheel-xx.x
📦 Installing Python dependencies...
Successfully installed fastapi-x.x uvicorn-x.x ...
✅ Build completed successfully!
```

### During Start:
```
==========================================
🚀 Starting Physical AI Backend
==========================================

📁 Current directory: /opt/render/project/src

✅ app/ directory found
✅ app/main.py found
🐍 Python version: Python 3.11.x
🔧 Testing module import...
✅ Import successful!

🚀 Starting Uvicorn server...
Command: uvicorn app.main:app --host 0.0.0.0 --port 10000

INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:10000
```

---

## ❌ What You Might See (And What It Means)

### Error 1: "app/ directory not found"
```
❌ ERROR: app/ directory not found!
⚠️  Current directory is wrong. Should be in 'backend/'
```

**Fix:** Root Directory should be `backend`

### Error 2: "Could not import module 'main'"
```
ERROR: Error loading ASGI app. Could not import module "main".
```

**Fix:** Start Command should include `app.main` not just `main`

### Error 3: "No module named 'app'"
```
ModuleNotFoundError: No module named 'app'
```

**Fix:** Root Directory is wrong or you're not in `backend/` folder

---

## 🆘 Send Me This Information

If still not working, copy these from Render logs and send me:

1. **Root Directory setting:**
   ```
   Root Directory: [what does it say?]
   ```

2. **Start Command:**
   ```
   Start Command: [what does it say?]
   ```

3. **Current directory from logs:**
   ```
   Look for: "Current directory: /path/to/directory"
   ```

4. **Error message:**
   ```
   Copy the full error including any lines before it
   ```

5. **Files in directory:**
   ```
   Look for output of: ls -la
   ```

---

## ✅ Quick Verification Commands

Run these in **Render Shell** (if available) or add to start command:

```bash
# Check where you are
pwd

# Check what's in current directory
ls -la

# Check if app exists
ls -la app/

# Try to import
python -c "from app.main import app"
```

---

## 🎯 Most Likely Solutions

**90% of the time it's ONE of these:**

1. ✅ Root Directory = `backend` (not empty, not `/backend`)
2. ✅ Start Command = `./start-render.sh` OR `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
3. ✅ Clear build cache and redeploy

---

**Double-check these three things and it should work!** 🚀

---

**Last Updated:** 2025-12-29
