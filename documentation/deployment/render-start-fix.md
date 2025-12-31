# 🚨 FIX: "Could not import module 'main'" Error

**Error Message:**
```
ERROR: Error loading ASGI app. Could not import module "main".
```

**Good News:** Build succeeded! ✅ This is just a start command issue.

---

## ✅ SOLUTION - Update Start Command

Go to **Render Dashboard** → **Your Service** → **Settings**

### Option 1: Use Start Script (Recommended)

**Start Command:**
```bash
./start-render.sh
```

**Why this works:**
- Handles all edge cases
- Uses correct module path
- Already created and executable

---

### Option 2: Direct Command

**Start Command:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Important:** Note `app.main:app` not just `main:app`

---

### Option 3: Explicit Python Module

**Start Command:**
```bash
python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

---

## 🎯 Complete Render Configuration

### Settings Tab

| Setting | Value |
|---------|-------|
| **Root Directory** | `backend` |
| **Build Command** | `python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt` |
| **Start Command** | `./start-render.sh` OR `uvicorn app.main:app --host 0.0.0.0 --port $PORT` |
| **Instance Type** | Free |

### Environment Variables

Make sure these are set:

- ✅ `GEMINI_API_KEY` = Your Gemini API key
- ✅ `QDRANT_URL` = Your Qdrant cluster URL
- ✅ `QDRANT_API_KEY` = Your Qdrant API key
- ✅ `POSTGRES_URL` = Your Neon Postgres connection string

---

## 🔍 Why This Happened

The error means Render was trying to run:
```bash
uvicorn main:app  # ❌ Wrong - no 'main' module at root
```

But your app structure is:
```
backend/
└── app/
    └── main.py  # ✅ Correct path is 'app.main'
```

So you need:
```bash
uvicorn app.main:app  # ✅ Correct
```

---

## ✅ After Fixing

Click **"Save Changes"** then **"Manual Deploy"**

You should see:
```
🚀 Starting Physical AI Backend...
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:10000
```

---

## 🧪 Test Your Deployment

Once deployed, test these endpoints:

### Health Check
```bash
curl https://your-service.onrender.com/health
```

**Expected:**
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

### API Documentation
Visit:
```
https://your-service.onrender.com/docs
```

Should show interactive API documentation.

---

## 🆘 Still Having Issues?

### Check 1: Verify Root Directory

In Render Settings:
```
Root Directory: backend
```

**NOT** empty, **NOT** `/backend`, just `backend`

### Check 2: Verify Files Exist

The following files should exist in your `backend/` directory:
- ✅ `app/main.py`
- ✅ `requirements.txt`
- ✅ `start-render.sh`
- ✅ `build.sh`
- ✅ `runtime.txt`

### Check 3: Check Logs

Look for:
```
ModuleNotFoundError: No module named 'app'
```

If you see this, your **Root Directory** is wrong.

---

## 📋 Quick Checklist

Before deploying:

- [ ] Root Directory = `backend`
- [ ] Build Command = `python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt`
- [ ] Start Command = `./start-render.sh` OR `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
- [ ] All environment variables set
- [ ] Changes committed and pushed to GitHub

---

## 🎉 Success!

Once working, your backend will be live at:
```
https://physical-ai-backend-xxxx.onrender.com
```

You can integrate it with your Docusaurus frontend by updating the API endpoint configuration.

---

**Last Updated:** 2025-12-29
**Status:** Build succeeds ✅ | Just need correct start command
