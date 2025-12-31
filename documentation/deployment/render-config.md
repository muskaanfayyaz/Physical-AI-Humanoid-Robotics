# Render Deployment Configuration

Quick reference for deploying to Render.

---

## 📋 Render Web Service Settings

### Basic Settings
```
Name:           physical-ai-backend
Region:         Oregon (US West) or Frankfurt (EU)
Branch:         main
Root Directory: backend
Runtime:        Python 3
```

### Build & Deploy

**Build Command:**
```bash
./build.sh
```

**Or if build.sh doesn't work:**
```bash
pip install --upgrade pip && pip install -r requirements.txt
```

**Start Command:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

### Instance Type
```
Instance Type: Free
```

---

## 🔑 Environment Variables

Add these in Render Dashboard → Environment:

### Required Variables

| Variable | Value | Where to Get |
|----------|-------|--------------|
| `GEMINI_API_KEY` | `your-gemini-api-key` | https://aistudio.google.com/app/apikey |
| `QDRANT_URL` | `https://your-cluster.qdrant.io` | Qdrant Cloud Dashboard |
| `QDRANT_API_KEY` | `your-qdrant-api-key` | Qdrant Cloud Dashboard |
| `POSTGRES_URL` | `postgresql://user:pass@host/db?sslmode=require` | Neon Dashboard → Connection String |

### Optional Variables

| Variable | Default Value | Description |
|----------|---------------|-------------|
| `CORS_ORIGINS` | `["http://localhost:3000"]` | Allowed origins |
| `DEBUG` | `False` | Debug mode |
| `API_PREFIX` | `/api/v1` | API prefix |

---

## 📁 Required Files

Make sure these files exist in `backend/`:

- ✅ `requirements.txt` - Python dependencies (no psycopg2-binary!)
- ✅ `runtime.txt` - Python version (python-3.11.9)
- ✅ `build.sh` - Build script (executable)
- ✅ `app/` - Application code

---

## ✅ Deployment Checklist

Before clicking "Create Web Service":

1. **Repository Setup:**
   - [ ] Code pushed to GitHub
   - [ ] `backend/` directory contains all files
   - [ ] No `psycopg2-binary` in requirements.txt

2. **External Services Ready:**
   - [ ] Neon Postgres database created
   - [ ] Qdrant Cloud cluster created
   - [ ] Gemini API key obtained

3. **Render Settings:**
   - [ ] Root Directory set to `backend`
   - [ ] Build Command: `./build.sh`
   - [ ] Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
   - [ ] Instance Type: Free
   - [ ] All environment variables added

4. **Post-Deployment:**
   - [ ] Service deployed successfully
   - [ ] Health check passes: `https://your-service.onrender.com/health`
   - [ ] API docs accessible: `https://your-service.onrender.com/docs`

---

## 🔗 After Deployment

Your backend will be available at:
```
https://physical-ai-backend-xxxx.onrender.com
```

**Test endpoints:**
- Health: `https://your-service.onrender.com/health`
- API Docs: `https://your-service.onrender.com/docs`
- API: `https://your-service.onrender.com/api/v1/`

---

## 🆘 Troubleshooting

See [Render Troubleshooting Guide](../documentation/deployment/render-troubleshooting.md)

---

**Updated:** 2025-12-29
