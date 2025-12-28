# Render Build Commands - Try These In Order

If you're getting `metadata-generation-failed` errors, try these build commands **in order** until one works:

---

## ✅ Option 1: Enhanced Build Script (Recommended)

**Build Command:**
```bash
./build.sh
```

**What it does:**
- Upgrades pip, setuptools, wheel
- Installs with no cache
- Better error messages

---

## ✅ Option 2: Direct Install with Upgraded Tools

**Build Command:**
```bash
python -m pip install --upgrade pip setuptools wheel && pip install --no-cache-dir -r requirements.txt
```

**Why this works:**
- Ensures latest build tools
- No caching issues
- Single command

---

## ✅ Option 3: Minimal Dependencies

**Build Command:**
```bash
pip install --upgrade pip && pip install --no-cache-dir -r requirements-minimal.txt
```

**Why this works:**
- Only essential packages
- No version pinning
- Faster build

Then add missing packages one by one in Render environment variables.

---

## ✅ Option 4: Install Without Binary Packages

**Build Command:**
```bash
pip install --upgrade pip && pip install --no-cache-dir --no-binary :all: -r requirements.txt
```

**Why this works:**
- Forces source installation
- Avoids prebuilt wheel issues
- Slower but more compatible

⚠️ **Warning:** This is VERY slow. Only use if other options fail.

---

## ✅ Option 5: One Package at a Time (Debug Mode)

**Build Command:**
```bash
pip install fastapi uvicorn sqlalchemy asyncpg qdrant-client google-generativeai python-dotenv pydantic pydantic-settings httpx python-multipart python-json-logger
```

**Why this works:**
- No requirements file
- Can see which package fails
- Flexible versions

---

## 🔍 Debugging: Find the Problem Package

If all above fail, use this build command to find which package is causing issues:

**Build Command:**
```bash
pip install --upgrade pip setuptools wheel && \
pip install fastapi && echo "✅ fastapi OK" && \
pip install uvicorn && echo "✅ uvicorn OK" && \
pip install sqlalchemy && echo "✅ sqlalchemy OK" && \
pip install asyncpg && echo "✅ asyncpg OK" && \
pip install qdrant-client && echo "✅ qdrant-client OK" && \
pip install google-generativeai && echo "✅ google-generativeai OK" && \
pip install python-dotenv && echo "✅ python-dotenv OK" && \
pip install pydantic && echo "✅ pydantic OK" && \
pip install pydantic-settings && echo "✅ pydantic-settings OK" && \
pip install httpx && echo "✅ httpx OK" && \
pip install python-multipart && echo "✅ python-multipart OK" && \
pip install python-json-logger && echo "✅ python-json-logger OK" && \
echo "🎉 All packages installed successfully!"
```

Check the logs to see where it fails.

---

## 📋 Current Configuration

**Your current setup should be:**

**Root Directory:**
```
backend
```

**Build Command (try first):**
```bash
./build.sh
```

**Start Command:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Runtime:**
```
Python 3.11
```

---

## 🆘 Still Failing?

1. **Check Render Status:** https://status.render.com/
2. **Copy FULL error log** from Render and send it
3. **Try requirements-minimal.txt:**
   - Change build command to: `pip install -r requirements-minimal.txt`
4. **Check if it's a specific package:**
   - Use debug build command above
   - Share which package fails

---

## ✅ Success Indicators

When build succeeds, you'll see:

```
📦 Upgrading build tools...
Successfully installed pip-24.x setuptools-xx.x wheel-xx.x
📦 Installing Python dependencies...
Successfully installed fastapi-x.x uvicorn-x.x sqlalchemy-x.x ...
✅ Build completed successfully!
```

---

**Last Updated:** 2025-12-29
