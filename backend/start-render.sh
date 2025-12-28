#!/usr/bin/env bash
# Render start script for Physical AI Backend
# Enhanced with debugging

set -o errexit

echo "=========================================="
echo "🚀 Starting Physical AI Backend"
echo "=========================================="
echo ""

# Show current directory
echo "📁 Current directory: $(pwd)"
echo ""

# List files to verify structure
echo "📄 Directory contents:"
ls -la
echo ""

# Verify app directory exists
if [ ! -d "app" ]; then
    echo "❌ ERROR: app/ directory not found!"
    echo "⚠️  Current directory is wrong. Should be in 'backend/'"
    exit 1
fi

echo "✅ app/ directory found"
echo ""

# Verify app/main.py exists
if [ ! -f "app/main.py" ]; then
    echo "❌ ERROR: app/main.py not found!"
    exit 1
fi

echo "✅ app/main.py found"
echo ""

# Show Python version
echo "🐍 Python version: $(python --version)"
echo ""

# Test import
echo "🔧 Testing module import..."
python -c "from app.main import app; print('✅ Import successful!')" || {
    echo "❌ Import failed!"
    exit 1
}
echo ""

# Start the server
echo "🚀 Starting Uvicorn server..."
echo "Command: uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}"
echo ""

exec uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}
