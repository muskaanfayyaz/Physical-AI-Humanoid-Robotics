#!/usr/bin/env bash
# Script to verify backend structure for Render deployment

echo "🔍 Verifying Physical AI Backend Structure..."
echo ""

# Check current directory
echo "📁 Current directory:"
pwd
echo ""

# List files in current directory
echo "📄 Files in current directory:"
ls -la
echo ""

# Check if app directory exists
if [ -d "app" ]; then
    echo "✅ app/ directory exists"
    echo "📄 Files in app/:"
    ls -la app/
else
    echo "❌ app/ directory NOT found"
    echo "⚠️  This is the problem! Render is in the wrong directory."
fi
echo ""

# Check if app/main.py exists
if [ -f "app/main.py" ]; then
    echo "✅ app/main.py exists"
else
    echo "❌ app/main.py NOT found"
fi
echo ""

# Check Python version
echo "🐍 Python version:"
python --version
echo ""

# Try to import the app
echo "🔧 Testing if app.main can be imported:"
python -c "from app.main import app; print('✅ Import successful!')" 2>&1
echo ""

# Show the correct uvicorn command
echo "✅ Correct uvicorn command should be:"
echo "   uvicorn app.main:app --host 0.0.0.0 --port \$PORT"
