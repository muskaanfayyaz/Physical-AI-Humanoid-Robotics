#!/bin/bash

# Startup script for Physical AI RAG Backend

echo "🚀 Starting Physical AI RAG Backend..."

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install dependencies if needed
if [ ! -f "venv/installed" ]; then
    echo "📥 Installing dependencies..."
    pip install -r requirements.txt
    touch venv/installed
fi

# Check if .env exists
if [ ! -f ".env" ]; then
    echo "⚠️  No .env file found. Copying from .env.example..."
    cp .env.example .env
    echo "⚠️  Please update .env with your credentials before running!"
    exit 1
fi

# Run the server
echo "✅ Starting FastAPI server..."
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
