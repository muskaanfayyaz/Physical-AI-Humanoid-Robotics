#!/usr/bin/env bash
# Render start script for Physical AI Backend

set -o errexit

echo "🚀 Starting Physical AI Backend..."

# Start uvicorn with correct module path
exec uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}
