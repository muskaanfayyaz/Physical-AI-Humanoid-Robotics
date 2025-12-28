#!/usr/bin/env bash
# Render build script for Physical AI Backend
# Enhanced version with better error handling

set -o errexit  # Exit on error
set -o pipefail # Exit on pipe failure

echo "🔧 Starting build process..."

# Upgrade pip, setuptools, and wheel first
echo "📦 Upgrading build tools..."
python -m pip install --upgrade pip setuptools wheel

# Install dependencies with verbose output
echo "📦 Installing Python dependencies..."
pip install --no-cache-dir -r requirements.txt

echo "✅ Build completed successfully!"
