@echo off

echo 🚀 Starting Physical AI RAG Backend...

REM Check if virtual environment exists
if not exist "venv\" (
    echo 📦 Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
echo 🔧 Activating virtual environment...
call venv\Scripts\activate.bat

REM Install dependencies if needed
if not exist "venv\installed" (
    echo 📥 Installing dependencies...
    pip install -r requirements.txt
    type nul > venv\installed
)

REM Check if .env exists
if not exist ".env" (
    echo ⚠️  No .env file found. Copying from .env.example...
    copy .env.example .env
    echo ⚠️  Please update .env with your credentials before running!
    pause
    exit /b 1
)

REM Run the server
echo ✅ Starting FastAPI server...
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
