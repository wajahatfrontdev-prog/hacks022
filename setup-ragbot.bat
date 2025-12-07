@echo off
REM RAG Chatbot Development Setup Script for Windows
REM Run this to quickly set up the development environment

setlocal enabledelayedexpansion

cls
echo ================================================
echo RAG Chatbot Development Environment Setup
echo ================================================
echo.

REM Check Python
echo [1/6] Checking Python installation...
python3 --version >nul 2>&1
if errorlevel 1 (
    echo X Python 3 not found. Please install Python 3.9+
    exit /b 1
)
for /f "tokens=2" %%i in ('python3 --version 2^>^&1') do set PYTHON_VERSION=%%i
echo [OK] Python %PYTHON_VERSION% found
echo.

REM Setup venv
echo [2/6] Setting up Python virtual environment...
if not exist "ragbot-api\venv" (
    python3 -m venv ragbot-api\venv
    echo [OK] Virtual environment created
) else (
    echo [OK] Virtual environment already exists
)
call ragbot-api\venv\Scripts\activate.bat
echo.

REM Install dependencies
echo [3/6] Installing Python dependencies...
cd ragbot-api
pip install -q --upgrade pip
pip install -q -r requirements.txt
cd ..
echo [OK] Python dependencies installed
echo.

REM Setup env file
echo [4/6] Setting up environment variables...
if not exist "ragbot-api\.env" (
    copy ragbot-api\.env.example ragbot-api\.env
    echo [OK] Created .env file from template
    echo.
    echo [WARNING] Update ragbot-api\.env with your credentials:
    echo    - OPENAI_API_KEY
    echo    - QDRANT_URL and QDRANT_API_KEY
    echo    - DATABASE_URL
) else (
    echo [OK] .env file already exists
)
echo.

REM Install npm dependencies
echo [5/6] Installing Node.js dependencies...
call npm install -q
echo [OK] Node dependencies installed
echo.

REM Summary
echo [6/6] Setup complete!
echo.
echo ================================================
echo Setup Complete!
echo ================================================
echo.
echo Next steps:
echo.
echo 1. Update credentials in ragbot-api\.env:
echo    - OPENAI_API_KEY
echo    - QDRANT_URL and QDRANT_API_KEY
echo    - DATABASE_URL
echo.
echo 2. Start backend server:
echo    cd ragbot-api
echo    venv\Scripts\activate.bat
echo    python -m uvicorn main:app --reload
echo.
echo 3. In another terminal, start Docusaurus:
echo    npm start
echo.
echo 4. Ingest documents (when ready):
echo    python ragbot-ingest.py
echo.
echo 5. Access:
echo    Frontend: http://localhost:3000
echo    RAG Chat: http://localhost:3000/ragbot
echo    API Docs: http://localhost:8000/docs
echo.
