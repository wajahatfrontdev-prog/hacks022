"""
Vercel serverless FastAPI handler for RAG chatbot backend.
This wraps the FastAPI app from ragbot-api/main.py to run on Vercel.
"""
import sys
import os
from pathlib import Path

# Add parent directory to path so we can import ragbot-api modules
sys.path.insert(0, str(Path(__file__).parent.parent / 'ragbot-api'))

# Import the FastAPI app
from main import app

# Vercel will call this handler for HTTP requests to /api/*
async def handler(request):
    """Vercel serverless handler for FastAPI."""
    # This is called directly by Vercel's runtime
    # Uvicorn/FastAPI will handle the request routing
    return app(request)
