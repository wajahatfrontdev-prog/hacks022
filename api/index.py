"""
Vercel serverless FastAPI handler for RAG chatbot backend.
This wraps the FastAPI app from ragbot-api/main.py to run on Vercel.
"""
import sys
import os
from pathlib import Path

# Add parent directory to path so we can import ragbot-api modules
sys.path.insert(0, str(Path(__file__).parent.parent / 'ragbot-api'))

# Set environment defaults for Vercel
os.environ.setdefault('GROQ_API_KEY', '')
os.environ.setdefault('QDRANT_URL', '')
os.environ.setdefault('QDRANT_API_KEY', '')
os.environ.setdefault('DATABASE_URL', 'sqlite:///./chat.db')
os.environ.setdefault('FRONTEND_URL', 'https://hacks022.vercel.app')

# Import the FastAPI app
from main import app
from fastapi.middleware.cors import CORSMiddleware

# Add CORS middleware for frontend on Vercel
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Vercel calls this handler for HTTP requests to /api/*
async def handler(request):
    """Vercel serverless handler for FastAPI."""
    return app(request)
