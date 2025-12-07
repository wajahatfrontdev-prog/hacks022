"""
FastAPI main application for RAG Chatbot
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from config import settings
from db import init_db
from routers import chat, ingest


# Startup event
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("🚀 Starting RAG Chatbot API...")
    init_db()
    print("✓ Database initialized")
    yield
    # Shutdown
    print("🛑 Shutting down RAG Chatbot API...")


# Create FastAPI app
app = FastAPI(
    title=settings.api_title,
    description="RAG Chatbot API for Physical AI Humanoid Robotics Book",
    version=settings.api_version,
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router)
app.include_router(ingest.router)


# Root endpoint
@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "service": "Physical AI Humanoid Robotics RAG Chatbot API",
        "version": settings.api_version,
        "docs": "/docs",
        "health": "/api/health"
    }


@app.get("/api/config")
async def get_config():
    """Get public configuration"""
    return {
        "openai_model": settings.openai_model,
        "chunk_size": settings.chunk_size,
        "top_k_results": settings.top_k_results,
        "rate_limit_calls": settings.rate_limit_calls,
        "rate_limit_period": settings.rate_limit_period
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
