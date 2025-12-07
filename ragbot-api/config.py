"""
Configuration module for RAG Chatbot backend
"""
import os
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings from environment variables"""
    
    # Groq Configuration (Free, Fast API for LLM inference)
    groq_api_key: str = os.getenv(
        "GROQ_API_KEY",
        "<REDACTED_GROQ>"
    )
    groq_model: str = "llama-3.3-70b-versatile"
    # Note: Groq doesn't have embeddings API; we'll use local embeddings or fallback search
    
    # Qdrant Configuration
    qdrant_url: str = os.getenv(
        "QDRANT_URL",
        "https://350100b4-1d42-4065-b3cd-1005505d0044.europe-west3-0.gcp.cloud.qdrant.io"
    )
    qdrant_api_key: str = os.getenv(
        "QDRANT_API_KEY",
        "<REDACTED_JWT>"
    )
    qdrant_collection_name: str = "robotics_docs"
    qdrant_vector_size: int = 384  # sentence-transformers all-MiniLM-L6-v2 embedding dimension
    
    # Database Configuration
    database_url: str = os.getenv(
        "DATABASE_URL",
        "postgresql://neondb_owner:npg_mrIvVkhZQx37@ep-purple-resonance-a40w6551-pooler.us-east-1.aws.neon.tech/neondb"
    )
    
    # Frontend Configuration
    frontend_url: str = os.getenv(
        "FRONTEND_URL",
        "https://physical-ai-humanoid-robotics-hacka.vercel.app/"
    )
    
    # RAG Configuration
    chunk_size: int = 500
    chunk_overlap: int = 100
    top_k_results: int = 10
    
    # API Configuration
    api_title: str = "Physical AI Humanoid Robotics RAG Chatbot API"
    api_version: str = "1.0.0"
    cors_origins: list = [
        "http://localhost:3000",
        "http://localhost:8000",
        "https://physical-ai-humanoid-robotics-hacka.vercel.app",
        "https://localhost",
        "*"
    ]
    
    # Rate Limiting
    rate_limit_calls: int = 100
    rate_limit_period: int = 3600  # 1 hour
    
    class Config:
        env_file = ".env"
        case_sensitive = False


settings = Settings()
