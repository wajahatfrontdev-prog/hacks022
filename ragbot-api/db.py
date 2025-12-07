"""
Database models and connection management
"""
from sqlalchemy import create_engine, Column, String, DateTime, Text, Integer, UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import uuid
from config import settings

# Create database connection
engine = create_engine(
    settings.database_url,
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10,
    echo=False
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


class UserSession(Base):
    """User session model"""
    __tablename__ = "user_sessions"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=False, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    session_metadata = Column(Text, nullable=True)


class ChatMessage(Base):
    """Chat message model"""
    __tablename__ = "chat_messages"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, nullable=False, index=True)
    user_query = Column(Text, nullable=False)
    bot_response = Column(Text, nullable=False)
    mode = Column(String, default="fullbook", index=True)  # fullbook or selected
    selected_text = Column(Text, nullable=True)
    source_sections = Column(Text, nullable=True)  # JSON array of sources
    tokens_used = Column(Integer, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    user_rating = Column(Integer, nullable=True)  # 1-5 star rating


class IngestionLog(Base):
    """Track document ingestion operations"""
    __tablename__ = "ingestion_logs"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    doc_path = Column(String, nullable=False, index=True)
    chunks_created = Column(Integer, nullable=False)
    vectors_stored = Column(Integer, nullable=False)
    status = Column(String, default="success")  # success or failed
    error_message = Column(Text, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)


def get_db():
    """Dependency to get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def init_db():
    """Initialize database tables"""
    Base.metadata.create_all(bind=engine)
    print("✓ Database tables initialized")
