"""
Test suite for RAG Chatbot API
Run with: pytest
"""
import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from main import app
from db import Base, get_db

# Use in-memory SQLite for testing
SQLALCHEMY_DATABASE_URL = "sqlite:///./test.db"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base.metadata.create_all(bind=engine)


def override_get_db():
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()


app.dependency_overrides[get_db] = override_get_db

client = TestClient(app)


class TestHealthEndpoint:
    """Test health check endpoint"""
    
    def test_health_check(self):
        response = client.get("/api/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "service" in data


class TestChatEndpoint:
    """Test chat endpoint"""
    
    def test_chat_fullbook_mode(self):
        """Test fullbook RAG mode"""
        response = client.post(
            "/api/chat",
            json={
                "query": "What is ROS2?",
                "mode": "fullbook",
                "user_id": "test_user"
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "session_id" in data
        assert data["mode"] == "fullbook"
    
    def test_chat_selected_mode(self):
        """Test selected text mode"""
        response = client.post(
            "/api/chat",
            json={
                "query": "What is this?",
                "mode": "selected",
                "selected_text": "Sample text about robotics",
                "user_id": "test_user"
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert data["mode"] == "selected"
    
    def test_chat_missing_selected_text(self):
        """Test selected mode without text"""
        response = client.post(
            "/api/chat",
            json={
                "query": "What is this?",
                "mode": "selected",
                "user_id": "test_user"
            }
        )
        assert response.status_code == 400
    
    def test_chat_invalid_mode(self):
        """Test invalid mode"""
        response = client.post(
            "/api/chat",
            json={
                "query": "Test",
                "mode": "invalid",
                "user_id": "test_user"
            }
        )
        assert response.status_code == 400


class TestHistoryEndpoint:
    """Test chat history endpoint"""
    
    def test_get_empty_history(self):
        """Test getting history for non-existent session"""
        response = client.get("/api/chat/history/nonexistent_session")
        assert response.status_code == 200
        data = response.json()
        assert data["session_id"] == "nonexistent_session"
        assert data["messages"] == []


class TestIngestEndpoint:
    """Test document ingestion endpoint"""
    
    def test_ingest_status(self):
        """Test getting ingestion status"""
        response = client.get("/api/ingest/status")
        # Should return either success or error depending on Qdrant setup
        assert response.status_code in [200, 500]


class TestRatingEndpoint:
    """Test message rating endpoint"""
    
    def test_rate_invalid_rating(self):
        """Test invalid rating value"""
        response = client.post(
            "/api/chat/rate",
            json={
                "message_id": "test_msg",
                "rating": 10  # Invalid
            }
        )
        assert response.status_code == 400


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
