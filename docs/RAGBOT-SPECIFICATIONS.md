# RAGBOT Specifications

## Physical AI Humanoid Robotics Book – Integrated RAG Chatbot System

**Version:** 1.0.0  
**Date:** December 2024  
**Status:** Production Ready  

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Data Flow](#data-flow)
3. [API Specifications](#api-specifications)
4. [Vector Ingestion Pipeline](#vector-ingestion-pipeline)
5. [Database Schema](#database-schema)
6. [Security Checklist](#security-checklist)
7. [Deployment Instructions](#deployment-instructions)
8. [Rate Limiting Strategy](#rate-limiting-strategy)
9. [Maintenance Guide](#maintenance-guide)

---

## System Architecture

### ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       FRONTEND LAYER                            │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │           Docusaurus Site + React Components              │ │
│  │  ┌──────────────────────────────────────────────────────┐  │ │
│  │  │      ChatInterface Component                         │  │ │
│  │  │  • Mode Selector (fullbook | selected)              │  │ │
│  │  │  • Message History Management                       │  │ │
│  │  │  • Source Display                                   │  │ │
│  │  │  • Real-time Streaming                              │  │ │
│  │  └──────────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                      HTTP/HTTPS REST API
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                     BACKEND API LAYER                           │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              FastAPI Application (main.py)               │ │
│  │  ┌──────────────────────────────────────────────────────┐  │ │
│  │  │           Routers                                    │  │ │
│  │  │  • /api/chat (POST) - Chat endpoint                 │  │ │
│  │  │  • /api/chat/history (GET) - Chat history           │  │ │
│  │  │  • /api/chat/rate (POST) - Rating system            │  │ │
│  │  │  • /api/ingest/docs (POST) - Ingest documents       │  │ │
│  │  │  • /api/ingest/status (GET) - Ingestion status      │  │ │
│  │  └──────────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
         ↙                       ↓                       ↘
      [Mode Selection]    [LLM Service]         [Vector Store]
         ↙                       ↓                       ↘
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  Mode Routing    │  │   OpenAI API     │  │  Qdrant Cloud    │
│  ┌────────────┐  │  │  (GPT-4 Turbo)   │  │  Vector Store    │
│  │ FULLBOOK   │  │  │                  │  │                  │
│  │ Mode RAG   │  │  │ • Embeddings     │  │ • Collections    │
│  │            │  │  │ • Chat Completio │  │ • Vectors        │
│  │ + Qdrant   │  │  │ • Responses      │  │ • Metadata       │
│  └────────────┘  │  └──────────────────┘  └──────────────────┘
│  ┌────────────┐  │                             ↑
│  │ SELECTED   │  │                             │
│  │ Mode Only  │  │                    [FULLBOOK MODE]
│  │ No Vector  │  │                    • Query Embedding
│  │ Search     │  │                    • Vector Search
│  └────────────┘  │                    • Top K Retrieval
└──────────────────┘
         ↓
┌──────────────────────────────────────────────────────────────────┐
│                    DATABASE LAYER                               │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │      Neon Serverless PostgreSQL                           │ │
│  │  ┌──────────────────────────────────────────────────────┐  │ │
│  │  │  Tables:                                             │  │ │
│  │  │  • user_sessions (session management)                │  │ │
│  │  │  • chat_messages (conversation history)              │  │ │
│  │  │  • ingestion_logs (document processing logs)         │  │ │
│  │  └──────────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

### Component Overview

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Frontend** | React + Docusaurus | Chat UI, document viewing |
| **Backend API** | FastAPI | Request routing, business logic |
| **LLM** | OpenAI GPT-4 Turbo | Answer generation, embeddings |
| **Vector Store** | Qdrant Cloud | Document vector storage |
| **Database** | Neon PostgreSQL | Session & chat logging |
| **Hosting** | Vercel (FE) / Render/Fly (BE) | Deployment targets |

---

## Data Flow

### Flow 1: FULLBOOK RAG Mode

```
User Query
    ↓
[ChatInterface Component]
    ↓
POST /api/chat {mode: "fullbook", query: "..."}
    ↓
[FastAPI Router: chat.py]
    ↓
[Generate Query Embedding] (OpenAI API)
    ↓
[Vector Search in Qdrant] (retrieve top-5 chunks)
    ↓
[Build Context] (combine relevant sections)
    ↓
[OpenAI Agent] (GPT-4 with retrieved context)
    ↓
[Generate Response] + [Source References]
    ↓
[Store in Neon] (session, messages, sources)
    ↓
ChatResponse {response, sources, session_id, tokens_used}
    ↓
[Display in UI] + [Show Sources]
```

### Flow 2: SELECTED Mode

```
User Selects Text
    ↓
[SelectedTextBox Component]
    ↓
User Enters Query
    ↓
POST /api/chat {mode: "selected", query: "...", selected_text: "..."}
    ↓
[FastAPI Router: chat.py]
    ↓
[Skip Qdrant Search] (no vector lookup)
    ↓
[Build Context] (ONLY from selected text)
    ↓
[OpenAI Agent] (GPT-4 with selected context only)
    ↓
[Generate Response] (constrained to selection)
    ↓
[Store in Neon] (mark mode as 'selected')
    ↓
ChatResponse {response, sources: [], session_id, tokens_used}
    ↓
[Display in UI]
```

### Flow 3: Document Ingestion

```
Admin Triggers Ingestion
    ↓
[ragbot-ingest.py script OR POST /api/ingest/docs]
    ↓
[Read all .md/.mdx files from /docs]
    ↓
[Extract Sections] (by headers, chunk size 500 tokens)
    ↓
[Generate Embeddings] (OpenAI text-embedding-3-small)
    ↓
[Batch Upload] (chunked, 100 vectors per batch)
    ↓
[Qdrant Upsert] (with metadata: filename, module, section, content)
    ↓
[Log to Neon] (ingestion_logs table)
    ↓
✓ Vectors Ready for Search
```

---

## API Specifications

### Base URL

- **Development:** `http://localhost:8000`
- **Production:** `https://ragbot-api.example.com`

### Authentication

All endpoints are currently **open** (for development). For production, add API keys:

```python
# In config.py, add:
API_KEY = os.getenv("RAGBOT_API_KEY")

# In routers, use Depends(verify_api_key)
```

### Endpoints

#### 1. POST `/api/chat`

**Chat Request**

```json
{
  "query": "What is ROS2?",
  "mode": "fullbook",
  "selected_text": null,
  "session_id": "session_123...",
  "user_id": "user_456..."
}
```

**Parameters:**

| Name | Type | Required | Description |
|------|------|----------|-------------|
| query | string | Yes | User question |
| mode | enum | Yes | "fullbook" or "selected" |
| selected_text | string | No | Required if mode="selected" |
| session_id | string | No | Session UUID (auto-generated if null) |
| user_id | string | No | User identifier (default: "anonymous") |

**Response:**

```json
{
  "response": "ROS2 is the Robot Operating System version 2...",
  "session_id": "session_123...",
  "sources": [
    {
      "score": 0.92,
      "filename": "module01-ros2/02-ros2-nodes-topics.md",
      "module": "module01-ros2",
      "section": "ROS2 Fundamentals",
      "content": "ROS2 (Robot Operating System version 2) is a middleware...",
      "full_path": "module01-ros2/02-ros2-nodes-topics.md"
    }
  ],
  "tokens_used": 487,
  "mode": "fullbook"
}
```

**Status Codes:**

- `200 OK` - Successful response
- `400 Bad Request` - Invalid query or missing required field
- `500 Internal Server Error` - Processing failed

---

#### 2. GET `/api/chat/history/{session_id}`

**Response:**

```json
{
  "session_id": "session_123...",
  "messages": [
    {
      "query": "What is ROS2?",
      "response": "ROS2 is the Robot Operating System...",
      "mode": "fullbook",
      "sources": [...],
      "created_at": "2024-12-06T10:30:00Z"
    }
  ]
}
```

---

#### 3. POST `/api/chat/rate`

**Parameters:**

```json
{
  "message_id": "msg_123...",
  "rating": 5
}
```

**Response:**

```json
{
  "status": "success",
  "message_id": "msg_123...",
  "rating": 5
}
```

---

#### 4. POST `/api/ingest/docs`

**Request:**

```json
{
  "force_reingest": false
}
```

**Response:**

```json
{
  "status": "success",
  "message": "Documents ingested successfully",
  "files_processed": 15,
  "chunks_created": 247,
  "vectors_stored": 247,
  "point_ids": ["123456789", "123456790", ...]
}
```

---

#### 5. GET `/api/ingest/status`

**Response:**

```json
{
  "status": "success",
  "collection": {
    "name": "robotics_docs",
    "point_count": 247,
    "vector_size": 1536
  }
}
```

---

#### 6. GET `/api/health`

**Response:**

```json
{
  "status": "healthy",
  "service": "RAG Chatbot API",
  "version": "1.0.0"
}
```

---

## Vector Ingestion Pipeline

### Pipeline Steps

1. **Document Discovery**
   - Scan `/docs` recursively
   - Identify `.md` and `.mdx` files
   - Extract module name from path

2. **Text Chunking**
   - Split by headers (H1, H2, H3)
   - Chunk size: 500 tokens
   - Overlap: 100 tokens
   - Preserve section hierarchy

3. **Metadata Extraction**
   - `filename`: relative file path
   - `module`: extracted from folder structure
   - `section`: header/title
   - `content`: chunk text (up to 1000 chars for preview)
   - `full_path`: full relative path
   - `created_at`: ingestion timestamp

4. **Embedding Generation**
   - Model: `text-embedding-3-small`
   - Vector dimension: 1536
   - Batch size: 25 texts per request
   - Rate limit: 0.1s between batches

5. **Vector Storage**
   - Upload to Qdrant Cloud
   - Batch size: 100 vectors per upsert
   - Payload: full metadata
   - Distance metric: COSINE

6. **Logging**
   - Record in `ingestion_logs` table
   - Status: success/failed
   - Error message if failed

### Configuration

```python
# In config.py
chunk_size: int = 500
chunk_overlap: int = 100
top_k_results: int = 5
qdrant_vector_size: int = 1536
```

---

## Database Schema

### Table: `user_sessions`

```sql
CREATE TABLE user_sessions (
    id VARCHAR PRIMARY KEY,
    user_id VARCHAR NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    metadata TEXT,
    
    INDEX idx_user_id (user_id),
    INDEX idx_created_at (created_at)
);
```

**Purpose:** Track user sessions and conversation context

---

### Table: `chat_messages`

```sql
CREATE TABLE chat_messages (
    id VARCHAR PRIMARY KEY,
    session_id VARCHAR NOT NULL,
    user_query TEXT NOT NULL,
    bot_response TEXT NOT NULL,
    mode VARCHAR DEFAULT 'fullbook',
    selected_text TEXT,
    source_sections TEXT,  -- JSON array
    tokens_used INTEGER,
    created_at TIMESTAMP DEFAULT NOW(),
    user_rating INTEGER CHECK (user_rating >= 1 AND user_rating <= 5),
    
    INDEX idx_session_id (session_id),
    INDEX idx_mode (mode),
    INDEX idx_created_at (created_at)
);
```

**Purpose:** Store conversation history and enable analytics

---

### Table: `ingestion_logs`

```sql
CREATE TABLE ingestion_logs (
    id VARCHAR PRIMARY KEY,
    doc_path VARCHAR NOT NULL,
    chunks_created INTEGER NOT NULL,
    vectors_stored INTEGER NOT NULL,
    status VARCHAR DEFAULT 'success',
    error_message TEXT,
    created_at TIMESTAMP DEFAULT NOW(),
    
    INDEX idx_doc_path (doc_path),
    INDEX idx_created_at (created_at)
);
```

**Purpose:** Track document ingestion operations for debugging

---

## Security Checklist

### ✓ Implementation Status

- [x] Environment variables for secrets (`.env`)
- [x] CORS configuration
- [x] Rate limiting support (configurable)
- [x] Database password encryption ready
- [x] API key injection ready
- [x] Secure headers middleware ready

### 🔒 Recommended Security Measures

#### For Production:

1. **API Authentication**
   ```python
   # Add to config.py
   API_KEY = os.getenv("RAGBOT_API_KEY")
   
   # Add to main.py
   from fastapi.security import HTTPBearer
   security = HTTPBearer()
   
   # Use in routers
   @router.post("/api/chat")
   async def chat(request: ChatRequest, credentials: HTTPAuthCredential = Depends(security)):
       verify_api_key(credentials.credentials)
   ```

2. **Database Security**
   - Use connection pooling (✓ configured in `db.py`)
   - Neon provides SSL by default
   - Add IP whitelisting at database level

3. **Rate Limiting**
   ```python
   from slowapi import Limiter
   from slowapi.util import get_remote_address
   
   limiter = Limiter(key_func=get_remote_address)
   app.state.limiter = limiter
   
   @router.post("/api/chat")
   @limiter.limit("10/minute")
   async def chat(...):
       pass
   ```

4. **Input Validation**
   - All Pydantic models with validation ✓
   - Query length limit: 2000 chars
   - Response length limit: 5000 chars

5. **HTTPS/TLS**
   - Enforce in frontend
   - Backend behind reverse proxy
   - Certificate management via platform

6. **Logging & Monitoring**
   ```python
   import logging
   logger = logging.getLogger(__name__)
   logger.info(f"Chat request from {user_id}")
   ```

7. **Data Privacy**
   - Chat history encrypted at rest (Neon)
   - GDPR compliance for user data
   - Data retention policy: 90 days

### 🚨 Secrets Management

```bash
# .env (local development only)
OPENAI_API_KEY=<REDACTED>
QDRANT_API_KEY=<REDACTED>
DATABASE_URL=postgresql://...

# Production (use platform secrets):
# Vercel: Settings → Environment Variables
# Render/Fly: Secrets management UI
```

---

## Deployment Instructions

### Prerequisites

- Python 3.9+
- Node.js 18+
- Git
- Accounts: OpenAI, Qdrant Cloud, Neon, Vercel, Render/Fly

### Step 1: Prepare Backend Environment

```bash
# Navigate to ragbot-api
cd /workspaces/hacks022/ragbot-api

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create .env file with secrets
cp .env.example .env
# Edit .env with actual credentials
```

### Step 2: Initialize Database & Qdrant

```bash
# Run migrations and create tables
python -c "from db import init_db; init_db()"

# Ingest documents (from project root)
cd /workspaces/hacks022
python ragbot-ingest.py
```

### Step 3: Test Backend Locally

```bash
cd ragbot-api
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Visit: `http://localhost:8000/docs` for OpenAPI UI

### Step 4: Deploy Backend to Render

1. **Create Repository**
   ```bash
   git add .
   git commit -m "Add RAG chatbot system"
   git push origin main
   ```

2. **Connect to Render**
   - Go to [render.com](https://render.com)
   - Click "New +" → "Web Service"
   - Connect GitHub repository
   - Select `/ragbot-api` as root directory

3. **Configure**
   - **Build Command:** `pip install -r requirements.txt`
   - **Start Command:** `gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app`
   - **Environment Variables:** Add from `.env.example`

4. **Deploy**
   - Click Deploy
   - Monitor build logs
   - Get service URL

### Step 5: Deploy Frontend to Vercel

1. **Prepare Docusaurus**
   ```bash
   cd /workspaces/hacks022
   npm install
   ```

2. **Update API URL**
   - In `src/pages/ragbot.jsx`: Update `apiUrl` to backend service URL
   - Or use environment variable: `REACT_APP_API_URL`

3. **Connect to Vercel**
   - Go to [vercel.com](https://vercel.com)
   - Click "New Project"
   - Import GitHub repository
   - Root directory: `/` (not needed if at project root)
   - Framework: Docusaurus
   - Environment variables:
     - `REACT_APP_API_URL=https://your-render-service-url`

4. **Deploy**
   - Click Deploy
   - Wait for build completion
   - Get deployment URL

### Step 6: Post-Deployment Checklist

```bash
# 1. Verify API health
curl https://your-render-service-url/api/health

# 2. Test chat endpoint
curl -X POST https://your-render-service-url/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"test","mode":"fullbook"}'

# 3. Check Docusaurus at Vercel URL
# 4. Test RAG chatbot at /ragbot
# 5. Monitor logs for errors
```

---

## Rate Limiting Strategy

### Configuration

```python
# In config.py
rate_limit_calls: int = 100
rate_limit_period: int = 3600  # 1 hour
```

### Implementation with SlowAPI

```python
from slowapi import Limiter
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

limiter = Limiter(key_func=get_remote_address)

@app.exception_handler(RateLimitExceeded)
async def ratelimit_handler(request, exc):
    return JSONResponse(
        status_code=429,
        content={"detail": "Rate limit exceeded. Max 100 requests per hour."}
    )

# In routers/chat.py
@router.post("/api/chat")
@limiter.limit("100/hour")
async def chat(request: ChatRequest, ...):
    pass
```

### Rate Limit Tiers

| Tier | Calls/Hour | Concurrent | Use Case |
|------|-----------|-----------|----------|
| Free | 100 | 5 | Development |
| Standard | 500 | 10 | Testing |
| Premium | 5000 | 50 | Production |

### Monitoring

```python
# Log rate limit hits
if rate_limited:
    logger.warning(f"Rate limit exceeded for {user_id}")
    
# Track in database
class RateLimitLog(Base):
    user_id = Column(String)
    timestamp = Column(DateTime)
    endpoint = Column(String)
```

---

## Maintenance Guide

### Monitoring

#### Health Checks

```bash
# API health
curl https://your-api-url/api/health

# Database connection
# Check Neon dashboard for connection metrics

# Qdrant status
curl https://your-qdrant-url/health -H "api-key: ..."

# OpenAI API status
# Monitor in OpenAI dashboard
```

#### Key Metrics

- **API Response Time:** Target < 3s
- **Qdrant Search Latency:** Target < 500ms
- **Embedding Generation:** Depends on batch size
- **Database Query Time:** Target < 100ms

### Logging

```python
# Structured logging
import logging
import json

logger = logging.getLogger(__name__)

# Log format
logger.info(json.dumps({
    "event": "chat_request",
    "user_id": user_id,
    "mode": mode,
    "duration_ms": elapsed_ms,
    "tokens_used": tokens
}))
```

### Backup & Recovery

```bash
# PostgreSQL backup (Neon handles automatically)
# Access backups in Neon dashboard

# Qdrant snapshot
curl -X POST https://your-qdrant-url/snapshots \
  -H "api-key: ..."

# Restore from backup
# Contact Neon support for recovery
```

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Slow chat responses | Check Qdrant search time, increase top_k_results |
| Embedding failures | Verify OpenAI API key, check rate limits |
| Database connection drops | Neon auto-reconnects; check connection pooling |
| Vectors not stored | Verify Qdrant collection exists, check payload size |
| UI not loading | Check CORS configuration, verify API URL |

### Update Procedure

1. **Test locally**
   ```bash
   git checkout -b feature/update
   # Make changes
   # Test thoroughly
   ```

2. **Deploy to staging**
   ```bash
   git push origin feature/update
   # Create pull request
   # Deploy to staging environment
   ```

3. **Production deployment**
   ```bash
   git merge main
   # Automated deployment triggers
   # Monitor logs for errors
   ```

### Regular Maintenance Tasks

- **Weekly:** Review logs for errors
- **Monthly:** Check API usage and costs
- **Monthly:** Update dependencies
- **Quarterly:** Review security settings
- **Quarterly:** Archive old chat logs

### Cost Optimization

```python
# Monitor token usage
# Optimize chunk size for better search
# Batch embedding generation
# Use smaller model for simple queries

# Example: fallback to GPT-3.5 for simple questions
if query_complexity < 0.5:
    model = "gpt-3.5-turbo"
else:
    model = "gpt-4-turbo"
```

### Scaling Considerations

As usage grows:

1. **Add caching layer (Redis)**
   ```python
   from redis import Redis
   redis = Redis(host='localhost', port=6379)
   
   # Cache embeddings
   cached = redis.get(f"embedding:{query}")
   ```

2. **Increase vector batch size**
   ```python
   # Adjust in config
   batch_size = 50  # increased from 25
   ```

3. **Add database read replicas (Neon)**
   - Create read-only replica for analytics

4. **Implement request queuing**
   ```python
   from celery import Celery
   app = Celery('ragbot')
   
   @app.task
   def process_chat(query, mode):
       # Long-running chat processing
       pass
   ```

---

## Appendix

### Environment Variables

```bash
# OpenAI
OPENAI_API_KEY=<REDACTED>
OPENAI_MODEL=gpt-4-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
QDRANT_COLLECTION_NAME=robotics_docs

# Database
DATABASE_URL=postgresql://...

# Frontend
FRONTEND_URL=https://...
REACT_APP_API_URL=https://...

# API Configuration
CHUNK_SIZE=500
TOP_K_RESULTS=5
RATE_LIMIT_CALLS=100
RATE_LIMIT_PERIOD=3600
```

### Dependencies

**Backend:**
- FastAPI 0.104.1
- Uvicorn 0.24.0
- SQLAlchemy 2.0.23
- Qdrant Client 2.7.0
- OpenAI 1.3.5

**Frontend:**
- React 18+
- Docusaurus 3+

### References

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Neon Documentation](https://neon.tech/docs/)
- [Docusaurus Documentation](https://docusaurus.io/)

---

**Last Updated:** December 2024  
**Maintained By:** Engineering Team
