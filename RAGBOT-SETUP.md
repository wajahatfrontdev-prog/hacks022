# RAG Chatbot Setup & Deployment Guide

## Quick Start

### Local Development

```bash
# 1. Install backend dependencies
cd ragbot-api
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 2. Setup environment
cp .env.example .env
# Edit .env with your credentials

# 3. Initialize database and ingest documents
python -c "from db import init_db; init_db()"
cd ..
python ragbot-ingest.py

# 4. Start the backend
cd ragbot-api
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000

# 5. In another terminal, start Docusaurus frontend
npm install
npm start
```

Backend will be available at: `http://localhost:8000`
Frontend will be available at: `http://localhost:3000`
RAG Chatbot at: `http://localhost:3000/ragbot`

---

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────┐
│        Frontend (Docusaurus + React)        │
│  - Chat Interface with mode selection       │
│  - Source display and attribution           │
│  - Conversation history                     │
└──────────────┬──────────────────────────────┘
               │
               │ HTTP/REST
               ▼
┌─────────────────────────────────────────────┐
│    Backend (FastAPI - ragbot-api/)          │
│  - /api/chat (fullbook | selected mode)     │
│  - /api/ingest (document processing)        │
│  - Session management                       │
└──────────┬──────────────┬──────────────┬────┘
           │              │              │
           ▼              ▼              ▼
    ┌───────────────┐ ┌────────────┐ ┌──────────────┐
    │  OpenAI API   │ │ Qdrant     │ │ Neon         │
    │  - Embeddings │ │ - Vector   │ │ - Sessions   │
    │  - Chat GPT-4 │ │   Storage  │ │ - Messages   │
    └───────────────┘ └────────────┘ └──────────────┘
```

### Mode Comparison

| Feature | FULLBOOK | SELECTED |
|---------|----------|----------|
| Context | Entire book (vector search) | User-selected text only |
| Qdrant Usage | Yes (search top-5) | No |
| Speed | ~2-3 seconds | ~1-2 seconds |
| Accuracy | Better for book-wide context | Better for specific sections |
| Use Case | General questions | Focused questions |

---

## Environment Variables

Create `.env` file in `ragbot-api/` directory:

```bash
# OpenAI Configuration
OPENAI_API_KEY=<REDACTED>
OPENAI_MODEL=gpt-4-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Configuration  
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
QDRANT_COLLECTION_NAME=robotics_docs

# Database Configuration
DATABASE_URL=postgresql://user:password@host:port/database

# Frontend Configuration
FRONTEND_URL=http://localhost:3000

# API Configuration
CHUNK_SIZE=500
CHUNK_OVERLAP=100
TOP_K_RESULTS=5
RATE_LIMIT_CALLS=100
RATE_LIMIT_PERIOD=3600
```

---

## API Endpoints

### Chat Endpoint

**POST** `/api/chat`

Request:
```json
{
  "query": "What is ROS2?",
  "mode": "fullbook",
  "selected_text": null,
  "session_id": "session_123",
  "user_id": "user_123"
}
```

Response:
```json
{
  "response": "ROS2 is the Robot Operating System...",
  "session_id": "session_123",
  "sources": [
    {
      "filename": "module01/02-ros2-nodes.md",
      "module": "module01-ros2",
      "section": "ROS2 Fundamentals",
      "score": 0.92,
      "content": "ROS2 is..."
    }
  ],
  "tokens_used": 487,
  "mode": "fullbook"
}
```

### Available Endpoints

- `POST /api/chat` - Send chat message
- `GET /api/chat/history/{session_id}` - Get conversation history
- `POST /api/chat/rate` - Rate a response (1-5 stars)
- `POST /api/ingest/docs` - Ingest/re-ingest documents
- `GET /api/ingest/status` - Check ingestion status
- `GET /api/health` - Health check
- `GET /` - API info
- `/docs` - OpenAPI documentation

---

## Document Ingestion Process

### How It Works

1. **Scan & Read**: Find all `.md` and `.mdx` files in `/docs`
2. **Extract Sections**: Split by headers (H1, H2, H3)
3. **Chunk**: Create 500-token chunks with 100-token overlap
4. **Extract Metadata**: 
   - Module name from folder (e.g., module01-ros2)
   - Filename
   - Section header
   - Full path
5. **Generate Embeddings**: Use OpenAI's text-embedding-3-small
6. **Store Vectors**: Upload to Qdrant with metadata
7. **Log**: Record in database for audit trail

### Running Ingestion

```bash
# From project root
python ragbot-ingest.py

# Output:
# ============================================================
# RAG Chatbot Document Ingestion Script
# ============================================================
# 
# 🔄 Initializing database...
# ✓ Database initialized
# 
# 🔄 Setting up Qdrant collection...
# ✓ Qdrant collection ready
# 
# 🔄 Processing markdown files from /workspaces/hacks022/docs...
# ✓ Processed module01-ros2/02-ros2-nodes-topics.md (3 sections)
# ...
# ✓ Found 247 chunks from 15 files
# 
# 🔄 Generating embeddings (this may take a minute)...
# ✓ Generated 247 embeddings
# 
# 🔄 Storing vectors in Qdrant...
# ✓ Successfully stored 247 vectors
```

---

## Database Schema

### user_sessions
```sql
id (PK)
user_id
created_at
updated_at
metadata (JSON)
```

### chat_messages
```sql
id (PK)
session_id (FK)
user_query
bot_response
mode (fullbook|selected)
selected_text (optional)
source_sections (JSON)
tokens_used
created_at
user_rating (1-5)
```

### ingestion_logs
```sql
id (PK)
doc_path
chunks_created
vectors_stored
status (success|failed)
error_message
created_at
```

---

## Deployment

### Backend (Render.com)

1. Push code to GitHub
2. Go to render.com → New Web Service
3. Connect GitHub repo
4. Set root directory: `ragbot-api`
5. Build: `pip install -r requirements.txt`
6. Start: `gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app`
7. Add environment variables
8. Deploy

### Frontend (Vercel)

1. Go to vercel.com → New Project
2. Import GitHub repo
3. Framework: Docusaurus
4. Set environment variable: `REACT_APP_API_URL=<render-url>`
5. Deploy

### Verify Deployment

```bash
# Check backend health
curl https://your-render-url/api/health

# Test chat
curl -X POST https://your-render-url/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"test","mode":"fullbook"}'

# Visit frontend
https://your-vercel-url
https://your-vercel-url/ragbot
```

---

## Troubleshooting

### Issue: "Cannot connect to Qdrant"
- Check QDRANT_URL and QDRANT_API_KEY in .env
- Verify Qdrant instance is running
- Check firewall/network access

### Issue: "OpenAI API error"
- Verify OPENAI_API_KEY is correct
- Check OpenAI account has available credits
- Monitor API usage in OpenAI dashboard

### Issue: "Database connection failed"
- Verify DATABASE_URL is correct
- Check Neon database is active
- Ensure network allows connection

### Issue: "No documents ingested"
- Run `python ragbot-ingest.py` again
- Check `/docs` folder has .md files
- Verify read permissions on docs folder

### Issue: "Chat responses are slow"
- Check Qdrant query latency
- Reduce top_k_results in config
- Check OpenAI API latency

---

## Security Considerations

- ✅ All secrets in .env (not in code)
- ✅ CORS configured for allowed origins
- ✅ Input validation with Pydantic
- ✅ Rate limiting support (configure in config.py)
- ✅ Database password protected
- ✅ API keys managed via environment variables

### Production Checklist

- [ ] Set strong DATABASE_URL password
- [ ] Use secure OPENAI_API_KEY
- [ ] Configure rate limiting
- [ ] Enable HTTPS everywhere
- [ ] Set up monitoring and alerts
- [ ] Configure database backups
- [ ] Review CORS settings
- [ ] Set up logging aggregation
- [ ] Enable API authentication
- [ ] Regular security audits

---

## Cost Estimation

### Monthly Costs

| Service | Plan | Est. Cost | Notes |
|---------|------|-----------|-------|
| OpenAI | Pay-as-you-go | $20-100 | Depends on usage |
| Qdrant | Starter | $40 | 50M vectors included |
| Neon | Free/Pro | $0-50 | Depends on connections |
| Render | Free/Paid | $0-7+ | Auto-scales |
| Vercel | Free/Pro | $0-20 | Serverless + CDN |

**Estimated Total:** $60-217/month depending on usage

### Cost Optimization

- Use smaller embedding model for pre-filtering
- Batch process large documents
- Implement caching for popular queries
- Monitor and optimize OpenAI API calls
- Use Neon's connection pooling

---

## Support & Resources

- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI API**: https://platform.openai.com/docs/
- **Neon Docs**: https://neon.tech/docs/
- **Docusaurus**: https://docusaurus.io/

---

**Version:** 1.0.0  
**Last Updated:** December 2024
