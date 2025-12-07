# RAG Chatbot Quick Reference Card

## 🚀 Start Here

```bash
# 1. Run setup
./setup-ragbot.sh  # or setup-ragbot.bat on Windows

# 2. Update credentials
# Edit: ragbot-api/.env
# Add: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL

# 3. Ingest documents
python ragbot-ingest.py

# 4. Start backend (Terminal 1)
cd ragbot-api
source venv/bin/activate
python -m uvicorn main:app --reload

# 5. Start frontend (Terminal 2)
npm start

# 6. Access
# Frontend: http://localhost:3000
# RAG Chat: http://localhost:3000/ragbot
# API Docs: http://localhost:8000/docs
```

---

## 📁 Project Structure

```
/ragbot-api/              FastAPI backend
├── main.py              Entry point
├── config.py            Settings
├── db.py                Database
├── qdrant_client.py     Vector store
├── embeddings.py        Embeddings
└── routers/
    ├── chat.py          Chat logic
    └── ingest.py        Document ingestion

/ragbot-ui/              React components
├── ChatInterface.jsx    Main chat
├── ModeSelector.jsx     Mode toggle
├── SelectedTextBox.jsx  Text selection
└── SourceDisplay.jsx    Source attribution

/docs/
└── RAGBOT-SPECIFICATIONS.md    Full docs

RAGBOT-*.md              Setup guides
ragbot-ingest.py         Ingestion script
```

---

## 🎯 Key Files

| File | Purpose | Lines |
|------|---------|-------|
| main.py | FastAPI app | 85 |
| config.py | Configuration | 87 |
| db.py | Database models | 96 |
| qdrant_client.py | Vector ops | 130 |
| embeddings.py | OpenAI embeddings | 71 |
| chat.py | Chat endpoints | 215 |
| ingest.py | Document ingestion | 155 |
| ChatInterface.jsx | UI component | 140 |
| ChatInterface.module.css | Styles | 400+ |

---

## 📚 API Endpoints

```bash
# Chat
POST /api/chat
GET /api/chat/history/{session_id}
POST /api/chat/rate

# Ingest
POST /api/ingest/docs
GET /api/ingest/status

# System
GET /api/health
GET /api/config
GET /docs
```

---

## ⚙️ Configuration

### Environment Variables (.env)
```bash
OPENAI_API_KEY=<REDACTED>
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
DATABASE_URL=postgresql://...
FRONTEND_URL=https://...
```

### Backend Settings (config.py)
```python
chunk_size = 500              # Chunk size
chunk_overlap = 100           # Overlap
top_k_results = 5             # Search results
openai_model = "gpt-4-turbo"  # LLM model
```

### Frontend URL (ragbot.jsx)
```javascript
<ChatInterface apiUrl={process.env.REACT_APP_API_URL || '/api'} />
```

---

## 🔄 Chat Modes

### FULLBOOK
- Search entire book via Qdrant
- ~2-3 seconds
- Uses GPT-4 with context
- Shows sources

### SELECTED
- Uses selected text only
- ~1-2 seconds
- No Qdrant search
- No sources shown

---

## 🗄️ Database Tables

```sql
-- Sessions
user_sessions(id, user_id, created_at, ...)

-- Messages
chat_messages(id, session_id, user_query, bot_response, ...)

-- Logs
ingestion_logs(id, doc_path, chunks_created, ...)
```

---

## 📊 Chat Request/Response

### Request
```json
{
  "query": "What is ROS2?",
  "mode": "fullbook",
  "session_id": "session_123",
  "user_id": "user_123"
}
```

### Response
```json
{
  "response": "ROS2 is...",
  "session_id": "session_123",
  "sources": [...],
  "tokens_used": 487,
  "mode": "fullbook"
}
```

---

## 🐳 Docker

```bash
# Build
docker build -t ragbot-api .

# Run
docker run -p 8000:8000 --env-file .env ragbot-api

# With Compose
docker-compose up
```

---

## 🚢 Deploy to Render

1. Push to GitHub
2. Render → New Web Service
3. Select repo
4. Root: `ragbot-api`
5. Build: `pip install -r requirements.txt`
6. Start: `gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app`
7. Add environment variables
8. Deploy

---

## 🚢 Deploy to Vercel

1. Vercel → New Project
2. Import repo
3. Framework: Docusaurus
4. Environment: `REACT_APP_API_URL=<render-url>`
5. Deploy

---

## 🧪 Testing

```bash
# Unit tests
cd ragbot-api
pytest test_api.py -v

# Health check
curl http://localhost:8000/api/health

# Test chat
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"test","mode":"fullbook","user_id":"test"}'

# Swagger UI
http://localhost:8000/docs
```

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Qdrant connection failed | Check URL, API key in .env |
| OpenAI error | Verify API key, check credits |
| Database error | Check DATABASE_URL |
| No documents | Run `python ragbot-ingest.py` |
| API 404 | Check backend is running on port 8000 |
| UI not loading | Check frontend is on port 3000 |
| Slow responses | Check Qdrant, reduce top_k_results |

---

## 📚 Documentation Files

1. **RAGBOT-README.md** - Overview & quick start
2. **RAGBOT-SETUP.md** - Detailed setup guide
3. **RAGBOT-INTEGRATION.md** - Docusaurus integration
4. **docs/RAGBOT-SPECIFICATIONS.md** - Full technical specs
5. **IMPLEMENTATION-SUMMARY.md** - What was built

---

## 📞 Support

- Full specs: `docs/RAGBOT-SPECIFICATIONS.md`
- Setup issues: `RAGBOT-SETUP.md`
- Integration help: `RAGBOT-INTEGRATION.md`
- API docs: http://localhost:8000/docs

---

## ✅ Checklist

- [ ] Run setup script
- [ ] Update .env credentials
- [ ] Run ingest script
- [ ] Start backend
- [ ] Start frontend
- [ ] Test at /ragbot
- [ ] Deploy backend to Render
- [ ] Deploy frontend to Vercel
- [ ] Set environment variables
- [ ] Test production URLs

---

## 🎯 Key Decisions

| Decision | Value |
|----------|-------|
| Backend Framework | FastAPI |
| Frontend | React + Docusaurus |
| LLM | OpenAI GPT-4 Turbo |
| Vector Store | Qdrant Cloud |
| Database | Neon PostgreSQL |
| Chunk Size | 500 tokens |
| Top K Results | 5 |
| Rate Limit | 100/hour |

---

## 📈 Performance Targets

| Metric | Target |
|--------|--------|
| Chat response | < 3 seconds |
| Vector search | < 500ms |
| DB query | < 100ms |
| Embedding gen | < 1 second |

---

## 🔐 Security Notes

- ✅ Secrets in .env (never in code)
- ✅ CORS configured
- ✅ Input validation
- ✅ Rate limiting ready
- ✅ Database pooling
- ✅ Error handling (no stack traces)

---

**Version:** 1.0.0  
**Status:** Production Ready  
**Last Updated:** December 2024
