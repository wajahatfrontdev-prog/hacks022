# RAG Chatbot System for Physical AI Humanoid Robotics Book

Complete Retrieval-Augmented Generation (RAG) chatbot system integrated into the Docusaurus documentation site.

## 🎯 Quick Overview

**What it does:**
- Allows users to ask questions about the Physical AI Humanoid Robotics book
- Retrieves relevant book sections and generates AI-powered answers
- Supports two modes:
  - **FULLBOOK**: Uses AI to search entire book (RAG)
  - **SELECTED**: Answers questions about user-selected text only

**Tech Stack:**
- **Frontend**: React + Docusaurus
- **Backend**: FastAPI + Python
- **AI/LLM**: OpenAI GPT-4
- **Vector Search**: Qdrant Cloud
- **Database**: Neon PostgreSQL
- **Deployment**: Vercel (frontend) + Render/Fly (backend)

## 📁 Directory Structure

```
/workspaces/hacks022/
├── ragbot-api/                    # FastAPI Backend
│   ├── main.py                    # FastAPI application entry point
│   ├── config.py                  # Configuration & environment variables
│   ├── db.py                      # Database models & connections
│   ├── qdrant_client.py          # Qdrant vector store integration
│   ├── embeddings.py             # OpenAI embedding generation
│   ├── utils.py                  # Utility functions
│   ├── advanced_rag.py           # Advanced RAG techniques
│   ├── test_api.py               # Unit tests
│   ├── requirements.txt           # Python dependencies
│   ├── .env.example              # Environment template
│   └── routers/
│       ├── chat.py               # Chat endpoints
│       └── ingest.py             # Document ingestion endpoints
├── ragbot-ui/                    # React UI Components
│   ├── ChatInterface.jsx         # Main chat component
│   ├── ModeSelector.jsx          # Mode selection UI
│   ├── SelectedTextBox.jsx       # Text selection component
│   ├── SourceDisplay.jsx         # Source attribution
│   └── ChatInterface.module.css  # Component styles
├── src/pages/
│   └── ragbot.jsx                # Docusaurus page wrapper
├── docs/
│   └── RAGBOT-SPECIFICATIONS.md  # Full technical documentation
├── ragbot-ingest.py              # Document ingestion script
├── setup-ragbot.sh               # Setup script (Linux/Mac)
├── setup-ragbot.bat              # Setup script (Windows)
├── Dockerfile                    # Docker container definition
├── docker-compose.yml            # Docker Compose configuration
├── RAGBOT-SETUP.md              # Setup & deployment guide
├── RAGBOT-INTEGRATION.md        # Integration guide
└── README.md (this file)
```

## 🚀 Quick Start

### Prerequisites
- Python 3.9+
- Node.js 18+
- Git
- Accounts: OpenAI, Qdrant Cloud, Neon PostgreSQL

### 1. Clone & Setup

```bash
cd /workspaces/hacks022

# Linux/Mac
chmod +x setup-ragbot.sh
./setup-ragbot.sh

# Windows
setup-ragbot.bat
```

### 2. Configure Credentials

Edit `ragbot-api/.env`:
```bash
OPENAI_API_KEY=<REDACTED>
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
DATABASE_URL=postgresql://user:pass@host/db
```

### 3. Start Development

**Terminal 1 - Backend:**
```bash
cd ragbot-api
source venv/bin/activate  # Windows: venv\Scripts\activate
python -m uvicorn main:app --reload
```

**Terminal 2 - Frontend:**
```bash
npm start
```

### 4. Test It Out

- Frontend: `http://localhost:3000`
- RAG Chat: `http://localhost:3000/ragbot`
- API Docs: `http://localhost:8000/docs`

### 5. Ingest Documents (when ready)

```bash
# From project root
python ragbot-ingest.py
```

## 🏗️ System Architecture

```
User Query
    ↓
[Docusaurus Frontend - React Component]
    ↓
POST /api/chat
    ↓
[FastAPI Backend]
    ├─→ Mode Check (fullbook/selected)
    ├─→ If FULLBOOK:
    │   ├─ Generate query embedding (OpenAI)
    │   ├─ Search Qdrant vectors (top-5)
    │   ├─ Combine context chunks
    │   └─ Call GPT-4 with context
    └─→ If SELECTED:
        └─ Call GPT-4 with selected text only
    ↓
[Store in Neon Database]
    ├─ Chat message
    ├─ Sources used
    └─ Tokens consumed
    ↓
JSON Response with Answer + Sources
    ↓
[Display in Chat UI]
```

## 📚 API Endpoints

### Chat Operations

```bash
# Send message (fullbook mode)
POST /api/chat
{
  "query": "What is ROS2?",
  "mode": "fullbook",
  "session_id": "session_123",
  "user_id": "user_123"
}

# Send message (selected text mode)
POST /api/chat
{
  "query": "What is this?",
  "mode": "selected",
  "selected_text": "Text from user selection...",
  "session_id": "session_123"
}

# Get chat history
GET /api/chat/history/{session_id}

# Rate a response
POST /api/chat/rate
{
  "message_id": "msg_123",
  "rating": 5
}
```

### Document Operations

```bash
# Ingest documents
POST /api/ingest/docs
{"force_reingest": false}

# Check ingestion status
GET /api/ingest/status

# Reset collection
POST /api/ingest/reset
```

### System Operations

```bash
# Health check
GET /api/health

# Get API config
GET /api/config
```

See `docs/RAGBOT-SPECIFICATIONS.md` for full API documentation.

## 🎮 Chat Modes

### FULLBOOK RAG Mode
- **Use case**: General questions about the book
- **Speed**: 2-3 seconds
- **Method**: Searches entire book, retrieves top-5 relevant sections, generates answer with AI
- **Accuracy**: High for finding relevant information across the book
- **Qdrant**: Yes (vector search enabled)

Example:
```
User: "What is ROS2?"
System: [Searches 247 vectors] → [Finds relevant sections] → [GPT-4 generates answer]
Result: "ROS2 is the Robot Operating System version 2..."
Sources: module01-ros2/02-ros2-nodes-topics.md, etc.
```

### SELECTED Mode
- **Use case**: Questions about specific text user selected
- **Speed**: 1-2 seconds
- **Method**: Uses ONLY selected text, no vector search
- **Accuracy**: Very high for targeted questions
- **Qdrant**: No (skipped)

Example:
```
User: Selects text → Asks "What does this mean?"
System: [Uses selected text only] → [GPT-4 explains]
Result: Explanation based strictly on selection
Sources: None (explicit constraint)
```

## 🔧 Configuration

### Backend Settings (`ragbot-api/config.py`)

```python
# Model
openai_model = "gpt-4-turbo"
openai_embedding_model = "text-embedding-3-small"

# RAG
chunk_size = 500          # Token size per chunk
chunk_overlap = 100       # Overlap between chunks
top_k_results = 5         # Results from vector search

# Database
database_url = "postgresql://..."

# Vector Store
qdrant_url = "https://..."
qdrant_collection_name = "robotics_docs"

# Rate Limiting
rate_limit_calls = 100
rate_limit_period = 3600
```

### Frontend Settings (`src/pages/ragbot.jsx`)

```javascript
<ChatInterface apiUrl={process.env.REACT_APP_API_URL || '/api'} />
```

## 📊 Database Schema

### user_sessions
Stores user session information for conversation continuity

### chat_messages
Logs all chat interactions with metadata (mode, sources, tokens, ratings)

### ingestion_logs
Tracks document ingestion operations for auditing

See `docs/RAGBOT-SPECIFICATIONS.md` for full schema details.

## 🚢 Deployment

### Deploy Backend to Render

1. Push to GitHub
2. Go to render.com → New Web Service
3. Select GitHub repository
4. Set root directory: `ragbot-api`
5. Build: `pip install -r requirements.txt`
6. Start: `gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app`
7. Add environment variables from `.env`
8. Deploy!

### Deploy Frontend to Vercel

1. Go to vercel.com → New Project
2. Import GitHub repository
3. Framework: Docusaurus
4. Environment: `REACT_APP_API_URL=https://your-render-url`
5. Deploy!

### Using Docker

```bash
# Build image
docker build -t ragbot-api .

# Run container
docker run -p 8000:8000 --env-file .env ragbot-api

# Or use docker-compose
docker-compose up
```

## 🔐 Security

### ✅ Implemented
- Environment variables for secrets
- CORS configuration
- Input validation (Pydantic)
- Database connection pooling
- Rate limiting support

### 🛡️ Production Recommendations
- Enable API key authentication
- Use HTTPS everywhere
- Set up monitoring & alerts
- Regular backups (Neon handles)
- Log aggregation
- IP whitelisting

See `docs/RAGBOT-SPECIFICATIONS.md` for security checklist.

## 📈 Monitoring & Logs

### Health Check
```bash
curl https://your-api-url/api/health
```

### View Logs
- Backend: `ragbot.log` file (configured in logging)
- Render: Dashboard logs
- Vercel: Deployment logs
- Neon: Database metrics dashboard

### Key Metrics to Monitor
- API response time (target: < 3s)
- Qdrant search latency (target: < 500ms)
- Database connection pool usage
- OpenAI token consumption
- Error rates

## 🧪 Testing

### Run Unit Tests

```bash
cd ragbot-api
pip install pytest
pytest test_api.py -v
```

### Manual API Testing

```bash
# Test with curl
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "mode": "fullbook",
    "user_id": "test"
  }'

# Use Swagger UI
# Visit: http://localhost:8000/docs
```

## 🐛 Troubleshooting

### "Cannot connect to Qdrant"
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Verify Qdrant instance is active
- Check network firewall

### "OpenAI API error"
- Verify `OPENAI_API_KEY` is correct
- Check API usage in OpenAI dashboard
- Ensure account has available credits

### "Database connection failed"
- Verify `DATABASE_URL` is correct
- Check Neon database status
- Ensure network allows connection

### "No documents showing in search"
- Run `python ragbot-ingest.py`
- Check `/docs` folder for markdown files
- Verify read permissions

### "Chat responses are slow"
- Check Qdrant query latency
- Reduce `top_k_results` in config
- Check OpenAI API status

See `docs/RAGBOT-SPECIFICATIONS.md` for full troubleshooting guide.

## 📖 Documentation

- **`RAGBOT-SETUP.md`** - Detailed setup & deployment guide
- **`RAGBOT-INTEGRATION.md`** - Integration into Docusaurus
- **`docs/RAGBOT-SPECIFICATIONS.md`** - Complete technical specifications
  - System architecture
  - API documentation
  - Database schema
  - Security guidelines
  - Maintenance procedures

## 💰 Cost Estimation

| Service | Plan | Est. Monthly |
|---------|------|-------------|
| OpenAI | Pay-as-you-go | $20-100 |
| Qdrant | Starter | $40 |
| Neon | Free/Pro | $0-50 |
| Render | Free/Paid | $0-7+ |
| Vercel | Free/Pro | $0-20 |
| **Total** | | **$60-217** |

## 🛣️ Roadmap

- [x] Backend API (FastAPI)
- [x] Frontend Components (React)
- [x] Docusaurus Integration
- [x] Document Ingestion Pipeline
- [x] Full Documentation
- [ ] Deploy to Render
- [ ] Deploy to Vercel
- [ ] Production monitoring
- [ ] Advanced features (caching, multi-turn, etc.)

## 🤝 Contributing

1. Create a feature branch
2. Make changes
3. Test locally
4. Submit pull request

## 📝 License

This project is part of the Physical AI Humanoid Robotics Book.

## 📞 Support

For issues, questions, or suggestions:
1. Check `docs/RAGBOT-SPECIFICATIONS.md`
2. Review API documentation at `/docs` endpoint
3. Check troubleshooting section
4. Review application logs

---

**Version:** 1.0.0  
**Last Updated:** December 2024  
**Status:** Production Ready
