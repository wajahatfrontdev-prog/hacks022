# 🤖 Physical AI Humanoid Robotics Book + RAG Chatbot

This is a comprehensive **Docusaurus-based documentation site** for Physical AI Humanoid Robotics with an **integrated RAG (Retrieval-Augmented Generation) Chatbot**.

## Features

- 📚 **Complete Robotics Curriculum:** Modules on ROS2, Digital Twins, NVIDIA Isaac, and Humanoid Robotics
- 🤖 **RAG Chatbot:** AI-powered assistant that answers questions based on your book content
- 🎨 **Modern UI:** Built with React and Docusaurus
- 🚀 **Production-Ready:** Docker containerization + Vercel/Hugging Face deployment

## Quick Links

- **Live Book:** https://your-deployment.vercel.app
- **Chat with AI:** https://your-deployment.vercel.app/ragbot
- **Deployment Guide:** See [DEPLOYMENT-GUIDE.md](./DEPLOYMENT-GUIDE.md)
- **Status:** See [PRODUCTION-READY.md](./PRODUCTION-READY.md)

---

## Local Development

### Prerequisites
- Node.js 16+
- Python 3.12
- Git

### 1. Frontend Setup
```bash
npm install
npm run start  # Dev server on http://localhost:3000
```

### 2. Backend Setup
```bash
cd ragbot-api
cp .env.example .env
# Fill .env with: GROQ_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL

pip install -r requirements.txt
python -m uvicorn main:app --reload --port 8000
```

### 3. Access
- **Book:** http://localhost:3000
- **Chat:** http://localhost:3000/ragbot
- **API:** http://localhost:8000/api

---

## Production Deployment

### 🌐 Frontend → Vercel
```bash
npm run build
# Deploy build/ folder to Vercel
```
See [DEPLOYMENT-GUIDE.md - Part 1](./DEPLOYMENT-GUIDE.md#part-1-frontend-deployment-to-vercel-)

### 🔧 Backend → Hugging Face Spaces
```bash
docker build -t ragbot-backend:latest .
# Push to HF Spaces via Git
```
See [DEPLOYMENT-GUIDE.md - Part 2](./DEPLOYMENT-GUIDE.md#part-2-backend-deployment-to-hugging-face-spaces)

---

## Key Documentation

| Document | Purpose |
|----------|---------|
| [DEPLOYMENT-GUIDE.md](./DEPLOYMENT-GUIDE.md) | Step-by-step deployment instructions |
| [DEPLOYMENT-CHECKLIST.md](./DEPLOYMENT-CHECKLIST.md) | Pre-launch verification checklist |
| [PRODUCTION-READY.md](./PRODUCTION-READY.md) | Project overview & technology stack |
| [RAGBOT-SETUP.md](./RAGBOT-SETUP.md) | Backend setup guide |

---

## Technology Stack

| Component | Technology |
|-----------|-----------|
| Frontend | React + Docusaurus |
| Backend | FastAPI + Python 3.12 |
| LLM | Groq API (llama-3.3-70b-versatile) |
| Embeddings | sentence-transformers |
| Vector DB | Qdrant Cloud |
| Container | Docker |

---

## Building

```bash
# Frontend
npm run build  # Output: build/

# Backend (Docker)
docker build -t ragbot-backend:latest .
```

---

## Environment Variables

Create `ragbot-api/.env`:
```env
GROQ_API_KEY=<REDACTED>
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
DATABASE_URL=sqlite:///./chat.db
```

---

## Status

✅ **Production Ready**
- Frontend: Built & optimized
- Backend: Docker containerized
- Documentation: Complete
- Deployment: Ready (see DEPLOYMENT-GUIDE.md)

---

See [DEPLOYMENT-GUIDE.md](./DEPLOYMENT-GUIDE.md) for detailed deployment instructions.
