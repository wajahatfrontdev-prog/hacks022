# RAG Chatbot - Production Deployment Guide

**کہاں deploy کریں:**
- **Frontend (Docusaurus + React)**: Vercel یا Netlify
- **Backend (FastAPI + Qdrant)**: Hugging Face Spaces یا Railway

---

## Part 1: Frontend Deployment to Vercel ✅

### Prerequisites:
- Vercel account (free at vercel.com)
- GitHub repo with code pushed

### Steps:

1. **Go to Vercel Dashboard:**
   ```
   https://vercel.com/dashboard
   ```

2. **Click "New Project" → Connect Git:**
   - Select your GitHub repo: `wajahatfrontdev-prog/hacks022`
   - Vercel auto-detects Docusaurus

3. **Configure Build Settings:**
   - **Build Command:** `npm run build`
   - **Output Directory:** `build`
   - **Environment Variables:** (Leave empty for frontend)

4. **Deploy:**
   - Click "Deploy"
   - Frontend will be live at: `https://hacks022.vercel.app` (or custom domain)

5. **Update Backend URL (if needed):**
   - If backend is deployed separately, update `src/pages/ragbot.jsx`:
     ```javascript
     // Change localhost API URL to production backend
     const API_BASE = "https://your-backend-domain.com/api"
     ```

---

## Part 2: Backend Deployment to Hugging Face Spaces

### Prerequisites:
- Hugging Face account (free at huggingface.co)
- Docker image built locally
- API Keys ready:
  - `GROQ_API_KEY` (from console.groq.com)
  - `QDRANT_URL` (Qdrant Cloud)
  - `QDRANT_API_KEY` (Qdrant Cloud)

### Option A: Using Docker (Recommended)

1. **Build Docker Image:**
   ```bash
   cd /workspaces/hacks022
   docker build -t ragbot-backend:latest .
   ```

2. **Test Locally:**
   ```bash
   docker run -p 8000:8000 \
     -e GROQ_API_KEY=<REDACTED> \
     -e QDRANT_URL=<REDACTED> \
     -e QDRANT_API_KEY=<REDACTED> \
     ragbot-backend:latest
   
   # Test: curl http://localhost:8000/api/health
   ```

3. **Push to Hugging Face Spaces:**

   a. **Create Space on HF:**
   ```
   https://huggingface.co/new-space
   - Space name: "ragbot-backend"
   - License: Open RAIL
   - Space SDK: Docker
   ```

   b. **Clone Space & Push Docker:**
   ```bash
   git clone https://huggingface.co/spaces/YOUR_HF_USERNAME/ragbot-backend
   cd ragbot-backend
   
   # Copy Dockerfile and ragbot-api
   cp /workspaces/hacks022/Dockerfile .
   cp -r /workspaces/hacks022/ragbot-api ./ragbot-api
   
   # Create .gitignore
   echo "ragbot-api/venv/" >> .gitignore
   echo ".env" >> .gitignore
   
   git add .
   git commit -m "Deploy RAG chatbot backend"
   git push
   ```

4. **Set Environment Secrets:**
   - Go to Space Settings → Secrets
   - Add:
     - `GROQ_API_KEY`: Your Groq API key
     - `QDRANT_URL`: Your Qdrant Cloud URL
     - `QDRANT_API_KEY`: Your Qdrant API key
     - `DATABASE_URL`: `sqlite:///./chat.db`

5. **Backend will be live at:**
   ```
   https://YOUR_HF_USERNAME-ragbot-backend.hf.space/api
   ```

---

### Option B: Using Railway (Alternative)

1. **Sign up at railway.app**

2. **Create New Project → Deploy from GitHub**

3. **Build Variables:**
   - Set environment variables in Railway dashboard
   - Add all `.env` variables (GROQ_API_KEY, QDRANT_*, etc.)

4. **Add Dockerfile:**
   - Railway auto-detects and builds Docker image

5. **Backend URL:** `https://your-project.railway.app/api`

---

## Part 3: Connect Frontend & Backend

Once both are deployed:

1. **Update Frontend API URL:**
   - Edit `src/pages/ragbot.jsx`
   - Change:
     ```javascript
     const API_BASE = "https://your-backend-domain.hf.space/api"
     ```
   - Or in `ragbot-ui/ChatInterface.jsx`

2. **Redeploy Frontend:**
   ```bash
   git push
   # Vercel auto-deploys on push
   ```

3. **Test:**
   - Go to `https://hacks022.vercel.app/ragbot`
   - Type "hello" and verify response

---

## Part 4: Environment Variables Checklist

### Backend Needs (.env or Secrets):

```env
# Groq (LLM Provider)
GROQ_API_KEY=<REDACTED>

# Qdrant Vector DB
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>

# Database (SQLite - auto-created)
DATABASE_URL=sqlite:///./chat.db

# Frontend (for CORS)
FRONTEND_URL=https://hacks022.vercel.app
```

### Frontend Needs (.env or hardcoded):

```env
VITE_API_URL=https://your-backend-domain.hf.space/api
```

---

## Part 5: Troubleshooting

### "CORS error" when calling backend:
- Backend needs CORS enabled ✅ (already in `main.py`)
- Check `FRONTEND_URL` in backend env matches frontend URL

### "Invalid API key" for Groq:
- Verify `GROQ_API_KEY` is correct from console.groq.com
- Ensure it's set in Secrets/Environment

### "Connection refused" to Qdrant:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Test locally: `curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/health`

### "Build fails on Vercel":
- Check `npm run build` works locally: `npm install && npm run build`
- Ensure `build/` directory is created

---

## Part 6: Monitor & Debug

### Backend Logs:
```bash
# Hugging Face Spaces → Logs
# Railway → Deployments → Logs
# Docker local: docker logs <container_id>
```

### Frontend Errors:
```bash
# Vercel Dashboard → Deployments → Logs
# Browser DevTools → Network & Console
```

### API Health Check:
```bash
curl https://your-backend.hf.space/api/health
# Should return: {"status": "ok"}
```

---

## Summary

| Component | Platform | Live URL | Build |
|-----------|----------|----------|-------|
| Frontend | Vercel | hacks022.vercel.app | `npm run build` ✅ |
| Backend | HF Spaces | your-space.hf.space | Docker ✅ |
| DB Vector | Qdrant Cloud | (Cloud URL) | Pre-ingested |
| LLM | Groq API | (API calls) | Configured |

**Next:** Monitor both deployments and handle any CORS/auth issues.
