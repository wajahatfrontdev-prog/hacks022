# RAG Chatbot Deployment Checklist

## Pre-Deployment Setup

### 1. Environment Preparation

- [ ] Clone repository locally
- [ ] Navigate to `/workspaces/hacks022`
- [ ] Ensure Python 3.9+ installed: `python3 --version`
- [ ] Ensure Node.js 18+ installed: `node --version`
- [ ] Ensure npm installed: `npm --version`

### 2. Credentials & Accounts

- [ ] OpenAI account created with API key
- [ ] Qdrant Cloud account with collection URL
- [ ] Neon PostgreSQL account with database
- [ ] Render.com account (for backend hosting)
- [ ] Vercel account (for frontend hosting)
- [ ] GitHub account with repo access

### 3. Local Development Setup

**Run Setup Script:**
```bash
# Linux/Mac
chmod +x setup-ragbot.sh
./setup-ragbot.sh

# Windows
setup-ragbot.bat
```

- [ ] Setup script completed without errors
- [ ] Python venv created
- [ ] Dependencies installed
- [ ] `.env` file created from `.env.example`

### 4. Configure Credentials

**Edit `ragbot-api/.env`:**
```bash
OPENAI_API_KEY=<REDACTED>

QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>

DATABASE_URL=postgresql://neondb_owner:npg_mrIvVkhZQx37@ep-purple-resonance-a40w6551-pooler.us-east-1.aws.neon.tech/neondb

FRONTEND_URL=https://physical-ai-humanoid-robotics-hacka.vercel.app/
```

- [ ] `OPENAI_API_KEY` updated with actual key
- [ ] `QDRANT_URL` set to Qdrant instance
- [ ] `QDRANT_API_KEY` set to Qdrant key
- [ ] `DATABASE_URL` set to Neon database URL
- [ ] `FRONTEND_URL` updated (after Vercel deployment)

### 5. Document Ingestion

```bash
# From project root
python ragbot-ingest.py
```

- [ ] Ingestion script ran successfully
- [ ] All files processed (15 markdown files)
- [ ] Chunks created (247 chunks expected)
- [ ] Vectors stored in Qdrant (247 vectors)
- [ ] Database logs created

**Check Status:**
```bash
cd ragbot-api
python -c "from qdrant_client import get_collection_info; print(get_collection_info())"
```

- [ ] Collection "robotics_docs" exists
- [ ] Point count = 247
- [ ] Vector size = 1536

---

## Local Testing

### 6. Start Backend Server

```bash
cd /workspaces/hacks022/ragbot-api
source venv/bin/activate  # Windows: venv\Scripts\activate
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

- [ ] Backend starts without errors
- [ ] "Uvicorn running on http://0.0.0.0:8000" appears
- [ ] No import errors or exceptions

### 7. Test API Endpoints

**Health Check:**
```bash
curl http://localhost:8000/api/health
```

- [ ] Returns `{"status": "healthy", ...}`

**API Documentation:**
```
http://localhost:8000/docs
```

- [ ] Swagger UI loads
- [ ] All endpoints visible
- [ ] Can expand endpoint details

**Test Chat Endpoint:**
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "mode": "fullbook",
    "user_id": "test_user"
  }'
```

- [ ] Returns 200 OK
- [ ] Contains "response" field
- [ ] Contains "sources" array
- [ ] Contains "session_id"

### 8. Start Frontend

```bash
# In new terminal from /workspaces/hacks022
npm start
```

- [ ] Docusaurus starts without errors
- [ ] "Local: http://localhost:3000" appears
- [ ] No build errors

### 9. Test Frontend

**Main Site:**
```
http://localhost:3000
```

- [ ] Homepage loads
- [ ] Navigation works
- [ ] No console errors

**RAG Chatbot Page:**
```
http://localhost:3000/ragbot
```

- [ ] Chat page loads
- [ ] Chat interface visible
- [ ] Mode selector visible
- [ ] Input field visible
- [ ] No console errors

### 10. Test Chat Functionality

**FULLBOOK Mode Test:**
- [ ] Select "FULLBOOK" mode
- [ ] Type "What is ROS2?"
- [ ] Click send or press Enter
- [ ] Response appears (wait ~3 seconds)
- [ ] Sources display at bottom
- [ ] Relevance scores visible
- [ ] Module names in sources

**SELECTED Mode Test:**
- [ ] Select "SELECTED" mode
- [ ] Paste text in "Selected Text" box
- [ ] Type "What does this say?"
- [ ] Click send
- [ ] Response appears (wait ~1-2 seconds)
- [ ] No sources shown
- [ ] Answer based only on pasted text

### 11. Test Session History

- [ ] Multiple messages in conversation
- [ ] Previous messages still visible
- [ ] Refresh page (F5)
- [ ] History persists (same session_id)
- [ ] New session_id if changing user

### 12. Test Rating System

- [ ] Find message response
- [ ] Click sources area (if expandable)
- [ ] Verify message stored in database

---

## Backend Deployment (Render)

### 13. Prepare for Render Deployment

```bash
cd /workspaces/hacks022
git add .
git commit -m "Add RAG chatbot system"
git push origin main
```

- [ ] Code pushed to GitHub
- [ ] All files committed
- [ ] No uncommitted changes

### 14. Create Render Web Service

1. Go to [render.com](https://render.com)
2. Click "New +" button
3. Select "Web Service"
4. Connect GitHub account
5. Select `hacks022` repository

- [ ] GitHub account connected
- [ ] Repository visible in Render
- [ ] Ready to deploy

### 15. Configure Render Deployment

1. **Name**: `ragbot-api`
2. **Environment**: `Python 3`
3. **Root Directory**: `ragbot-api`
4. **Build Command**: `pip install -r requirements.txt`
5. **Start Command**: 
   ```
   gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app
   ```

- [ ] Build command correct
- [ ] Start command correct
- [ ] Root directory points to ragbot-api/

### 16. Add Environment Variables to Render

Click "Advanced" and add:

```
OPENAI_API_KEY=<REDACTED>
QDRANT_URL=<REDACTED>
QDRANT_API_KEY=<REDACTED>
DATABASE_URL=postgresql://...
FRONTEND_URL=https://...  (update later)
```

- [ ] All 5 environment variables added
- [ ] Values copied correctly (no typos)
- [ ] No extra spaces in values

### 17. Deploy to Render

1. Click "Create Web Service"
2. Wait for build and deployment
3. Monitor build log for errors
4. Deployment should complete in 2-5 minutes

- [ ] "Build successful" message appears
- [ ] "Service started" message appears
- [ ] No error logs visible
- [ ] Deployment URL appears (e.g., ragbot-api.onrender.com)

### 18. Test Backend in Production

```bash
# Replace with your Render URL
curl https://ragbot-api.onrender.com/api/health
```

- [ ] Returns `{"status": "healthy", ...}`
- [ ] No 502/503 errors
- [ ] Response time reasonable

**Full Chat Test:**
```bash
curl -X POST https://ragbot-api.onrender.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "mode": "fullbook"
  }'
```

- [ ] Returns full response with sources
- [ ] No rate limit errors
- [ ] Latency acceptable (~3 seconds)

---

## Frontend Deployment (Vercel)

### 19. Update Frontend API URL

**In `src/pages/ragbot.jsx`, update:**
```javascript
apiUrl={process.env.REACT_APP_API_URL || 'https://ragbot-api.onrender.com/api'}
```

OR set environment variable in Vercel.

- [ ] API URL points to Render backend
- [ ] No localhost references

### 20. Push Frontend Code

```bash
git add src/pages/ragbot.jsx
git commit -m "Update API URL for production"
git push origin main
```

- [ ] Changes pushed to GitHub
- [ ] Ready for Vercel import

### 21. Deploy to Vercel

1. Go to [vercel.com](https://vercel.com)
2. Click "Add New..."
3. Select "Project"
4. Import from Git → select `hacks022` repo
5. Click "Import"

- [ ] Repository imported
- [ ] Root directory auto-detected
- [ ] Framework: Docusaurus selected

### 22. Configure Vercel Environment

Click "Environment Variables" and add:

```
REACT_APP_API_URL=https://ragbot-api.onrender.com/api
```

- [ ] Environment variable added
- [ ] Value matches Render API URL
- [ ] No trailing slashes or extra spaces

### 23. Deploy to Vercel

Click "Deploy"

- [ ] Build starts automatically
- [ ] Build completes successfully (2-5 minutes)
- [ ] "Deployment Complete" message appears
- [ ] Deployment URL provided (e.g., physical-ai-humanoid-robotics-hacka.vercel.app)

### 24. Test Production Frontend

Visit deployment URL:
```
https://physical-ai-humanoid-robotics-hacka.vercel.app
```

- [ ] Site loads
- [ ] Navigation works
- [ ] No 404 errors

### 25. Test Production RAG Chatbot

```
https://physical-ai-humanoid-robotics-hacka.vercel.app/ragbot
```

- [ ] Chat page loads
- [ ] Chat interface visible
- [ ] No console errors
- [ ] API connects to Render backend

**Test Chat:**
- [ ] Type question
- [ ] Get response from production API
- [ ] Sources display correctly
- [ ] Session persists

---

## Post-Deployment Verification

### 26. Health Checks

```bash
# Backend health
curl https://ragbot-api.onrender.com/api/health

# Frontend accessible
curl -s https://physical-ai-humanoid-robotics-hacka.vercel.app | grep -q "html" && echo "Frontend OK"

# Chat endpoint responsive
curl -X POST https://ragbot-api.onrender.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"test","mode":"fullbook"}'
```

- [ ] Backend health check passes
- [ ] Frontend loads without 404
- [ ] Chat endpoint responsive

### 27. Monitor Logs

**Render Backend Logs:**
- [ ] Visit Render dashboard
- [ ] Select `ragbot-api` service
- [ ] Check "Logs" tab
- [ ] No ERROR messages
- [ ] "Server started" message visible

**Vercel Frontend Logs:**
- [ ] Visit Vercel deployment
- [ ] Check "Deployments" tab
- [ ] Click latest deployment
- [ ] Check "Logs"
- [ ] No build errors

### 28. Update Database URL (if needed)

If using development database for testing:
- [ ] Create production Neon database
- [ ] Update `DATABASE_URL` in Render environment
- [ ] Restart Render service
- [ ] Verify chat history saves

### 29. Update Frontend URL (if needed)

In `ragbot-api/.env` and Render environment:
```
FRONTEND_URL=https://physical-ai-humanoid-robotics-hacka.vercel.app
```

- [ ] Updated to production frontend URL
- [ ] Render service restarted
- [ ] CORS now includes production domain

### 30. Security Checklist

- [ ] No secrets in code (all in .env)
- [ ] API keys not visible in logs
- [ ] CORS configured for production domains
- [ ] HTTPS enforced (automatic on Vercel/Render)
- [ ] Rate limiting enabled (if desired)
- [ ] Database password strong and unique

---

## Post-Deployment Maintenance

### 31. Monitor Usage

**Daily:**
- [ ] Check Render logs for errors
- [ ] Monitor OpenAI API usage/costs
- [ ] Check Qdrant vector search latency

**Weekly:**
- [ ] Review chat logs in database
- [ ] Check error rates
- [ ] Verify all features working

**Monthly:**
- [ ] Review cost estimates
- [ ] Update dependencies
- [ ] Backup database
- [ ] Check OpenAI API quotas

### 32. Setup Monitoring (Optional)

- [ ] Add error tracking (Sentry, etc.)
- [ ] Setup uptime monitoring (UptimeRobot)
- [ ] Create database backup schedule
- [ ] Monitor API response times

### 33. Documentation Updates

- [ ] Update production URLs in docs
- [ ] Document any custom changes
- [ ] Create runbooks for common issues
- [ ] Add deployment troubleshooting guide

### 34. Performance Optimization (Optional)

- [ ] Monitor vector search latency
- [ ] Consider adding Redis cache for frequently asked questions
- [ ] Optimize chunk size if needed
- [ ] Adjust top_k_results based on usage patterns

---

## Troubleshooting During Deployment

### Backend Build Fails

**Symptoms:** Render build logs show errors

**Solutions:**
1. Check Python version compatibility
2. Verify `requirements.txt` has all dependencies
3. Check for import errors in code
4. Review Render build logs for specific error

### Backend Crashes After Deploy

**Symptoms:** Service starts but crashes immediately

**Solutions:**
1. Check environment variables are set in Render
2. Verify database connection string
3. Check Qdrant URL and API key
4. Review Render runtime logs

### Frontend Can't Connect to API

**Symptoms:** Chat sends message but no response

**Solutions:**
1. Verify `REACT_APP_API_URL` environment variable
2. Check backend is running and healthy
3. Verify CORS settings in `config.py`
4. Check browser console for CORS errors

### Documents Not Found in Search

**Symptoms:** Chat returns "no relevant documents"

**Solutions:**
1. Verify documents were ingested: `GET /api/ingest/status`
2. Re-run ingestion if needed
3. Check `/docs` folder has markdown files
4. Verify Qdrant collection has vectors

### Database Connection Fails

**Symptoms:** Chat throws database error

**Solutions:**
1. Verify `DATABASE_URL` in Render environment
2. Check Neon database is active
3. Ensure IP whitelisting if needed
4. Test connection string locally

---

## Success Criteria

✅ **All of the following must be true:**

- [ ] Backend API running on Render (health check passes)
- [ ] Frontend site accessible on Vercel
- [ ] RAG chat page loads at `/ragbot` route
- [ ] FULLBOOK mode works (searches vectors, returns sources)
- [ ] SELECTED mode works (uses only selected text)
- [ ] Messages saved to database
- [ ] Session history preserved
- [ ] Sources displayed with relevance scores
- [ ] No console errors on frontend
- [ ] No error logs on backend
- [ ] Chat latency acceptable (~2-3 seconds)
- [ ] Multiple users can use simultaneously

---

## Final Sign-Off

- [ ] All checklist items completed
- [ ] Production system tested and working
- [ ] Team trained on deployment process
- [ ] Documentation updated
- [ ] Backups configured
- [ ] Monitoring setup (if applicable)
- [ ] Go-live decision made

**Deployment Date:** _______________  
**Deployed By:** _______________  
**Verified By:** _______________  

---

**Version:** 1.0.0  
**Last Updated:** December 2024
