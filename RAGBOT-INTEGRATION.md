# RAG Chatbot Integration into Docusaurus

This document describes how the RAG chatbot is integrated into the existing Docusaurus site.

## File Structure

```
/workspaces/hacks022/
├── ragbot-api/                    # FastAPI backend
│   ├── main.py                    # FastAPI application
│   ├── config.py                  # Configuration settings
│   ├── db.py                      # Database models
│   ├── qdrant_client.py          # Vector store client
│   ├── embeddings.py             # OpenAI embeddings
│   ├── requirements.txt           # Python dependencies
│   ├── .env.example              # Environment template
│   └── routers/
│       ├── chat.py               # Chat endpoints
│       └── ingest.py             # Document ingestion
├── ragbot-ui/                    # React components
│   ├── ChatInterface.jsx         # Main chat component
│   ├── ModeSelector.jsx          # Mode selection UI
│   ├── SelectedTextBox.jsx       # Text selection UI
│   ├── SourceDisplay.jsx         # Source attribution
│   └── ChatInterface.module.css  # Component styles
├── src/
│   └── pages/
│       └── ragbot.jsx            # Docusaurus page wrapper
├── docs/
│   └── RAGBOT-SPECIFICATIONS.md  # Full specifications
├── ragbot-ingest.py              # Document ingestion script
└── RAGBOT-SETUP.md              # This file
```

## Integration Points

### 1. New Route: `/ragbot`

Created in `src/pages/ragbot.jsx`:
- Wraps ChatInterface component
- Integrated with Docusaurus Layout
- Responsive design

Access at: `http://localhost:3000/ragbot`

### 2. Navbar Button (Optional)

To add "Ask the Book 🤖" button to navbar, add to `docusaurus.config.js`:

```javascript
const config = {
  // ... existing config
  navbar: {
    items: [
      // ... existing items
      {
        href: '/ragbot',
        label: 'Ask the Book 🤖',
        position: 'right',
      },
    ],
  },
};
```

### 3. React Components

Four React components in `ragbot-ui/`:

1. **ChatInterface.jsx** - Main chat component
   - Message management
   - Session handling
   - API communication
   - ~400 lines

2. **ModeSelector.jsx** - Mode selection buttons
   - Fullbook RAG mode
   - Selected text mode
   - ~30 lines

3. **SelectedTextBox.jsx** - Text selection
   - User text input
   - Character counter
   - ~40 lines

4. **SourceDisplay.jsx** - Source attribution
   - Collapsible sources
   - Relevance scores
   - Metadata display
   - ~50 lines

### 4. Styling

`ChatInterface.module.css`:
- CSS Modules for component isolation
- Responsive design
- Dark/light theme support
- ~400 lines

### 5. API Communication

Backend API at: `http://localhost:8000` (development)

Endpoints used by frontend:
- `POST /api/chat` - Send message
- `GET /api/chat/history/{session_id}` - Load history
- `POST /api/chat/rate` - Rate response

## How to Use in Docusaurus

### Option 1: Use Existing Route

Simply navigate to `/ragbot`:
```
http://localhost:3000/ragbot
```

### Option 2: Add to Navbar

Edit `docusaurus.config.js`:
```javascript
navbar: {
  items: [
    {
      href: '/ragbot',
      label: '🤖 Ask Book',
      position: 'right',
    },
  ],
},
```

### Option 3: Embed in Documentation

Add to any markdown page:
```jsx
import ChatInterface from '../../ragbot-ui/ChatInterface';

## Ask the Book

<ChatInterface apiUrl="/api" />
```

## Configuration

### Backend Configuration

Edit `ragbot-api/config.py`:

```python
# Model settings
openai_model = "gpt-4-turbo"
chunk_size = 500
top_k_results = 5

# Rate limiting
rate_limit_calls = 100
rate_limit_period = 3600

# CORS origins
cors_origins = [
    "http://localhost:3000",
    "https://your-vercel-domain.com",
]
```

### Frontend Configuration

Edit `src/pages/ragbot.jsx`:

```javascript
<ChatInterface 
  apiUrl={process.env.REACT_APP_API_URL || '/api'}
/>
```

Set `REACT_APP_API_URL` environment variable:
- Local: `http://localhost:8000`
- Production: `https://your-api.com`

## Development Workflow

### 1. Start Backend

```bash
cd ragbot-api
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Start Frontend

```bash
npm install  # If not done
npm start
```

### 3. Test Chatbot

Visit: `http://localhost:3000/ragbot`

### 4. Modify Components

Edit files in `ragbot-ui/` and changes hot-reload in Docusaurus

## Deployment Workflow

### 1. Backend to Render

```bash
# Push changes
git add .
git commit -m "Update RAG chatbot"
git push origin main

# Render auto-deploys from GitHub
# Monitor at render.com dashboard
```

### 2. Frontend to Vercel

```bash
# Frontend auto-deploys on git push
# Set environment variable:
# REACT_APP_API_URL=https://your-render-url
```

### 3. Verify

```bash
# Test API
curl https://your-render-url/api/health

# Test frontend
https://your-vercel-domain.com/ragbot
```

## Troubleshooting

### Issue: Chat not loading
1. Check console errors (F12)
2. Verify backend is running
3. Check API URL in ragbot.jsx
4. Check CORS settings in config.py

### Issue: Documents not showing
1. Run `python ragbot-ingest.py` to ingest docs
2. Check Qdrant collection at: `GET /api/ingest/status`
3. Verify `/docs` folder has markdown files

### Issue: Responses are slow
1. Check OpenAI API status
2. Increase chunk_size to reduce vectors
3. Reduce top_k_results
4. Monitor token usage

### Issue: API 404 errors
1. Verify backend is running on correct port
2. Check API URL in frontend
3. Verify API routes are registered
4. Check browser network tab

## Next Steps

1. ✅ Backend fully implemented
2. ✅ Frontend components ready
3. ✅ Docusaurus integration complete
4. ⬜ Deploy backend to Render
5. ⬜ Deploy frontend to Vercel
6. ⬜ Ingest documents to Qdrant
7. ⬜ Test in production
8. ⬜ Add navbar button
9. ⬜ Set up monitoring

## Support

See `docs/RAGBOT-SPECIFICATIONS.md` for:
- Full API documentation
- Database schema
- Security guidelines
- Deployment instructions
- Rate limiting strategy
- Maintenance procedures
