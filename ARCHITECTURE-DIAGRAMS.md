# RAG Chatbot Architecture & Data Flow Diagrams

## System Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                             FRONTEND LAYER                                  │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │                     Docusaurus Website                                 │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                 │  │
│  │  │   /docs      │  │  /blog       │  │ /ragbot      │ NEW             │  │
│  │  │   pages      │  │  pages       │  │ Chat Page    │                 │  │
│  │  └──────────────┘  └──────────────┘  └──────┬───────┘                 │  │
│  │                                            │                          │  │
│  │  ┌─────────────────────────────────────────▼──────────────────────┐   │  │
│  │  │           ChatInterface React Component                        │   │  │
│  │  │  ┌────────────────┐ ┌────────────────┐ ┌────────────────┐     │   │  │
│  │  │  │ Mode Selector  │ │ Selected Text  │ │ Source Display │     │   │  │
│  │  │  │ (FULLBOOK)     │ │ Box (SELECTED) │ │ (with scoring) │     │   │  │
│  │  │  └────────────────┘ └────────────────┘ └────────────────┘     │   │  │
│  │  │                                                                 │   │  │
│  │  │  Message History ← Session Management → API Communication     │   │  │
│  │  └─────────────────────────────────────────────────────────────────┘   │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
                                    ↕
                              HTTP/REST (JSON)
                                    ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                        BACKEND API LAYER (FastAPI)                           │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │                        main.py (FastAPI app)                          │  │
│  │  ├─ CORS Middleware                                                  │  │
│  │  ├─ Exception Handlers                                               │  │
│  │  ├─ Lifespan (Startup/Shutdown)                                      │  │
│  │  └─ Health Check Endpoint                                            │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                    │                                          │
│  ┌────────────────────────────────┴──────────────────────────────────────┐   │
│  │                          Router Layer                                 │   │
│  │  ┌──────────────────────────────────┬────────────────────────────┐   │   │
│  │  │      routers/chat.py             │  routers/ingest.py         │   │   │
│  │  ├─ POST /api/chat                  ├─ POST /api/ingest/docs     │   │   │
│  │  ├─ GET /api/chat/history           ├─ GET /api/ingest/status    │   │   │
│  │  ├─ POST /api/chat/rate             └─ POST /api/ingest/reset    │   │   │
│  │  └─ Session Management                                            │   │   │
│  │                                                                     │   │   │
│  │  ┌──────────────────────────────────────────────────────────────┐  │   │
│  │  │              Business Logic Layer                           │  │   │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐            │  │   │
│  │  │  │  config.py │  │  db.py     │  │utils.py    │            │  │   │
│  │  │  │ Settings   │  │ Models &   │  │Helpers &   │            │  │   │
│  │  │  │            │  │ Connection │  │Utilities   │            │  │   │
│  │  │  └────────────┘  └────────────┘  └────────────┘            │  │   │
│  │  └──────────────────────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────────┘
         │                        │                      │
         ▼                        ▼                      ▼
    ┌─────────────┐         ┌───────────┐         ┌──────────────┐
    │ OpenAI API  │         │ Qdrant    │         │ Neon DB      │
    │             │         │ Cloud     │         │              │
    │ • GPT-4     │         │           │         │ • Sessions   │
    │ • Embeddings│         │ • Vectors │         │ • Messages   │
    │ • Responses │         │ • Storage │         │ • Logs       │
    └─────────────┘         └───────────┘         └──────────────┘
```

---

## Chat Request Flow (FULLBOOK MODE)

```
User enters query: "What is ROS2?"
        │
        ▼
┌───────────────────┐
│ ChatInterface.jsx │ Capture query, set mode="fullbook"
└────────┬──────────┘
         │
         ▼ POST /api/chat
┌──────────────────────────────────┐
│    routers/chat.py               │
│    chat() endpoint               │
└────────┬─────────────────────────┘
         │
         ├─ Create/Get Session (db.py)
         │  └─ Store session in Neon if new
         │
         ├─ Sanitize & validate query (utils.py)
         │
         ├─ Check mode == "fullbook"
         │
         ▼ YES, FULLBOOK MODE
┌────────────────────────────────┐
│  Generate Query Embedding      │  embeddings.py
│  openai.Embedding.create()     │  generate_query_embedding()
│  Model: text-embedding-3-small │
│  Input: "What is ROS2?"        │
└────────┬───────────────────────┘
         │
         ▼ Embedding vector [1536 dimensions]
┌────────────────────────────────┐
│ Search Vectors in Qdrant       │  qdrant_client.py
│ client.search()                │  search_vectors()
│ - query_vector: embedding      │
│ - limit: 5 (top_k_results)     │
│ - threshold: 0.5               │
└────────┬───────────────────────┘
         │
         ▼ Top 5 results with metadata
┌────────────────────────────────────────┐
│ Results:                               │
│  1. module01-ros2/02-nodes.md (0.92)  │
│  2. module01-ros2/03-services.md (0.89)
│  3. module02-digital-twin/01-intro.md (0.78)
│  ...                                   │
└────────┬───────────────────────────────┘
         │
         ▼
┌────────────────────────────────────┐
│ Build Context String              │  chat.py
│ Combine top-5 chunks with         │  build_user_prompt()
│ module + filename + section info   │
└────────┬───────────────────────────┘
         │
         ▼
┌──────────────────────────────────────┐
│  Call OpenAI GPT-4 Turbo            │  openai.ChatCompletion.create()
│  - system_prompt: expertise + rules │  Model: gpt-4-turbo
│  - user_prompt: query + context     │
│  - temperature: 0.7                 │
│  - max_tokens: 1000                 │
└────────┬───────────────────────────┘
         │
         ▼ Response from OpenAI
┌──────────────────────────────────────┐
│ "ROS2 (Robot Operating System 2)... │
│  [detailed answer with context]      │
│  According to module01-ros2/02-...  │
└────────┬───────────────────────────┘
         │
         ▼
┌────────────────────────────────────┐
│ Create ChatMessage Record          │  db.py
│ Store in Neon PostgreSQL:          │
│  - session_id                      │
│  - user_query: "What is ROS2?"     │
│  - bot_response: answer text       │
│  - mode: "fullbook"                │
│  - sources: [source list JSON]     │
│  - tokens_used: 487                │
│  - created_at: timestamp           │
└────────┬───────────────────────────┘
         │
         ▼
┌──────────────────────────────────────┐
│ Return ChatResponse JSON            │
│ {                                   │
│   "response": "ROS2 is...",         │
│   "session_id": "session_123",      │
│   "sources": [                      │
│     {                               │
│       "filename": "02-nodes.md",    │
│       "module": "module01-ros2",    │
│       "section": "ROS2 Basics",     │
│       "score": 0.92,                │
│       "content": "..."              │
│     }                               │
│   ],                                │
│   "tokens_used": 487,               │
│   "mode": "fullbook"                │
│ }                                   │
└────────┬───────────────────────────┘
         │
         ▼
      Frontend renders
      - Answer text
      - Collapsible sources with scores
      - Save to message history
```

---

## Chat Request Flow (SELECTED MODE)

```
User selects text from /docs page
User enters query with selected text
        │
        ▼
┌───────────────────┐
│ ChatInterface.jsx │ Capture query & selected_text
│ SelectedTextBox   │ mode="selected"
└────────┬──────────┘
         │
         ▼ POST /api/chat
┌──────────────────────────────────┐
│    routers/chat.py               │
│    chat() endpoint               │
└────────┬─────────────────────────┘
         │
         ├─ Create/Get Session
         │
         ├─ Validate selected_text present
         │  (Required for "selected" mode)
         │
         ├─ Check mode == "selected"
         │
         ▼ YES, SELECTED MODE
┌────────────────────────────────┐
│ Build Prompt with:             │  chat.py
│ - System: expertise rules      │  build_user_prompt()
│ - Query: "What does this...?"  │
│ - Context: selected_text ONLY  │
│ - Constraint: "Answer ONLY...  │
│   from selected text"          │
└────────┬───────────────────────┘
         │
         ▼ NO QDRANT SEARCH (skipped)
┌──────────────────────────────────┐
│  Call OpenAI GPT-4 Turbo        │  openai.ChatCompletion.create()
│  - system: expertise            │
│  - user: query + selected text  │
│  - temperature: 0.7             │
│  - max_tokens: 1000             │
└────────┬───────────────────────┘
         │
         ▼ Response from OpenAI
┌──────────────────────────────────┐
│ "This refers to... [based on    │
│  selected text only]"            │
└────────┬───────────────────────┘
         │
         ▼
┌────────────────────────────────┐
│ Create ChatMessage Record      │  db.py
│ Store in Neon:                 │
│  - session_id                  │
│  - user_query                  │
│  - bot_response                │
│  - mode: "selected"            │
│  - selected_text: [text]       │
│  - source_sections: [] (empty) │
│  - tokens_used                 │
└────────┬───────────────────────┘
         │
         ▼
┌──────────────────────────────────┐
│ Return ChatResponse JSON        │
│ {                               │
│   "response": "This refers to...",
│   "session_id": "session_123",  │
│   "sources": [],  (empty array) │
│   "tokens_used": 234,           │
│   "mode": "selected"            │
│ }                               │
└────────┬───────────────────────┘
         │
         ▼
      Frontend renders
      - Answer text
      - No sources (mode constraint)
      - Clear selection indicator
```

---

## Document Ingestion Pipeline

```
Administrator / Setup Process
        │
        ▼ Run: python ragbot-ingest.py
┌───────────────────────────────────┐
│  ragbot-ingest.py                 │
│  main() function                  │
└────────┬────────────────────────┬─┘
         │                        │
         ▼                        ▼
┌──────────────────┐    ┌────────────────────┐
│ Init Database    │    │ Create Qdrant      │
│ init_db()        │    │ Collection         │
│ Create tables    │    │ robotics_docs      │
└────────┬─────────┘    └────────┬───────────┘
         │                       │
         └───────────┬───────────┘
                     │
                     ▼
        ┌──────────────────────────┐
        │ Scan /docs recursively   │
        │ Find *.md and *.mdx      │
        │ 15 files found           │
        └────────┬─────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Extract Module from Path     │
        │ module01-ros2/02-nodes.md    │
        │ → module01-ros2              │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Extract Sections by Headers  │
        │ Split #, ##, ### into        │
        │ separate content chunks      │
        │ 247 total chunks created     │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Extract Metadata:            │
        │ - filename                   │
        │ - module                     │
        │ - section (header)           │
        │ - content (preview)          │
        │ - full_path                  │
        │ - created_at                 │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Generate Embeddings (batch)  │
        │ embeddings.py                │
        │ model: text-embedding-3-sm   │
        │ batch_size: 25               │
        │ 247 embeddings generated     │
        │ dimension: 1536              │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Store in Qdrant (batch)      │
        │ qdrant_client.py             │
        │ batch_size: 100              │
        │ upsert operation             │
        │ 247 vectors stored           │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ Log Ingestion (Neon DB)      │
        │ db.py IngestionLog table     │
        │ - doc_path                   │
        │ - chunks_created: 247        │
        │ - vectors_stored: 247        │
        │ - status: success            │
        │ - created_at: timestamp      │
        └────────┬─────────────────────┘
                 │
                 ▼
        ┌──────────────────────────────┐
        │ ✓ INGESTION COMPLETE         │
        │                              │
        │ Summary:                     │
        │ Files: 15                    │
        │ Chunks: 247                  │
        │ Vectors: 247                 │
        │ Status: Ready for Search     │
        └──────────────────────────────┘
```

---

## Database Entity Relationships

```
┌─────────────────────────────────────────────────┐
│              DATABASE SCHEMA                    │
│           (Neon PostgreSQL)                     │
└─────────────────────────────────────────────────┘

┌────────────────────────┐
│   user_sessions        │
├────────────────────────┤
│ id (PK)                │
│ user_id                │────────┐
│ created_at             │        │
│ updated_at             │        │
│ metadata               │        │
└────────────────────────┘        │
                                  │
                    ┌─────────────▼──────────┐
                    │  chat_messages         │
                    ├────────────────────────┤
                    │ id (PK)                │
                    │ session_id (FK) ◄──────┤
                    │ user_query             │
                    │ bot_response           │
                    │ mode                   │
                    │ selected_text          │
                    │ source_sections (JSON) │
                    │ tokens_used            │
                    │ created_at             │
                    │ user_rating            │
                    └────────────────────────┘

┌────────────────────────┐
│   ingestion_logs       │
├────────────────────────┤
│ id (PK)                │
│ doc_path               │
│ chunks_created         │
│ vectors_stored         │
│ status                 │
│ error_message          │
│ created_at             │
└────────────────────────┘
```

---

## Vector Store Structure (Qdrant)

```
Qdrant Collection: robotics_docs
├─ Vector Size: 1536 dimensions
├─ Distance Metric: COSINE
├─ Points: 247
│
└─ Sample Point Structure:
   ┌────────────────────────────────────────┐
   │ Point ID: 1733493087654321             │
   │ Vector: [0.235, -0.102, 0.456, ...]    │
   │                                        │
   │ Payload (Metadata):                    │
   │ ├─ chunk_id: "chunk_12345"             │
   │ ├─ filename: "02-ros2-nodes-topics.md" │
   │ ├─ module: "module01-ros2"             │
   │ ├─ section: "ROS2 Nodes"               │
   │ ├─ content: "ROS2 nodes are..."        │
   │ ├─ full_path: "module01/.../02-..."    │
   │ └─ created_at: "2024-12-06T10:30Z"     │
   └────────────────────────────────────────┘
```

---

## API Request/Response Cycle

```
CLIENT REQUEST
└─ POST /api/chat
   ├─ Headers: Content-Type: application/json
   │
   └─ Body: ChatRequest
      {
        "query": "What is ROS2?",
        "mode": "fullbook",
        "selected_text": null,
        "session_id": "session_123",
        "user_id": "user_123"
      }
           │
           ▼
     [FastAPI validates]
     [Pydantic model]
           │
           ▼
     [Router processes]
     [Business logic]
           │
           ▼
SERVER RESPONSE
└─ 200 OK
   ├─ Headers: Content-Type: application/json
   │
   └─ Body: ChatResponse
      {
        "response": "ROS2 is the Robot...",
        "session_id": "session_123",
        "sources": [
          {
            "score": 0.92,
            "filename": "02-nodes.md",
            "module": "module01-ros2",
            "section": "ROS2 Fundamentals",
            "content": "ROS2 is...",
            "full_path": "module01-ros2/02-..."
          }
        ],
        "tokens_used": 487,
        "mode": "fullbook"
      }
```

---

## Component Interaction Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Frontend                      │
│  ┌──────────────────────────────────────────────────────┐   │
│  │             ChatInterface Component                  │   │
│  │                                                      │   │
│  │  State:                                             │   │
│  │  ├─ messages[]      (conversation)                 │   │
│  │  ├─ query (input)                                  │   │
│  │  ├─ sessionId                                      │   │
│  │  ├─ mode (fullbook|selected)                       │   │
│  │  ├─ selectedText                                   │   │
│  │  └─ sources[]                                      │   │
│  │                                                      │   │
│  │  Child Components:                                  │   │
│  │  ├─ ModeSelector                                   │   │
│  │  │  └─ onChange → setMode()                        │   │
│  │  │                                                  │   │
│  │  ├─ SelectedTextBox (conditional)                  │   │
│  │  │  └─ onChange → setSelectedText()                │   │
│  │  │                                                  │   │
│  │  └─ SourceDisplay (in messages)                    │   │
│  │     └─ Shows relevance scores                      │   │
│  │                                                      │   │
│  │  Effects:                                           │   │
│  │  ├─ useEffect on mount → initSession()             │   │
│  │  ├─ useEffect on messages → scrollToBottom()       │   │
│  │  └─ useEffect on sessionId → loadHistory()         │   │
│  │                                                      │   │
│  │  Event Handlers:                                    │   │
│  │  └─ handleSendMessage()                            │   │
│  │     ├─ Validation                                  │   │
│  │     ├─ POST /api/chat                              │   │
│  │     ├─ Handle response                             │   │
│  │     └─ Update messages[]                           │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ HTTP
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   FastAPI Backend                           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            FastAPI Application                       │   │
│  │                                                      │   │
│  │  ├─ CORS Middleware                                │   │
│  │  ├─ Exception Handlers                              │   │
│  │  └─ Routes                                           │   │
│  │     ├─ /api/chat (POST)                             │   │
│  │     ├─ /api/chat/history (GET)                      │   │
│  │     ├─ /api/ingest/docs (POST)                      │   │
│  │     └─ ... (more routes)                            │   │
│  │                                                      │   │
│  │  Router: chat.py                                    │   │
│  │  ├─ chat() endpoint                                 │   │
│  │  │  └─ Logic flow (see previous diagrams)           │   │
│  │  ├─ get_chat_history() endpoint                     │   │
│  │  └─ rate_message() endpoint                         │   │
│  │                                                      │   │
│  │  Dependencies:                                      │   │
│  │  ├─ embeddings.generate_query_embedding()          │   │
│  │  ├─ qdrant_client.search_vectors()                 │   │
│  │  ├─ db.get_db() session                            │   │
│  │  └─ openai.ChatCompletion.create()                │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
         │                  │                    │
         ▼                  ▼                    ▼
    ┌─────────┐      ┌────────────┐      ┌────────────┐
    │ OpenAI  │      │  Qdrant    │      │   Neon     │
    │  API    │      │   Cloud    │      │    DB      │
    └─────────┘      └────────────┘      └────────────┘
```

---

**Diagrams Summary**: This document provides 10 different views of the RAG chatbot system architecture, showing how data flows from user input through the system to external services and back to the frontend for display.
