---
title: Physical AI & Humanoid Robotics
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_file: ragbot-api/main.py
pinned: false
---

# 🤖 Physical AI & Humanoid Robotics

> **Interactive Learning Platform with AI-Powered Chatbot Assistant**

A comprehensive educational platform for Physical AI and Humanoid Robotics featuring an intelligent RAG-powered chatbot backend. This FastAPI-based service provides real-time conversational AI assistance for learning robotics concepts, ROS 2, digital twins, NVIDIA Isaac, and VLA (Vision-Language-Action) models through advanced Retrieval-Augmented Generation technology.

## ✨ Features

### 🤖 AI Chatbot Assistant
- 🧠 **Intelligent RAG Chatbot** - AI assistant trained on robotics documentation
- 💬 **Real-time Q&A** - Instant answers about ROS 2, Isaac Sim, digital twins
- 🎯 **Context-Aware Responses** - Understands robotics concepts and terminology
- 📖 **Learning Support** - Guides through Physical AI and Humanoid Robotics modules

### 🚀 Technical Infrastructure
- ⚡ **FastAPI Backend** - High-performance async API with auto-documentation
- 🔍 **Vector Search** - Qdrant-powered semantic search for precise information retrieval
- 🌩️ **Groq Integration** - Lightning-fast LLM inference for real-time responses
- 📚 **Document Processing** - Automated ingestion of robotics educational content
- 🔒 **Secure & Scalable** - JWT authentication with Docker deployment
- 🌐 **Cross-Platform** - CORS-enabled for web and mobile applications

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Learning UI    │────│  Chatbot API    │────│  Vector Store   │
│  (Robotics      │    │  (RAG Engine)   │    │  (Qdrant)       │
│   Platform)     │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                       ┌─────────────────┐
                       │   AI Assistant  │
                       │   (Groq LLM)    │
                       └─────────────────┘

📚 Knowledge Base: ROS 2 • Digital Twins • NVIDIA Isaac • VLA Models
```

## 🚀 Quick Start

### Prerequisites

- Python 3.8+
- Docker & Docker Compose
- Groq API Key
- Qdrant Cloud account (or local instance)

### 1. Clone the Repository

```bash
git clone <repository-url>
cd hacks022
```

### 2. Environment Setup

Create a `.env.local` file in the root directory:

```env
# API Configuration
OPENAI_API_KEY=your_groq_api_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=postgresql://user:password@localhost/ragbot
FRONTEND_URL=http://localhost:3000

# Optional: Redis for caching
REDIS_URL=redis://localhost:6379
```

### 3. Docker Deployment (Recommended)

```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f ragbot-api

# Stop services
docker-compose down
```

### 4. Local Development

```bash
# Install dependencies
cd ragbot-api
pip install -r requirements.txt

# Run the API server
python main.py
```

The API will be available at `http://localhost:8000`

## 📖 API Documentation

Once the server is running, access the interactive API documentation:

- **Swagger UI**: `http://localhost:8000/docs`
- **ReDoc**: `http://localhost:8000/redoc`

### Key Endpoints

| Endpoint | Method | Description |
|----------|--------|--------------|
| `/` | GET | Service information and health |
| `/api/chat` | POST | Chat with AI assistant about robotics topics |
| `/api/ingest` | POST | Upload and process documents |
| `/api/config` | GET | Get public configuration |
| `/api/health` | GET | Health check endpoint |

## 🔧 Configuration

The application uses environment variables for configuration. Key settings include:

- `OPENAI_MODEL`: LLM model to use (default: groq models)
- `CHUNK_SIZE`: Document chunking size for processing
- `TOP_K_RESULTS`: Number of relevant documents to retrieve
- `RATE_LIMIT_CALLS`: API rate limiting configuration

## 📁 Project Structure

```
ragbot-api/
├── routers/
│   ├── chat.py          # Chat endpoint handlers
│   └── ingest.py        # Document ingestion endpoints
├── utils/
│   └── embeddings.py    # Embedding utilities
├── main.py              # FastAPI application entry point
├── config.py            # Configuration management
├── db.py                # Database initialization
├── qdrant_service.py    # Qdrant vector database service
├── advanced_rag.py      # RAG implementation
└── requirements.txt     # Python dependencies
```

## 🧪 Testing

```bash
# Run API tests
cd ragbot-api
python test_api.py

# Test specific endpoints
curl -X GET http://localhost:8000/api/health
```

## 🚀 Deployment

### Production Deployment

1. **Environment Variables**: Set all required environment variables
2. **Database**: Configure PostgreSQL database
3. **Vector Database**: Set up Qdrant Cloud or self-hosted instance
4. **Reverse Proxy**: Configure Nginx or similar for production
5. **SSL**: Enable HTTPS with proper certificates

### Scaling Considerations

- Use Redis for session management and caching
- Implement horizontal scaling with load balancers
- Monitor performance with APM tools
- Set up proper logging and monitoring

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

If you encounter any issues or have questions:

1. Check the [documentation](docs/)
2. Search existing [issues](../../issues)
3. Create a new issue with detailed information

## 🙏 Acknowledgments

- **FastAPI** - Modern, fast web framework for building APIs
- **Qdrant** - Vector similarity search engine
- **Groq** - High-performance LLM inference
- **Sentence Transformers** - State-of-the-art text embeddings

---

**Built with ❤️ for the Physical AI & Humanoid Robotics Learning Community**

🎓 **Learn • Chat • Build • Deploy Humanoid Robots**
