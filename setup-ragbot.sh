#!/bin/bash
# RAG Chatbot Development Setup Script
# Run this to quickly set up the development environment

set -e

echo "================================================"
echo "RAG Chatbot Development Environment Setup"
echo "================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. Check Python
echo -e "${BLUE}[1/6] Checking Python installation...${NC}"
if ! command -v python3 &> /dev/null; then
    echo "✗ Python 3 not found. Please install Python 3.9+"
    exit 1
fi
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
echo -e "${GREEN}✓ Python ${PYTHON_VERSION} found${NC}"
echo ""

# 2. Setup Python venv
echo -e "${BLUE}[2/6] Setting up Python virtual environment...${NC}"
if [ ! -d "ragbot-api/venv" ]; then
    python3 -m venv ragbot-api/venv
    echo -e "${GREEN}✓ Virtual environment created${NC}"
else
    echo -e "${GREEN}✓ Virtual environment already exists${NC}"
fi
source ragbot-api/venv/bin/activate
echo ""

# 3. Install Python dependencies
echo -e "${BLUE}[3/6] Installing Python dependencies...${NC}"
cd ragbot-api
pip install -q --upgrade pip
pip install -q -r requirements.txt
cd ..
echo -e "${GREEN}✓ Python dependencies installed${NC}"
echo ""

# 4. Setup environment variables
echo -e "${BLUE}[4/6] Setting up environment variables...${NC}"
if [ ! -f "ragbot-api/.env" ]; then
    cp ragbot-api/.env.example ragbot-api/.env
    echo -e "${GREEN}✓ Created .env file from template${NC}"
    echo "   ⚠️  Update ragbot-api/.env with your credentials:"
    echo "      - OPENAI_API_KEY"
    echo "      - QDRANT_URL and QDRANT_API_KEY"
    echo "      - DATABASE_URL"
else
    echo -e "${GREEN}✓ .env file already exists${NC}"
fi
echo ""

# 5. Install Node dependencies
echo -e "${BLUE}[5/6] Installing Node.js dependencies...${NC}"
npm install -q
echo -e "${GREEN}✓ Node dependencies installed${NC}"
echo ""

# 6. Summary
echo -e "${BLUE}[6/6] Setup complete!${NC}"
echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "1. Update credentials in ragbot-api/.env:"
echo "   - OPENAI_API_KEY"
echo "   - QDRANT_URL and QDRANT_API_KEY"
echo "   - DATABASE_URL"
echo ""
echo "2. Start backend server:"
echo "   cd ragbot-api"
echo "   source venv/bin/activate"
echo "   python -m uvicorn main:app --reload"
echo ""
echo "3. In another terminal, start Docusaurus:"
echo "   npm start"
echo ""
echo "4. Ingest documents (when ready):"
echo "   python ragbot-ingest.py"
echo ""
echo "5. Access:"
echo "   Frontend: http://localhost:3000"
echo "   RAG Chat: http://localhost:3000/ragbot"
echo "   API Docs: http://localhost:8000/docs"
echo ""
