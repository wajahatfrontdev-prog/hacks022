import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / 'ragbot-api'))

os.environ.setdefault('GROQ_API_KEY', os.getenv('GROQ_API_KEY', ''))
os.environ.setdefault('QDRANT_URL', os.getenv('QDRANT_URL', ''))
os.environ.setdefault('QDRANT_API_KEY', os.getenv('QDRANT_API_KEY', ''))
os.environ.setdefault('DATABASE_URL', os.getenv('DATABASE_URL', 'sqlite:///./chat.db'))
os.environ.setdefault('FRONTEND_URL', os.getenv('FRONTEND_URL', 'https://hacks022.vercel.app'))
os.environ.setdefault('EMBEDDINGS_PROVIDER', os.getenv('EMBEDDINGS_PROVIDER', 'hf'))
os.environ.setdefault('HUGGINGFACE_API_KEY', os.getenv('HUGGINGFACE_API_KEY', ''))

from main import app

app = app
