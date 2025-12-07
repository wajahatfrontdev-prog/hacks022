"""
Utility functions for RAG Chatbot
"""
import json
import logging
from datetime import datetime
from typing import List, Dict, Any

logger = logging.getLogger(__name__)


def setup_logging(level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('ragbot.log'),
            logging.StreamHandler()
        ]
    )


def format_sources(sources: List[Dict]) -> str:
    """Format sources for display"""
    if not sources:
        return "No sources found"
    
    formatted = "\n\n📚 **Sources:**\n"
    for i, source in enumerate(sources, 1):
        formatted += f"\n{i}. **{source.get('filename', 'Unknown')}**"
        if source.get('module'):
            formatted += f" (Module: {source['module']})"
        if source.get('section'):
            formatted += f"\n   Section: {source['section']}"
        if source.get('score'):
            formatted += f"\n   Relevance: {source['score']*100:.0f}%"
    
    return formatted


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 100) -> List[str]:
    """Split text into chunks with overlap"""
    chunks = []
    words = text.split()
    
    current_chunk = []
    current_size = 0
    
    for word in words:
        current_chunk.append(word)
        current_size += len(word) + 1
        
        if current_size >= chunk_size:
            chunks.append(' '.join(current_chunk))
            # Keep last words for overlap
            overlap_count = min(overlap // 6, len(current_chunk))  # ~6 chars per word
            current_chunk = current_chunk[-overlap_count:]
            current_size = sum(len(w) + 1 for w in current_chunk)
    
    # Add remaining chunk
    if current_chunk:
        chunks.append(' '.join(current_chunk))
    
    return chunks


def sanitize_input(text: str, max_length: int = 2000) -> str:
    """Sanitize user input"""
    if not text:
        return ""
    
    # Remove leading/trailing whitespace
    text = text.strip()
    
    # Limit length
    if len(text) > max_length:
        text = text[:max_length]
    
    # Remove potential injection attempts
    dangerous_chars = ['<', '>', '{', '}', '`']
    for char in dangerous_chars:
        text = text.replace(char, '')
    
    return text


def estimate_tokens(text: str) -> int:
    """Rough estimate of tokens in text"""
    # OpenAI uses ~4 characters per token
    return len(text) // 4


def format_timestamp(dt: datetime) -> str:
    """Format datetime for display"""
    return dt.strftime('%Y-%m-%d %H:%M:%S UTC')


def json_to_dict(json_str: str) -> Dict[str, Any]:
    """Safely parse JSON string"""
    try:
        return json.loads(json_str) if json_str else {}
    except json.JSONDecodeError:
        logger.warning(f"Failed to parse JSON: {json_str}")
        return {}


def dict_to_json(data: Dict[str, Any]) -> str:
    """Convert dict to JSON string"""
    return json.dumps(data)


class Timer:
    """Simple timer context manager"""
    
    def __init__(self, name: str = "Operation"):
        self.name = name
        self.start = None
        self.end = None
    
    def __enter__(self):
        self.start = datetime.utcnow()
        return self
    
    def __exit__(self, *args):
        self.end = datetime.utcnow()
        duration = (self.end - self.start).total_seconds()
        logger.info(f"{self.name} took {duration:.2f}s")
    
    @property
    def elapsed(self) -> float:
        if self.start and self.end:
            return (self.end - self.start).total_seconds()
        return 0
