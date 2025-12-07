"""
Advanced RAG utilities for chunking, retrieval, and processing
"""
from typing import List, Tuple, Dict
import re
from pathlib import Path


class AdvancedChunker:
    """Advanced document chunking with semantic awareness"""
    
    def __init__(self, chunk_size: int = 500, overlap: int = 100):
        self.chunk_size = chunk_size
        self.overlap = overlap
    
    def chunk_by_sections(self, text: str, filename: str = "") -> List[Dict]:
        """Chunk text by markdown sections"""
        sections = []
        current_section = None
        current_content = []
        
        lines = text.split('\n')
        
        for line in lines:
            # Detect headers
            if line.startswith('#'):
                # Save previous section
                if current_section is not None:
                    content = '\n'.join(current_content).strip()
                    if len(content) > 50:  # Minimum content length
                        sections.append({
                            'header': current_section,
                            'content': content,
                            'filename': filename
                        })
                
                # Start new section
                header_level = len(re.match(r'^#+', line).group())
                current_section = line.replace('#', '').strip()
                current_content = []
            else:
                if current_section is not None:
                    current_content.append(line)
        
        # Save last section
        if current_section is not None:
            content = '\n'.join(current_content).strip()
            if len(content) > 50:
                sections.append({
                    'header': current_section,
                    'content': content,
                    'filename': filename
                })
        
        return sections
    
    def smart_split(self, text: str) -> List[str]:
        """Intelligently split text preserving context"""
        # Split by sentences first
        sentences = re.split(r'(?<=[.!?])\s+', text)
        
        chunks = []
        current_chunk = []
        current_size = 0
        
        for sentence in sentences:
            current_chunk.append(sentence)
            current_size += len(sentence)
            
            if current_size >= self.chunk_size:
                chunks.append(' '.join(current_chunk))
                # Keep last 1-2 sentences for context
                current_chunk = current_chunk[-2:]
                current_size = sum(len(s) for s in current_chunk)
        
        if current_chunk:
            chunks.append(' '.join(current_chunk))
        
        return chunks


class MetadataExtractor:
    """Extract metadata from documents"""
    
    @staticmethod
    def extract_from_path(file_path: str) -> Dict[str, str]:
        """Extract module and section info from file path"""
        path = Path(file_path)
        parts = path.parts
        
        metadata = {
            'filename': path.name,
            'full_path': file_path,
            'module': 'general',
            'extension': path.suffix
        }
        
        # Extract module name
        for part in parts:
            if part.startswith('module'):
                metadata['module'] = part
                break
        
        return metadata
    
    @staticmethod
    def extract_headers(text: str) -> List[str]:
        """Extract all headers from text"""
        headers = re.findall(r'^#+\s+(.+)$', text, re.MULTILINE)
        return headers


class RetrievalEnhancer:
    """Enhance retrieval results with re-ranking and filtering"""
    
    @staticmethod
    def filter_by_relevance(results: List[Dict], threshold: float = 0.5) -> List[Dict]:
        """Filter results by relevance score"""
        return [r for r in results if r.get('score', 0) >= threshold]
    
    @staticmethod
    def deduplicate_results(results: List[Dict]) -> List[Dict]:
        """Remove duplicate results"""
        seen = set()
        unique = []
        
        for result in results:
            key = f"{result.get('filename')}#{result.get('section')}"
            if key not in seen:
                seen.add(key)
                unique.append(result)
        
        return unique
    
    @staticmethod
    def rerank_by_recency(results: List[Dict]) -> List[Dict]:
        """Re-rank results by creation date"""
        return sorted(results, key=lambda x: x.get('created_at', ''), reverse=True)
    
    @staticmethod
    def diversity_rank(results: List[Dict]) -> List[Dict]:
        """Rank to maximize source diversity"""
        # Prefer results from different modules
        seen_modules = set()
        ranked = []
        
        # First pass: one from each module
        for result in results:
            module = result.get('module', 'unknown')
            if module not in seen_modules:
                ranked.append(result)
                seen_modules.add(module)
        
        # Second pass: add remaining
        ranked.extend([r for r in results if r not in ranked])
        
        return ranked


class QueryProcessor:
    """Process and enhance user queries"""
    
    @staticmethod
    def expand_query(query: str) -> List[str]:
        """Generate related queries for multi-round search"""
        # Simple expansion - in production, use GPT-3 or similar
        expansions = [query]
        
        # Add question variations
        if '?' in query:
            expansion = query.replace('?', '').strip()
            expansions.append(expansion)
        
        # Add keyword extraction
        important_words = [w for w in query.split() if len(w) > 4]
        if important_words:
            expansions.append(' '.join(important_words))
        
        return list(set(expansions))  # Remove duplicates
    
    @staticmethod
    def extract_keywords(query: str) -> List[str]:
        """Extract key terms from query"""
        # Remove common words
        stop_words = {'what', 'is', 'the', 'a', 'an', 'how', 'why', 'when', 'where', 'who'}
        words = [w.lower() for w in query.split() if len(w) > 3]
        keywords = [w for w in words if w not in stop_words]
        return keywords


class CacheManager:
    """Simple caching for embeddings and results"""
    
    def __init__(self):
        self.cache = {}
    
    def get(self, key: str):
        """Get from cache"""
        return self.cache.get(key)
    
    def set(self, key: str, value: any, ttl: int = 3600):
        """Set in cache with TTL"""
        # In production, use Redis
        self.cache[key] = {
            'value': value,
            'ttl': ttl,
            'timestamp': self._current_time()
        }
    
    def clear(self):
        """Clear cache"""
        self.cache.clear()
    
    @staticmethod
    def _current_time() -> float:
        import time
        return time.time()
