"""
Embeddings generation using sentence-transformers (local, free alternative)
Groq doesn't provide embeddings, so we use open-source models
"""
from typing import List
import time
import os
import requests

# Embeddings provider: 'local' (sentence-transformers) or 'hf' (Hugging Face Inference API)
EMBEDDINGS_PROVIDER = os.environ.get('EMBEDDINGS_PROVIDER', 'local')

model = None
if EMBEDDINGS_PROVIDER == 'local':
    try:
        from sentence_transformers import SentenceTransformer
        model = SentenceTransformer('all-MiniLM-L6-v2')
    except ImportError:
        print("⚠ sentence-transformers not installed. Using fallback search only.")
        model = None


def get_embeddings_model():
    """Get the embeddings model (lazy load)"""
    global model
    if model is None and EMBEDDINGS_PROVIDER == 'local':
        try:
            from sentence_transformers import SentenceTransformer
            model = SentenceTransformer('all-MiniLM-L6-v2')
        except Exception as e:
            print(f"⚠ Could not load embeddings model: {e}")
            return None
    return model


def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using sentence-transformers
    
    Args:
        text: Text to embed
    
    Returns:
        Embedding vector (or None if model unavailable)
    """
    # Choose provider
    if EMBEDDINGS_PROVIDER == 'local':
        try:
            em = get_embeddings_model()
            if em is None:
                return None

            # Clean and limit text length
            text = text.replace("\n", " ")
            if len(text) > 8000:
                text = text[:8000]

            embedding = em.encode(text)
            return embedding.tolist()

        except Exception as e:
            print(f"✗ Error generating embedding (local): {e}")
            return None

    elif EMBEDDINGS_PROVIDER == 'hf':
        # Use Hugging Face Inference API for embeddings
        hf_token = os.environ.get('HUGGINGFACE_API_KEY') or os.environ.get('HUGGINGFACE_HUB_TOKEN')
        hf_model = os.environ.get('EMBEDDINGS_HF_MODEL', 'sentence-transformers/all-MiniLM-L6-v2')

        if not hf_token:
            print("⚠ HUGGINGFACE_API_KEY not set for hf embeddings provider")
            return None

        url = f"https://api-inference.huggingface.co/embeddings/{hf_model}"
        headers = { 'Authorization': f'Bearer {hf_token}' }
        payload = { 'inputs': text }

        try:
            resp = requests.post(url, headers=headers, json=payload, timeout=15)
            if resp.status_code != 200:
                print(f"✗ HF embeddings API error: {resp.status_code} {resp.text}")
                return None

            result = resp.json()
            # The HF embeddings API returns a dict with 'embedding' or similar
            if isinstance(result, dict) and 'embedding' in result:
                return result['embedding']
            # Some endpoints return list of embeddings
            if isinstance(result, list) and len(result) and isinstance(result[0], (list, float)):
                # single embedding
                return result[0]

            print("✗ Unexpected HF embeddings response shape")
            return None

        except Exception as e:
            print(f"✗ Error calling HF embeddings API: {e}")
            return None

    else:
        print(f"⚠ Unknown EMBEDDINGS_PROVIDER={EMBEDDINGS_PROVIDER}")
        return None


def generate_embeddings_batch(texts: List[str], batch_size: int = 25) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batches using sentence-transformers
    
    Args:
        texts: List of texts to embed
        batch_size: Number of texts per batch
    
    Returns:
        List of embedding vectors
    """
    # For batches we only support local model currently
    em = get_embeddings_model()
    if em is None:
        print("✗ Embeddings model unavailable. Set EMBEDDINGS_PROVIDER=hf and configure HUGGINGFACE_API_KEY, or install sentence-transformers locally.")
        return None
    
    embeddings = []
    
    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        
        try:
            # Clean texts
            batch = [t.replace("\n", " ")[:8000] for t in batch]
            
            batch_embeddings = em.encode(batch)
            embeddings.extend([e.tolist() for e in batch_embeddings])
            
            print(f"✓ Generated embeddings for batch {i//batch_size + 1}")
            
            # Rate limiting (local, so minimal)
            time.sleep(0.05)
            
        except Exception as e:
            print(f"✗ Error generating batch embeddings: {e}")
            raise
    
    return embeddings


def generate_query_embedding(query: str) -> List[float]:
    """
    Generate embedding for a query
    
    Args:
        query: Query text
    
    Returns:
        Query embedding vector
    """
    embedding = generate_embedding(query)
    if embedding is None:
        raise ValueError("Embeddings unavailable. Fallback search will be used.")
    return embedding
