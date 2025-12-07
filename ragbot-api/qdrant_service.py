"""
Qdrant service wrapper (avoid module name collision with installed package)
"""
import sys
import os
import site
import importlib

# Ensure we import the installed `qdrant_client` package from site-packages
# and not the local `qdrant_client.py` file which may exist in this folder.
_this_dir = os.path.dirname(__file__)

# Build candidate site-packages locations
_site_candidates = []
try:
    _site_candidates.extend(site.getsitepackages())
except Exception:
    pass
try:
    _site_candidates.append(site.getusersitepackages())
except Exception:
    pass

# Also consider common venv-relative locations (handles lib and lib64)
_venv_base = os.path.join(_this_dir, "venv")
for _libdir in ("lib", "lib64"):
    _path = os.path.join(_venv_base, _libdir)
    if os.path.isdir(_path):
        for entry in os.listdir(_path):
            if entry.startswith("python"):
                _site_candidates.append(os.path.join(_path, entry, "site-packages"))

# Deduplicate while preserving order
_seen = set()
_site_candidates_clean = []
for p in _site_candidates:
    if not p:
        continue
    p_abs = os.path.abspath(p)
    if p_abs in _seen:
        continue
    _seen.add(p_abs)
    _site_candidates_clean.append(p_abs)

_installed_path = None
for p in _site_candidates_clean:
    if os.path.isdir(os.path.join(p, "qdrant_client")):
        _installed_path = p
        break

_sys_inserted = False
if _installed_path:
    if _installed_path not in sys.path:
        sys.path.insert(0, _installed_path)
        _sys_inserted = True

# Import the installed qdrant_client package (now that site-packages path is preferred)
try:
    import qdrant_client as _qdrant_pkg
    from qdrant_client.http.models import Distance, VectorParams, PointStruct
except Exception:
    # Clean up path mutation if import failed
    if _sys_inserted:
        try:
            sys.path.remove(_installed_path)
        except Exception:
            pass
    raise
finally:
    # Restore original sys.path ordering by removing the inserted site-packages path
    if _sys_inserted:
        try:
            sys.path.remove(_installed_path)
        except Exception:
            pass
from config import settings
from typing import List, Dict, Tuple
import json
import time

# Initialize Qdrant client
client = _qdrant_pkg.QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
    timeout=30
)


def create_collection_if_not_exists():
    """Create Qdrant collection if it doesn't exist (or recreate with correct dimensions)"""
    try:
        # Check if collection exists
        collections = client.get_collections()
        coll_names = [c.name for c in collections.collections]
        
        if settings.qdrant_collection_name in coll_names:
            # Check if dimensions match; if not, delete and recreate
            coll_info = client.get_collection(settings.qdrant_collection_name)
            if coll_info.config.params.vectors.size != settings.qdrant_vector_size:
                print(f"⚠ Collection has wrong dimension ({coll_info.config.params.vectors.size} vs expected {settings.qdrant_vector_size}). Recreating...")
                client.delete_collection(settings.qdrant_collection_name)
                print(f"Creating Qdrant collection: {settings.qdrant_collection_name} with dim {settings.qdrant_vector_size}")
                client.create_collection(
                    collection_name=settings.qdrant_collection_name,
                    vectors_config=VectorParams(
                        size=settings.qdrant_vector_size,
                        distance=Distance.COSINE
                    )
                )
                print(f"✓ Qdrant collection created successfully")
            else:
                print(f"✓ Collection {settings.qdrant_collection_name} already exists with correct dimension")
        else:
            print(f"Creating Qdrant collection: {settings.qdrant_collection_name}")
            client.create_collection(
                collection_name=settings.qdrant_collection_name,
                vectors_config=VectorParams(
                    size=settings.qdrant_vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"✓ Qdrant collection created successfully")
    except Exception as e:
        print(f"✗ Error creating collection: {e}")
        raise


def store_vectors(
    vectors: List[List[float]],
    metadata: List[Dict],
    batch_size: int = 100
) -> Tuple[int, List[str]]:
    """
    Store vectors in Qdrant with metadata
    """
    try:
        points = []
        point_ids = []

        for idx, (vector, meta) in enumerate(zip(vectors, metadata)):
            point_id = int(time.time() * 1000000) + idx
            point_ids.append(str(point_id))

            points.append(
                PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "chunk_id": meta.get("chunk_id", f"chunk_{idx}"),
                        "filename": meta.get("filename", ""),
                        "module": meta.get("module", ""),
                        "section": meta.get("section", ""),
                        "content": meta.get("content", "")[:1000],
                        "full_path": meta.get("full_path", ""),
                        "created_at": meta.get("created_at", "")
                    }
                )
            )

            # Upload in batches
            if len(points) >= batch_size:
                client.upsert(
                    collection_name=settings.qdrant_collection_name,
                    points=points
                )
                points = []

        # Upload remaining points
        if points:
            client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=points
            )

        total_stored = len(metadata)
        print(f"✓ Stored {total_stored} vectors in Qdrant")
        return total_stored, point_ids

    except Exception as e:
        print(f"✗ Error storing vectors: {e}")
        raise


def search_vectors(
    query_vector: List[float],
    top_k: int = 5,
    score_threshold: float = 0.5
) -> List[Dict]:
    """
    Search for similar vectors in Qdrant using query_points
    """
    try:
        # Use query_points which accepts a vector directly
        results = client.query_points(
            collection_name=settings.qdrant_collection_name,
            query=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            with_payload=True,
            with_vectors=False
        )
        
        formatted_results = []
        for result in results.points:
            payload = result.payload if hasattr(result, 'payload') else {}
            score = result.score if hasattr(result, 'score') else 0.0

            formatted_results.append({
                "score": score,
                "filename": payload.get("filename", ""),
                "module": payload.get("module", ""),
                "section": payload.get("section", ""),
                "content": payload.get("content", ""),
                "full_path": payload.get("full_path", "")
            })
        
        return formatted_results
        
    except Exception as e:
        print(f"✗ Error searching vectors: {e}")
        raise


def get_collection_info() -> Dict:
    """Get collection information"""
    try:
        collection = client.get_collection(settings.qdrant_collection_name)
        return {
            "name": settings.qdrant_collection_name,
            "point_count": collection.points_count,
            "vector_size": settings.qdrant_vector_size
        }
    except Exception as e:
        print(f"✗ Error getting collection info: {e}")
        raise


def delete_collection():
    """Delete the collection (useful for reset)"""
    try:
        client.delete_collection(settings.qdrant_collection_name)
        print(f"✓ Collection {settings.qdrant_collection_name} deleted")
    except Exception as e:
        print(f"✗ Error deleting collection: {e}")
