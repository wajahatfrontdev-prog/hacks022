"""
Standalone ingestion script to populate Qdrant with documents
Run this script to ingest all documentation before starting the API server
"""

import sys
import os

# Ensure the repository root is on sys.path so we can import the `ragbot_api` package
repo_root = os.path.abspath(os.path.dirname(__file__))
# Add the `ragbot-api` package directory to path so imports work the same way
ragbot_api_path = os.path.join(repo_root, 'ragbot-api')
sys.path.insert(0, ragbot_api_path)

# Import modules from the backend package (same style as running inside ragbot-api)
from routers.ingest import process_markdown_files, DOCS_PATH
from embeddings import generate_embeddings_batch
from qdrant_service import create_collection_if_not_exists, store_vectors
from db import init_db, SessionLocal, IngestionLog
from datetime import datetime


def main():
    """Main ingestion function"""
    print("=" * 60)
    print("RAG Chatbot Document Ingestion Script")
    print("=" * 60)
    print()
    
    # Initialize database
    print("🔄 Initializing database...")
    init_db()
    print("✓ Database initialized")
    print()
    
    # Create Qdrant collection
    print("🔄 Setting up Qdrant collection...")
    create_collection_if_not_exists()
    print("✓ Qdrant collection ready")
    print()
    
    # Process documents
    print(f"🔄 Processing markdown files from {DOCS_PATH}...")
    chunks, metadata_list, total_files = process_markdown_files(DOCS_PATH)
    
    if not chunks:
        print("✗ No markdown files found!")
        return
    
    print(f"✓ Found {len(chunks)} chunks from {total_files} files")
    print()
    
    # Generate embeddings
    print("🔄 Generating embeddings (this may take a minute)...")
    try:
        embeddings = generate_embeddings_batch(chunks, batch_size=25)
        print(f"✓ Generated {len(embeddings)} embeddings")
    except Exception as e:
        print(f"✗ Embedding generation failed: {e}")
        return
    
    print()
    
    # Store in Qdrant
    print("🔄 Storing vectors in Qdrant...")
    try:
        vectors_stored, point_ids = store_vectors(embeddings, metadata_list)
        print(f"✓ Successfully stored {vectors_stored} vectors")
    except Exception as e:
        print(f"✗ Vector storage failed: {e}")
        return
    
    print()
    
    # Log to database
    db = SessionLocal()
    try:
        ingestion_log = IngestionLog(
            doc_path=DOCS_PATH,
            chunks_created=len(chunks),
            vectors_stored=vectors_stored,
            status="success"
        )
        db.add(ingestion_log)
        db.commit()
        print("✓ Ingestion logged to database")
    finally:
        db.close()
    
    print()
    print("=" * 60)
    print("✓ INGESTION COMPLETE!")
    print("=" * 60)
    print()
    print("Summary:")
    print(f"  • Files processed: {total_files}")
    print(f"  • Chunks created: {len(chunks)}")
    print(f"  • Vectors stored: {vectors_stored}")
    print(f"  • Sample point IDs: {point_ids[:5]}")
    print()
    print("Next steps:")
    print("  1. Start the FastAPI server: python -m uvicorn ragbot_api.main:app --reload")
    print("  2. Test the API: http://localhost:8000/docs")
    print("  3. Deploy the frontend to Vercel")
    print()


if __name__ == "__main__":
    main()
