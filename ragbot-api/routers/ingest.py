"""
Document ingestion router - handles document processing and embedding
"""
from fastapi import APIRouter, BackgroundTasks, HTTPException
from pydantic import BaseModel
import os
import re
from pathlib import Path
from typing import Optional
import json
from datetime import datetime

from config import settings
from embeddings import generate_embeddings_batch
from qdrant_service import store_vectors, create_collection_if_not_exists
from db import IngestionLog, SessionLocal

router = APIRouter(prefix="/api", tags=["ingest"])

DOCS_PATH = "/workspaces/hacks022/docs"


class IngestionRequest(BaseModel):
    """Ingestion request model"""
    force_reingest: bool = False


def extract_sections_from_markdown(content: str, filename: str) -> list:
    """Extract sections from markdown content"""
    sections = []
    current_section = ""
    current_header = ""
    
    lines = content.split("\n")
    
    for line in lines:
        # Detect headers
        if line.startswith("#"):
            header_level = len(re.match(r'^#+', line).group())
            header_text = line.replace("#", "").strip()
            current_header = header_text
        
        current_section += line + "\n"
        
        # Split by headers or when section gets long
        if len(current_section) > settings.chunk_size:
            if current_section.strip():
                sections.append({
                    "filename": filename,
                    "section": current_header or "Introduction",
                    "content": current_section.strip()
                })
            current_section = ""
    
    if current_section.strip():
        sections.append({
            "filename": filename,
            "section": current_header or "Conclusion",
            "content": current_section.strip()
        })
    
    return sections


def extract_module_from_path(file_path: str) -> str:
    """Extract module name from file path"""
    parts = file_path.split("/")
    for part in parts:
        if part.startswith("module"):
            return part
    return "general"


def process_markdown_files(file_path: str = DOCS_PATH) -> tuple:
    """
    Process all markdown files in docs folder and return chunks
    
    Returns:
        Tuple of (chunks, metadata_list, total_files)
    """
    chunks = []
    metadata_list = []
    total_files = 0
    
    # Get all markdown files
    md_files = []
    for root, dirs, files in os.walk(file_path):
        for file in files:
            if file.endswith((".md", ".mdx")):
                md_files.append(os.path.join(root, file))
    
    print(f"Found {len(md_files)} markdown files")
    
    for file_path in md_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            relative_path = os.path.relpath(file_path, DOCS_PATH)
            module = extract_module_from_path(relative_path)
            
            # Extract sections
            sections = extract_sections_from_markdown(content, relative_path)
            
            for section in sections:
                chunk_id = f"{relative_path}#{section['section']}"
                
                metadata_list.append({
                    "chunk_id": chunk_id,
                    "filename": section['filename'],
                    "module": module,
                    "section": section['section'],
                    "content": section['content'],
                    "full_path": relative_path,
                    "created_at": datetime.utcnow().isoformat()
                })
                
                chunks.append(section['content'])
            
            total_files += 1
            print(f"✓ Processed {relative_path} ({len(sections)} sections)")
            
        except Exception as e:
            print(f"✗ Error processing {file_path}: {e}")
    
    return chunks, metadata_list, total_files


@router.post("/ingest/docs")
async def ingest_docs(
    request: IngestionRequest,
    background_tasks: BackgroundTasks
):
    """
    Ingest all documentation into Qdrant
    Supports full-book RAG mode
    """
    try:
        # Create collection if needed
        create_collection_if_not_exists()
        
        # Process markdown files
        print("🔄 Processing markdown files...")
        chunks, metadata_list, total_files = process_markdown_files(DOCS_PATH)
        
        if not chunks:
            raise HTTPException(status_code=400, detail="No markdown files found")
        
        print(f"📊 Generated {len(chunks)} chunks from {total_files} files")
        
        # Generate embeddings
        print("🔄 Generating embeddings...")
        embeddings = generate_embeddings_batch(chunks)
        
        # Store in Qdrant
        print("🔄 Storing vectors in Qdrant...")
        vectors_stored, point_ids = store_vectors(embeddings, metadata_list)
        
        # Log ingestion
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
        finally:
            db.close()
        
        return {
            "status": "success",
            "message": "Documents ingested successfully",
            "files_processed": total_files,
            "chunks_created": len(chunks),
            "vectors_stored": vectors_stored,
            "point_ids": point_ids[:10]  # Return first 10 as sample
        }
        
    except Exception as e:
        print(f"✗ Ingestion error: {e}")
        
        # Log failure
        db = SessionLocal()
        try:
            ingestion_log = IngestionLog(
                doc_path=DOCS_PATH,
                chunks_created=0,
                vectors_stored=0,
                status="failed",
                error_message=str(e)
            )
            db.add(ingestion_log)
            db.commit()
        finally:
            db.close()
        
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


@router.post("/ingest/reset")
async def reset_ingestion():
    """Reset all ingestion (delete collection)"""
    try:
        from qdrant_service import delete_collection
        delete_collection()
        return {"status": "success", "message": "Collection reset"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Reset failed: {str(e)}")


@router.get("/ingest/status")
async def get_ingestion_status():
    """Get ingestion status"""
    try:
        from qdrant_service import get_collection_info
        info = get_collection_info()
        return {"status": "success", "collection": info}
    except Exception as e:
        return {
            "status": "error",
            "message": f"Collection not initialized: {str(e)}"
        }
