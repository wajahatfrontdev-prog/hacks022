"""
Chat router - handles RAG chatbot endpoints using Groq for fast LLM inference
"""
from fastapi import APIRouter, Depends, HTTPException, BackgroundTasks
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional, List
import json
from groq import Groq
from datetime import datetime
import uuid

from config import settings
from db import ChatMessage, UserSession, get_db
from embeddings import generate_query_embedding
from qdrant_service import search_vectors

router = APIRouter(prefix="/api", tags=["chat"])


def get_groq_client():
    """Lazy-load Groq client"""
    return Groq(api_key=settings.groq_api_key)


class ChatRequest(BaseModel):
    """Chat request model"""
    query: str
    mode: str = "fullbook"  # "fullbook" or "selected"
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    user_id: Optional[str] = "anonymous"


class ChatResponse(BaseModel):
    """Chat response model"""
    response: str
    session_id: str
    sources: List[dict]
    tokens_used: int
    mode: str


def get_or_create_session(user_id: str, db: Session) -> str:
    """Get or create a user session"""
    session = db.query(UserSession).filter(
        UserSession.user_id == user_id
    ).order_by(UserSession.created_at.desc()).first()
    
    if not session:
        session = UserSession(
            user_id=user_id,
            session_metadata=json.dumps({"created_at": datetime.utcnow().isoformat()})
        )
        db.add(session)
        db.commit()
        db.refresh(session)
    
    return session.id


def is_greeting_or_general(query: str) -> bool:
    """Check if query is a greeting or general conversation"""
    greetings = [
        'hello', 'hi', 'hey', 'salam', 'assalam', 'salaam', 
        'halo', 'howdy', 'good morning', 'good afternoon', 'good evening',
        'what is your name', 'who are you', 'how are you', 'whats up',
        'thanks', 'thank you', 'welcome', 'bye', 'goodbye', 'take care'
    ]
    query_lower = query.lower().strip()
    return any(greeting in query_lower for greeting in greetings)


def build_system_prompt(include_general: bool = False) -> str:
    """Build system prompt for the agent"""
    if include_general:
        return """You are a friendly and knowledgeable AI assistant for the Physical AI Humanoid Robotics Book.
    Your role is to:
    1. Have friendly conversations and answer general questions
    2. When asked about robotics, ROS2, digital twins, NVIDIA Isaac, or VLA - provide expert information from the book
    3. Be helpful, conversational, and maintain a professional yet friendly tone
    4. You can answer general knowledge questions about robotics and AI
    5. Always be honest about what you know and don't know
    6. Keep responses concise and engaging"""
    else:
        return """You are an expert AI assistant for the Physical AI Humanoid Robotics Book. 
    Your role is to:
    1. Answer questions about robotics, ROS2, digital twins, NVIDIA Isaac, and VLA (Vision Language Actions)
    2. Provide detailed, accurate information from the book content
    3. Include module and section references in your answers
    4. Be helpful, accurate, and concise
    5. If information is not in the provided context, clearly state that
    6. Always maintain a professional and educational tone"""


def build_user_prompt(query: str, context: str, sources: List[dict]) -> str:
    """Build user prompt with context"""
    source_text = "\n".join([
        f"- {s['filename']} (Module: {s['module']}, Section: {s['section']}): {s['content'][:300]}..."
        for s in sources
    ])

    return f"""User Query: {query}

Context from the book (use ONLY this information):
{source_text}

Instructions:
- Answer based ONLY on the provided context above
- Include specific module and section references for all information
- If the context doesn't contain the answer, say "Based on the book content, I cannot provide information about this topic"
- Be precise and cite sources for every claim
- Keep answers focused and educational"""


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    """
    Main chat endpoint supporting two modes:
    - fullbook: RAG search + Groq agent answer
    - selected: Answer only from user-selected text
    - general: General conversation without RAG
    """
    
    try:
        # Get or create session
        if not request.session_id:
            request.session_id = get_or_create_session(request.user_id, db)
        
        sources = []
        bot_response = ""
        tokens_used = 0
        
        # Check if this is a greeting or general question
        is_greeting = is_greeting_or_general(request.query)
        
        # MODE 1: FULL-BOOK RAG
        if request.mode == "fullbook":
            # If it's a greeting or general question, respond conversationally without RAG
            if is_greeting:
                system_prompt = build_system_prompt(include_general=True)
                user_prompt = f"User: {request.query}"
                
                client = get_groq_client()
                response = client.chat.completions.create(
                    model=settings.groq_model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=0.8,
                    max_tokens=500,
                    top_p=0.95
                )
                
                bot_response = response.choices[0].message.content
                tokens_used = response.usage.total_tokens
                sources = []  # No book sources for greetings
            
            else:
                # Attempt normal RAG flow (embeddings + Qdrant + Groq answer)
                try:
                    # Generate query embedding
                    query_embedding = generate_query_embedding(request.query)
                    
                    # Search Qdrant
                    search_results = search_vectors(
                        query_embedding,
                        top_k=settings.top_k_results,
                        score_threshold=0.3
                    )
                    
                    if not search_results:
                        # If no book results, try general knowledge
                        system_prompt = build_system_prompt(include_general=True)
                        user_prompt = f"User Query: {request.query}\n\nNote: I didn't find this topic in the robotics book, but I can provide general information about it."
                        
                        client = get_groq_client()
                        response = client.chat.completions.create(
                            model=settings.groq_model,
                            messages=[
                                {"role": "system", "content": system_prompt},
                                {"role": "user", "content": user_prompt}
                            ],
                            temperature=0.7,
                            max_tokens=1000,
                            top_p=0.95
                        )
                        
                        bot_response = response.choices[0].message.content
                        tokens_used = response.usage.total_tokens
                        sources = []
                    else:
                        sources = search_results
                        
                        # Build context from search results
                        context = "\n\n".join([
                            f"[{s['module']} - {s['filename']}]\n{s['content']}"
                            for s in search_results
                        ])
                        
                        # Call Groq Agent for fast response
                        system_prompt = build_system_prompt(include_general=False)
                        user_prompt = build_user_prompt(request.query, context, search_results)
                        
                        client = get_groq_client()
                        response = client.chat.completions.create(
                            model=settings.groq_model,
                            messages=[
                                {"role": "system", "content": system_prompt},
                                {"role": "user", "content": user_prompt}
                            ],
                            temperature=0.7,
                            max_tokens=1000,
                            top_p=0.95
                        )
                        
                        bot_response = response.choices[0].message.content
                        tokens_used = response.usage.total_tokens
                
                except Exception as rag_error:
                    # If RAG pipeline fails, fall back to general knowledge
                    print(f"⚠ RAG pipeline failed: {rag_error}. Using general knowledge...")
                    
                    system_prompt = build_system_prompt(include_general=True)
                    user_prompt = f"User Query: {request.query}\n\nNote: The book search is temporarily unavailable, but I can provide general information about this topic."
                    
                    client = get_groq_client()
                    response = client.chat.completions.create(
                        model=settings.groq_model,
                        messages=[
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": user_prompt}
                        ],
                        temperature=0.7,
                        max_tokens=1000,
                        top_p=0.95
                    )
                    
                    bot_response = response.choices[0].message.content
                    tokens_used = response.usage.total_tokens
                    sources = []
        
        # MODE 2: SELECTION-ONLY
        elif request.mode == "selected":
            if not request.selected_text:
                raise HTTPException(status_code=400, detail="selected_text required for selected mode")
            
            # Answer ONLY from selected text using Groq
            system_prompt = build_system_prompt()
            user_prompt = f"""User Query: {request.query}

Selected Text from Book:
{request.selected_text}

Please answer the user's question based ONLY on the selected text above. Do not use any external knowledge."""
            
            client = get_groq_client()
            response = client.chat.completions.create(
                model=settings.groq_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=1000
            )
            
            bot_response = response.choices[0].message.content
            tokens_used = response.usage.total_tokens
        
        else:
            raise HTTPException(status_code=400, detail="Invalid mode. Use 'fullbook' or 'selected'")
        
        # Save to database
        chat_message = ChatMessage(
            session_id=request.session_id,
            user_query=request.query,
            bot_response=bot_response,
            mode=request.mode,
            selected_text=request.selected_text,
            source_sections=json.dumps(sources),
            tokens_used=tokens_used
        )
        db.add(chat_message)
        db.commit()
        
        return ChatResponse(
            response=bot_response,
            session_id=request.session_id,
            sources=sources,
            tokens_used=tokens_used,
            mode=request.mode
        )
        
    except HTTPException:
        raise
    except Exception as e:
        print(f"✗ Chat error: {e}")
        raise HTTPException(status_code=500, detail=f"Chat processing failed: {str(e)}")


@router.get("/chat/history/{session_id}")
async def get_chat_history(session_id: str, db: Session = Depends(get_db)):
    """Get chat history for a session"""
    try:
        messages = db.query(ChatMessage).filter(
            ChatMessage.session_id == session_id
        ).order_by(ChatMessage.created_at.asc()).all()
        
        return {
            "session_id": session_id,
            "messages": [
                {
                    "query": msg.user_query,
                    "response": msg.bot_response,
                    "mode": msg.mode,
                    "sources": json.loads(msg.source_sections) if msg.source_sections else [],
                    "created_at": msg.created_at.isoformat()
                }
                for msg in messages
            ]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve chat history: {str(e)}")


@router.post("/chat/rate")
async def rate_message(
    message_id: str,
    rating: int,
    db: Session = Depends(get_db)
):
    """Rate a chat message (1-5 stars)"""
    try:
        if rating < 1 or rating > 5:
            raise HTTPException(status_code=400, detail="Rating must be between 1 and 5")
        
        message = db.query(ChatMessage).filter(ChatMessage.id == message_id).first()
        if not message:
            raise HTTPException(status_code=404, detail="Message not found")
        
        message.user_rating = rating
        db.commit()
        
        return {"status": "success", "message_id": message_id, "rating": rating}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to rate message: {str(e)}")


@router.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "RAG Chatbot API",
        "version": settings.api_version
    }
