from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from groq import Groq
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    query: str
    mode: str = "fullbook"
    selected_text: str = None
    session_id: str = None
    user_id: str = "anonymous"

@app.post("/api/chat")
async def chat(request: ChatRequest):
    try:
        client = Groq(api_key=os.getenv("GROQ_API_KEY"))
        
        response = client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[
                {"role": "system", "content": "You are a helpful AI assistant for Physical AI Humanoid Robotics."},
                {"role": "user", "content": request.query}
            ],
            temperature=0.7,
            max_tokens=1000
        )
        
        return {
            "response": response.choices[0].message.content,
            "session_id": request.session_id or "default",
            "sources": [],
            "tokens_used": response.usage.total_tokens,
            "mode": request.mode
        }
    except Exception as e:
        return {"response": f"Error: {str(e)}", "session_id": "error", "sources": [], "tokens_used": 0, "mode": request.mode}

@app.get("/api/health")
async def health():
    return {"status": "healthy"}
