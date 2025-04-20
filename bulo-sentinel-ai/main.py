from fastapi import FastAPI, UploadFile, File, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, Dict, Any
import os
import httpx
import asyncio
from fastapi import FastAPI, UploadFile, File, HTTPException, Depends, Response
from backend.api.dependencies import get_current_user
from bulo_sentinel_ai.audit_log import log_audit_entry
from bulo_sentinel_ai.monitoring import router as monitoring_router

app = FastAPI(
    title="Bulo.Cloud Sentinel AI Integrations",
    description="AI integrations microservice for Bulo.Cloud Sentinel",
    version="0.1.0"
)

# Base interface for AI tool adapters
class AIAdapter:
    async def chat(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> str:
        raise NotImplementedError()

    async def analyze_image(self, image_bytes: bytes) -> Dict[str, Any]:
        raise NotImplementedError()

    async def transcribe_audio(self, audio_bytes: bytes) -> str:
        raise NotImplementedError()

    async def status(self) -> Dict[str, Any]:
        raise NotImplementedError()

# ChatGPT Adapter example
class ChatGPTAdapter(AIAdapter):
    def __init__(self):
        self.api_key = os.getenv("CHATGPT_API_KEY")
        if not self.api_key:
            raise ValueError("CHATGPT_API_KEY not set in environment")

    async def chat(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> str:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        data = {
            "model": "gpt-4",
            "messages": [{"role": "user", "content": prompt}]
        }
        async with httpx.AsyncClient(timeout=30) as client:
            response = await client.post("https://api.openai.com/v1/chat/completions", json=data, headers=headers)
            response.raise_for_status()
            result = response.json()
            return result["choices"][0]["message"]["content"]

    async def status(self) -> Dict[str, Any]:
        # Simplified status check
        return {"name": "ChatGPT", "available": True}

from bulo_sentinel_ai.claude_adapter import ClaudeAdapter

from bulo_sentinel_ai.gemini_adapter import GeminiAdapter

from bulo_sentinel_ai.dalle_adapter import DalleAdapter

from bulo_sentinel_ai.whisper_adapter import WhisperAdapter

# Instantiate adapters
adapters = {
    "chatgpt": ChatGPTAdapter(),
    "claude": ClaudeAdapter(),
    "gemini": GeminiAdapter(),
    "dalle": DalleAdapter(),
    "whisper": WhisperAdapter(),
}

# Audit log store (in-memory for demo)
audit_log = []

class ChatRequest(BaseModel):
    tool: str
    prompt: str
    context: Optional[Dict[str, Any]] = None

@app.post("/ai/chat")
async def ai_chat(request: ChatRequest, current_user: str = Depends(get_current_user)):
    if request.tool not in adapters:
        raise HTTPException(status_code=400, detail="Unsupported AI tool")
    adapter = adapters[request.tool]
    try:
        response = await adapter.chat(request.prompt, request.context)
        audit_entry = {
            "user": current_user,
            "tool": request.tool,
            "prompt": request.prompt,
            "response": response,
            "timestamp": asyncio.get_event_loop().time()
        }
        audit_log.append(audit_entry)
        log_audit_entry(audit_entry)
        return {"response": response}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/ai/status")
async def ai_status():
    statuses = {}
    for name, adapter in adapters.items():
        statuses[name] = await adapter.status()
    return statuses

# Placeholder endpoints for vision and audio
@app.post("/ai/vision/analyze")
async def ai_vision_analyze(file: UploadFile = File(...), current_user: str = Depends(get_current_user)):
    # For demo, just return file info
    content = await file.read()
    return {"filename": file.filename, "size": len(content)}

@app.post("/ai/audio/transcribe")
async def ai_audio_transcribe(file: UploadFile = File(...), current_user: str = Depends(get_current_user)):
    # For demo, just return file info
    content = await file.read()
    return {"filename": file.filename, "size": len(content)}

app.include_router(monitoring_router)
