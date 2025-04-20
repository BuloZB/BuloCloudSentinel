import os
import httpx
from typing import Optional, Dict, Any

class WhisperAdapter:
    def __init__(self):
        self.api_key = os.getenv("WHISPER_API_KEY")
        if not self.api_key:
            raise ValueError("WHISPER_API_KEY not set in environment")
        self.base_url = "https://api.openai.com/v1/audio/transcriptions"

    async def transcribe_audio(self, audio_bytes: bytes) -> str:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "audio/wav"
        }
        async with httpx.AsyncClient(timeout=60) as client:
            response = await client.post(self.base_url, content=audio_bytes, headers=headers)
            response.raise_for_status()
            result = response.json()
            return result.get("text", "")

    async def status(self) -> Dict[str, Any]:
        # Simplified status check
        return {"name": "Whisper", "available": True}
