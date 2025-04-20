import os
import httpx
from typing import Optional, Dict, Any

class GeminiAdapter:
    def __init__(self):
        self.api_key = os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY not set in environment")
        self.base_url = "https://ai.google.dev/gemini-api"

    async def analyze_image(self, image_bytes: bytes) -> Dict[str, Any]:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/octet-stream"
        }
        async with httpx.AsyncClient(timeout=30) as client:
            response = await client.post(f"{self.base_url}/vision/analyze", content=image_bytes, headers=headers)
            response.raise_for_status()
            return response.json()

    async def status(self) -> Dict[str, Any]:
        # Simplified status check
        return {"name": "Gemini", "available": True}
