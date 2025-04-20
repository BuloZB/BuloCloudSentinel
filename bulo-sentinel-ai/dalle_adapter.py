import os
import httpx
from typing import Optional, Dict, Any

class DalleAdapter:
    def __init__(self):
        self.api_key = os.getenv("DALLE_API_KEY")
        if not self.api_key:
            raise ValueError("DALLE_API_KEY not set in environment")
        self.base_url = "https://api.openai.com/v1/images/generations"

    async def generate_image(self, prompt: str) -> Dict[str, Any]:
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        data = {
            "prompt": prompt,
            "n": 1,
            "size": "1024x1024"
        }
        async with httpx.AsyncClient(timeout=60) as client:
            response = await client.post(self.base_url, json=data, headers=headers)
            response.raise_for_status()
            return response.json()

    async def status(self) -> Dict[str, Any]:
        # Simplified status check
        return {"name": "DALL·E", "available": True}
