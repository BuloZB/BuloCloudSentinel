import os
import httpx
from typing import Optional, Dict, Any

class ClaudeAdapter:
    def __init__(self):
        self.api_key = os.getenv("CLAUDE_API_KEY")
        if not self.api_key:
            raise ValueError("CLAUDE_API_KEY not set in environment")
        self.base_url = "https://api.anthropic.com/v1"

    async def chat(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> str:
        headers = {
            "x-api-key": self.api_key,
            "Content-Type": "application/json"
        }
        data = {
            "prompt": prompt,
            "model": "claude-v1",
            "max_tokens_to_sample": 1000
        }
        async with httpx.AsyncClient(timeout=30) as client:
            response = await client.post(f"{self.base_url}/complete", json=data, headers=headers)
            response.raise_for_status()
            result = response.json()
            return result.get("completion", "")

    async def status(self) -> Dict[str, Any]:
        # Simplified status check
        return {"name": "Claude", "available": True}
