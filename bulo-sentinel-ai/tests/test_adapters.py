import pytest
import asyncio
from bulo_sentinel_ai.main import adapters

@pytest.mark.asyncio
async def test_chatgpt_adapter_chat():
    adapter = adapters["chatgpt"]
    response = await adapter.chat("Hello, world!")
    assert isinstance(response, str)
    assert len(response) > 0

@pytest.mark.asyncio
async def test_claude_adapter_chat():
    adapter = adapters["claude"]
    response = await adapter.chat("Hello, world!")
    assert isinstance(response, str)

@pytest.mark.asyncio
async def test_gemini_adapter_status():
    adapter = adapters["gemini"]
    status = await adapter.status()
    assert "name" in status
    assert "available" in status

@pytest.mark.asyncio
async def test_dalle_adapter_status():
    adapter = adapters["dalle"]
    status = await adapter.status()
    assert "name" in status
    assert "available" in status

@pytest.mark.asyncio
async def test_whisper_adapter_status():
    adapter = adapters["whisper"]
    status = await adapter.status()
    assert "name" in status
    assert "available" in status
