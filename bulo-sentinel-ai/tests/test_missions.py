import pytest
import asyncio
from httpx import AsyncClient
from bulo_sentinel_ai.main import app

@pytest.mark.asyncio
async def test_mission_crud():
    async with AsyncClient(app=app, base_url="http://test") as ac:
        # Create mission
        mission_data = {
            "name": "Test Mission",
            "description": "Test Description",
            "waypoints": [
                {"latitude": 10.0, "longitude": 20.0, "altitude": 100, "action": "takeoff", "order": 1},
                {"latitude": 10.1, "longitude": 20.1, "altitude": 110, "action": "hover", "order": 2}
            ]
        }
        response = await ac.post("/missions/", json=mission_data)
        assert response.status_code == 201
        mission = response.json()
        mission_id = mission["id"]

        # Get mission
        response = await ac.get(f"/missions/{mission_id}")
        assert response.status_code == 200
        assert response.json()["name"] == "Test Mission"

        # Update mission
        mission_data["name"] = "Updated Mission"
        response = await ac.put(f"/missions/{mission_id}", json=mission_data)
        assert response.status_code == 200
        assert response.json()["name"] == "Updated Mission"

        # List missions
        response = await ac.get("/missions/")
        assert response.status_code == 200
        assert any(m["id"] == mission_id for m in response.json())

        # Delete mission
        response = await ac.delete(f"/missions/{mission_id}")
        assert response.status_code == 204

        # Confirm deletion
        response = await ac.get(f"/missions/{mission_id}")
        assert response.status_code == 404
