#!/usr/bin/env python3
"""
Indoor Fallback Integration Example

This script demonstrates how to integrate the Weather Guard service with an indoor
drone system for automatic fallback when outdoor conditions are unfavorable.
"""

import asyncio
import argparse
import json
import logging
import sys
from datetime import datetime, timedelta, timezone

import httpx

from weather_guard.client import WeatherGuardClient


class IndoorFallbackSystem:
    """Indoor fallback system for drone operations."""
    
    def __init__(
        self,
        weather_api_url: str = "http://localhost:8090/api",
        indoor_api_url: str = "http://localhost:8091/api",
        check_interval: int = 300,  # 5 minutes
    ):
        """Initialize the indoor fallback system.
        
        Args:
            weather_api_url: Weather Guard API URL
            indoor_api_url: Indoor drone system API URL
            check_interval: Weather check interval in seconds
        """
        self.weather_api_url = weather_api_url
        self.indoor_api_url = indoor_api_url
        self.check_interval = check_interval
        
        self.weather_client = WeatherGuardClient(base_url=weather_api_url)
        self.indoor_client = httpx.AsyncClient(base_url=indoor_api_url, timeout=30)
        
        self.logger = logging.getLogger("indoor_fallback")
        
        # Store missions
        self.outdoor_missions = {}
        self.indoor_fallbacks = {}
    
    async def close(self):
        """Close clients."""
        await self.weather_client.close()
        await self.indoor_client.aclose()
    
    async def register_mission(
        self,
        mission_id: str,
        latitude: float,
        longitude: float,
        start_time: datetime,
        end_time: datetime,
        mission_data: dict,
    ) -> bool:
        """Register an outdoor mission with indoor fallback.
        
        Args:
            mission_id: Mission ID
            latitude: Mission latitude
            longitude: Mission longitude
            start_time: Mission start time
            end_time: Mission end time
            mission_data: Mission data
            
        Returns:
            True if mission was registered successfully
        """
        try:
            # Store mission
            self.outdoor_missions[mission_id] = {
                "id": mission_id,
                "latitude": latitude,
                "longitude": longitude,
                "start_time": start_time,
                "end_time": end_time,
                "data": mission_data,
                "status": "pending",
            }
            
            # Create indoor fallback mission
            indoor_mission_id = await self._create_indoor_fallback(mission_id, mission_data)
            
            if indoor_mission_id:
                self.indoor_fallbacks[mission_id] = indoor_mission_id
                self.logger.info(f"Created indoor fallback {indoor_mission_id} for mission {mission_id}")
                return True
            else:
                self.logger.error(f"Failed to create indoor fallback for mission {mission_id}")
                return False
        
        except Exception as e:
            self.logger.error(f"Error registering mission: {str(e)}")
            return False
    
    async def _create_indoor_fallback(self, mission_id: str, mission_data: dict) -> str:
        """Create an indoor fallback mission.
        
        Args:
            mission_id: Outdoor mission ID
            mission_data: Mission data
            
        Returns:
            Indoor mission ID or empty string if failed
        """
        try:
            # Convert outdoor mission to indoor mission
            indoor_data = self._convert_to_indoor_mission(mission_data)
            
            # Create indoor mission
            response = await self.indoor_client.post(
                "/missions",
                json={
                    "name": f"Indoor Fallback for {mission_id}",
                    "description": f"Automatically created indoor fallback for outdoor mission {mission_id}",
                    "mission_type": "indoor_fallback",
                    "status": "pending",
                    "data": indoor_data,
                    "metadata": {
                        "outdoor_mission_id": mission_id,
                        "created_by": "weather_guard",
                    },
                },
            )
            
            response.raise_for_status()
            data = response.json()
            
            return data.get("id", "")
        
        except Exception as e:
            self.logger.error(f"Error creating indoor fallback: {str(e)}")
            return ""
    
    def _convert_to_indoor_mission(self, outdoor_mission: dict) -> dict:
        """Convert outdoor mission to indoor mission.
        
        Args:
            outdoor_mission: Outdoor mission data
            
        Returns:
            Indoor mission data
        """
        # This is a simplified example - in a real system, this would be more complex
        # and would depend on the specific mission types and requirements
        
        # Extract waypoints and convert to indoor coordinates
        outdoor_waypoints = outdoor_mission.get("waypoints", [])
        indoor_waypoints = []
        
        for wp in outdoor_waypoints:
            # Convert GPS coordinates to indoor coordinates
            # This is a placeholder - real implementation would use actual conversion
            indoor_wp = {
                "x": (wp.get("latitude", 0) - 52.0) * 1000,  # Example conversion
                "y": (wp.get("longitude", 0) - 13.0) * 1000,  # Example conversion
                "z": wp.get("altitude", 2),  # Use same altitude or default to 2m
                "action": wp.get("action", "hover"),
                "duration": wp.get("duration", 5),
            }
            
            indoor_waypoints.append(indoor_wp)
        
        # Create indoor mission data
        indoor_mission = {
            "waypoints": indoor_waypoints,
            "drone_type": "indoor",
            "speed": outdoor_mission.get("speed", 1.0),
            "use_collision_avoidance": True,
            "use_visual_positioning": True,
        }
        
        return indoor_mission
    
    async def check_weather_and_update_missions(self) -> None:
        """Check weather for all pending missions and update status."""
        now = datetime.now(timezone.utc)
        
        # Check each pending mission
        for mission_id, mission in list(self.outdoor_missions.items()):
            if mission["status"] != "pending":
                continue
            
            # Skip missions that are too far in the future (> 24h)
            if mission["start_time"] > now + timedelta(hours=24):
                continue
            
            # Check weather for this mission
            try:
                check_result = await self.weather_client.check_mission_weather(
                    mission["latitude"],
                    mission["longitude"],
                    mission["start_time"],
                    mission["end_time"],
                    mission_id,
                )
                
                # If not flyable, trigger indoor fallback
                if not check_result.is_flyable:
                    await self._trigger_indoor_fallback(mission_id, check_result)
            
            except Exception as e:
                self.logger.error(f"Error checking weather for mission {mission_id}: {str(e)}")
    
    async def _trigger_indoor_fallback(self, mission_id: str, check_result) -> None:
        """Trigger indoor fallback for a mission.
        
        Args:
            mission_id: Mission ID
            check_result: Weather check result
        """
        try:
            # Get indoor fallback mission ID
            indoor_mission_id = self.indoor_fallbacks.get(mission_id)
            
            if not indoor_mission_id:
                self.logger.error(f"No indoor fallback found for mission {mission_id}")
                return
            
            # Update outdoor mission status
            self.outdoor_missions[mission_id]["status"] = "cancelled_weather"
            
            # Cancel outdoor mission
            await self._cancel_outdoor_mission(mission_id, check_result)
            
            # Activate indoor fallback
            await self._activate_indoor_fallback(indoor_mission_id, check_result)
            
            self.logger.info(f"Triggered indoor fallback {indoor_mission_id} for mission {mission_id}")
        
        except Exception as e:
            self.logger.error(f"Error triggering indoor fallback: {str(e)}")
    
    async def _cancel_outdoor_mission(self, mission_id: str, check_result) -> None:
        """Cancel outdoor mission due to weather.
        
        Args:
            mission_id: Mission ID
            check_result: Weather check result
        """
        # This would integrate with your mission planning system
        self.logger.info(f"Cancelled outdoor mission {mission_id} due to weather")
        
        # Log the weather conditions that caused the cancellation
        limitations = check_result.limitations
        if limitations:
            for limitation in limitations:
                self.logger.info(f"Weather limitation: {limitation.get('description', 'Unknown')}")
    
    async def _activate_indoor_fallback(self, indoor_mission_id: str, check_result) -> None:
        """Activate indoor fallback mission.
        
        Args:
            indoor_mission_id: Indoor mission ID
            check_result: Weather check result
        """
        try:
            # Update indoor mission status
            response = await self.indoor_client.patch(
                f"/missions/{indoor_mission_id}",
                json={
                    "status": "scheduled",
                    "metadata": {
                        "weather_conditions": {
                            "wind_speed": check_result.current_weather.wind_speed,
                            "precipitation": check_result.current_weather.precipitation,
                            "condition": str(check_result.current_weather.condition),
                            "is_flyable": check_result.is_flyable,
                            "severity": str(check_result.severity),
                        }
                    },
                },
            )
            
            response.raise_for_status()
            self.logger.info(f"Activated indoor fallback mission {indoor_mission_id}")
        
        except Exception as e:
            self.logger.error(f"Error activating indoor fallback: {str(e)}")
    
    async def run(self) -> None:
        """Run the indoor fallback system."""
        try:
            while True:
                await self.check_weather_and_update_missions()
                await asyncio.sleep(self.check_interval)
        
        except asyncio.CancelledError:
            self.logger.info("Indoor fallback system stopped")
        
        except Exception as e:
            self.logger.error(f"Error in indoor fallback system: {str(e)}")
        
        finally:
            await self.close()


async def main():
    """Run the indoor fallback integration example."""
    parser = argparse.ArgumentParser(description="Indoor Fallback Integration Example")
    parser.add_argument("--weather-api", type=str, default="http://localhost:8090/api", help="Weather Guard API URL")
    parser.add_argument("--indoor-api", type=str, default="http://localhost:8091/api", help="Indoor drone system API URL")
    parser.add_argument("--interval", type=int, default=300, help="Check interval in seconds")
    args = parser.parse_args()
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    
    logger = logging.getLogger("indoor_fallback_example")
    
    # Create indoor fallback system
    system = IndoorFallbackSystem(
        weather_api_url=args.weather_api,
        indoor_api_url=args.indoor_api,
        check_interval=args.interval,
    )
    
    # Register example mission
    now = datetime.now(timezone.utc)
    start_time = now + timedelta(hours=1)
    end_time = start_time + timedelta(hours=1)
    
    mission_data = {
        "name": "Example Outdoor Mission",
        "drone_type": "outdoor",
        "waypoints": [
            {"latitude": 52.52, "longitude": 13.41, "altitude": 10, "action": "takeoff"},
            {"latitude": 52.53, "longitude": 13.42, "altitude": 20, "action": "hover", "duration": 30},
            {"latitude": 52.54, "longitude": 13.43, "altitude": 15, "action": "capture_image"},
            {"latitude": 52.52, "longitude": 13.41, "altitude": 5, "action": "land"},
        ],
        "speed": 2.0,
    }
    
    success = await system.register_mission(
        "example-mission-1",
        52.52,
        13.41,
        start_time,
        end_time,
        mission_data,
    )
    
    if success:
        logger.info("Registered example mission with indoor fallback")
        
        # Run the system
        try:
            await system.run()
        except KeyboardInterrupt:
            logger.info("Stopping indoor fallback system")
        finally:
            await system.close()
    else:
        logger.error("Failed to register example mission")
        await system.close()


if __name__ == "__main__":
    asyncio.run(main())
