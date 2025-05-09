#!/usr/bin/env python3
"""
Weather Guard Demo

This script demonstrates how to use the Weather Guard service to get weather data
and check mission weather conditions.
"""

import asyncio
import argparse
import logging
from datetime import datetime, timedelta

from weather_guard.client import WeatherGuardClient
from weather_guard.models.weather import WeatherProvider


async def main():
    """Run the Weather Guard demo."""
    parser = argparse.ArgumentParser(description="Weather Guard Demo")
    parser.add_argument("--latitude", type=float, default=52.52, help="Latitude")
    parser.add_argument("--longitude", type=float, default=13.41, help="Longitude")
    parser.add_argument("--hours", type=int, default=24, help="Forecast hours")
    parser.add_argument("--api-url", type=str, default="http://localhost:8090/api", help="Weather Guard API URL")
    args = parser.parse_args()
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    
    # Create client
    client = WeatherGuardClient(base_url=args.api_url)
    
    try:
        # Get current weather
        print(f"\n=== Current Weather for {args.latitude}, {args.longitude} ===")
        current_weather = await client.get_current_weather(args.latitude, args.longitude)
        print(f"Temperature: {current_weather.temperature}°C")
        print(f"Condition: {current_weather.condition}")
        print(f"Wind Speed: {current_weather.wind_speed} m/s")
        print(f"Wind Direction: {current_weather.wind_direction}°")
        print(f"Precipitation: {current_weather.precipitation} mm/h")
        
        # Get forecast
        print(f"\n=== Weather Forecast for {args.latitude}, {args.longitude} ({args.hours} hours) ===")
        forecast = await client.get_forecast(args.latitude, args.longitude, args.hours)
        for i, item in enumerate(forecast):
            if i > 5:  # Show only first 6 hours
                break
            print(f"{item.forecast_time.strftime('%Y-%m-%d %H:%M')}: {item.temperature}°C, {item.condition}, Wind: {item.wind_speed} m/s, Rain: {item.precipitation} mm/h")
        
        # Check mission weather
        print(f"\n=== Mission Weather Check ===")
        start_time = datetime.utcnow() + timedelta(hours=2)  # Mission starts in 2 hours
        end_time = start_time + timedelta(hours=1)  # Mission lasts 1 hour
        check_result = await client.check_mission_weather(
            args.latitude,
            args.longitude,
            start_time,
            end_time,
            mission_id="demo-mission-123",
        )
        
        print(f"Mission Start: {check_result.start_time}")
        print(f"Mission End: {check_result.end_time}")
        print(f"Is Flyable: {check_result.is_flyable}")
        print(f"Severity: {check_result.severity}")
        
        if check_result.limitations:
            print("\nLimitations:")
            for limitation in check_result.limitations:
                print(f"- {limitation['description']}")
        
        if check_result.recommendations:
            print("\nRecommendations:")
            for recommendation in check_result.recommendations:
                print(f"- {recommendation}")
        
        # Get service status
        print(f"\n=== Service Status ===")
        status = await client.get_service_status()
        print(f"Status: {status['status']}")
        print(f"Version: {status['version']}")
        print(f"Providers: {', '.join(status['providers'])}")
        print(f"Uptime: {status['uptime'] / 60:.1f} minutes")
        print(f"Error Count: {status['error_count']}")
        
        # Cache stats
        print("\nCache Stats:")
        for cache_name, stats in status['cache_status'].items():
            print(f"- {cache_name}: {stats['total_keys']} keys, TTL: {stats['ttl']} seconds")
    
    except Exception as e:
        logging.error(f"Error: {str(e)}")
    
    finally:
        # Close client
        await client.close()


if __name__ == "__main__":
    asyncio.run(main())
