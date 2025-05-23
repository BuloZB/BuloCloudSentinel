#!/usr/bin/env python3
"""
DJI Mission Planner Web UI

This module provides a simple web-based UI for planning and monitoring DJI drone missions.
It allows users to:
- Create and edit waypoint missions
- Schedule missions
- Monitor mission execution
- View telemetry data
- Control the drone remotely

The UI is built using Flask and communicates with the DJI adapter through a REST API.
"""

import os
import sys
import json
import logging
import datetime
import time
import threading
import asyncio
from typing import Dict, Any, List, Optional
from flask import Flask, request, jsonify, render_template, send_from_directory
from werkzeug.serving import run_simple

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dronecore.adapter_factory import AdapterFactory
from dronecore.dji_adapter import DJIAdapter
from dronecore.dji_dock_integration import DJIDockIntegration
from dock_driver.adapters.dji.adapter import DJIDockAdapter

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create Flask app
app = Flask(__name__, 
            static_folder=os.path.join(os.path.dirname(__file__), 'static'),
            template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Global variables
drone_adapter = None
dock_adapter = None
dock_integration = None
telemetry_data = {}
mission_status = {
    "state": "idle",
    "current_mission": None,
    "progress": 0,
    "waypoint_index": 0,
    "total_waypoints": 0,
    "battery": 0,
    "errors": []
}
scheduled_missions = []
mission_history = []
is_connected = False
is_mission_running = False
connection_params = {}

# Async event loop for background tasks
loop = None
telemetry_task = None
running = False

# Mission directory
MISSION_DIR = os.path.join(os.path.dirname(__file__), '..', 'missions')
os.makedirs(MISSION_DIR, exist_ok=True)

# Default connection parameters
DEFAULT_CONNECTION_PARAMS = {
    "app_id": os.environ.get("DJI_APP_ID", "your_app_id_here"),
    "app_key": os.environ.get("DJI_APP_KEY", "your_app_key_here"),
    "connection_type": "USB",
    "drone_model": "Mavic 3",
    "enable_virtual_stick": True,
    "enable_camera": True,
    "enable_gimbal": True,
    "enable_waypoint": True,
    "enable_hotpoint": True,
    "enable_follow_me": True,
    "enable_timeline": True,
    "enable_hd_video": True,
}

# Default dock parameters
DEFAULT_DOCK_PARAMS = {
    "api_key": os.environ.get("DJI_DOCK_API_KEY", "your_api_key_here"),
    "api_secret": os.environ.get("DJI_DOCK_API_SECRET", "your_api_secret_here"),
    "dock_sn": os.environ.get("DJI_DOCK_SN", "your_dock_sn_here"),
    "region": "us-east-1",
    "refresh_interval": 30
}

# Routes
@app.route('/')
def index():
    """Render the main page."""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """Get the current status of the drone and mission."""
    global telemetry_data, mission_status, is_connected
    
    return jsonify({
        "connected": is_connected,
        "telemetry": telemetry_data,
        "mission": mission_status,
        "timestamp": datetime.datetime.now().isoformat()
    })

@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to the drone."""
    global drone_adapter, is_connected, connection_params, telemetry_data
    
    data = request.json
    
    # Update connection parameters
    params = DEFAULT_CONNECTION_PARAMS.copy()
    if data:
        params.update(data)
    
    connection_params = params
    
    # Create adapter if it doesn't exist
    if not drone_adapter:
        drone_adapter = AdapterFactory.create_adapter("dji", connection_params)
        if not drone_adapter:
            return jsonify({"success": False, "error": "Failed to create DJI adapter"})
    
    # Connect to the drone
    async def _connect():
        global is_connected
        try:
            connected = await drone_adapter.connect()
            is_connected = connected
            if connected:
                # Start telemetry task
                start_telemetry_task()
                return {"success": True}
            else:
                return {"success": False, "error": "Failed to connect to drone"}
        except Exception as e:
            logger.error(f"Error connecting to drone: {str(e)}")
            return {"success": False, "error": str(e)}
    
    result = asyncio.run_coroutine_threadsafe(_connect(), loop).result()
    return jsonify(result)

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from the drone."""
    global drone_adapter, is_connected
    
    # Disconnect from the drone
    async def _disconnect():
        global is_connected
        try:
            if drone_adapter and is_connected:
                # Stop telemetry task
                stop_telemetry_task()
                
                # Disconnect
                disconnected = await drone_adapter.disconnect()
                is_connected = not disconnected
                if disconnected:
                    return {"success": True}
                else:
                    return {"success": False, "error": "Failed to disconnect from drone"}
            else:
                return {"success": True, "message": "Not connected"}
        except Exception as e:
            logger.error(f"Error disconnecting from drone: {str(e)}")
            return {"success": False, "error": str(e)}
    
    result = asyncio.run_coroutine_threadsafe(_disconnect(), loop).result()
    return jsonify(result)

@app.route('/api/command', methods=['POST'])
def send_command():
    """Send a command to the drone."""
    global drone_adapter, is_connected
    
    if not is_connected:
        return jsonify({"success": False, "error": "Not connected to drone"})
    
    data = request.json
    if not data or "command" not in data:
        return jsonify({"success": False, "error": "Invalid command"})
    
    # Send command to the drone
    try:
        command = data["command"]
        parameters = data.get("parameters", {})
        
        result = drone_adapter.send_command({
            "command": command,
            "parameters": parameters
        })
        
        return jsonify({"success": result})
    except Exception as e:
        logger.error(f"Error sending command: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/missions', methods=['GET'])
def list_missions():
    """List all available missions."""
    missions = []
    
    try:
        for filename in os.listdir(MISSION_DIR):
            if filename.endswith('.json'):
                mission_path = os.path.join(MISSION_DIR, filename)
                with open(mission_path, 'r') as f:
                    mission_data = json.load(f)
                    missions.append({
                        "filename": filename,
                        "name": mission_data.get("mission_name", "Unnamed Mission"),
                        "type": mission_data.get("mission_type", "waypoint"),
                        "waypoints": len(mission_data.get("waypoints", [])),
                        "created": datetime.datetime.fromtimestamp(os.path.getctime(mission_path)).isoformat()
                    })
    except Exception as e:
        logger.error(f"Error listing missions: {str(e)}")
    
    return jsonify(missions)

@app.route('/api/missions/<filename>', methods=['GET'])
def get_mission(filename):
    """Get a specific mission."""
    mission_path = os.path.join(MISSION_DIR, filename)
    
    if not os.path.exists(mission_path):
        return jsonify({"success": False, "error": "Mission not found"})
    
    try:
        with open(os.path.normpath(mission_path, 'r')) as f:
            mission_data = json.load(f)
        return jsonify({"success": True, "mission": mission_data})
    except Exception as e:
        logger.error(f"Error reading mission: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/missions', methods=['POST'])
def create_mission():
    """Create a new mission."""
    data = request.json
    
    if not data or "mission_name" not in data or "mission_type" not in data:
        return jsonify({"success": False, "error": "Invalid mission data"})
    
    try:
        # Generate filename
        filename = f"{data['mission_name'].replace(' ', '_').lower()}_{int(time.time())}.json"
        mission_path = os.path.join(MISSION_DIR, filename)
        
        # Save mission
        with open(os.path.normpath(mission_path, 'w')) as f:
            json.dump(data, f, indent=2)
        
        return jsonify({"success": True, "filename": filename})
    except Exception as e:
        logger.error(f"Error creating mission: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/missions/<filename>', methods=['PUT'])
def update_mission(filename):
    """Update an existing mission."""
    mission_path = os.path.join(MISSION_DIR, filename)
    
    if not os.path.exists(mission_path):
        return jsonify({"success": False, "error": "Mission not found"})
    
    data = request.json
    if not data:
        return jsonify({"success": False, "error": "Invalid mission data"})
    
    try:
        # Save mission
        with open(os.path.normpath(mission_path, 'w')) as f:
            json.dump(data, f, indent=2)
        
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error updating mission: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/missions/<filename>', methods=['DELETE'])
def delete_mission(filename):
    """Delete a mission."""
    mission_path = os.path.join(MISSION_DIR, filename)
    
    if not os.path.exists(mission_path):
        return jsonify({"success": False, "error": "Mission not found"})
    
    try:
        os.remove(mission_path)
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Error deleting mission: {str(e)}")
        return jsonify({"success": False, "error": str(e)})

@app.route('/api/execute_mission/<filename>', methods=['POST'])
def execute_mission(filename):
    """Execute a mission."""
    global drone_adapter, is_connected, is_mission_running, mission_status
    
    if not is_connected:
        return jsonify({"success": False, "error": "Not connected to drone"})
    
    if is_mission_running:
        return jsonify({"success": False, "error": "Mission already running"})
    
    mission_path = os.path.join(MISSION_DIR, filename)
    
    if not os.path.exists(mission_path):
        return jsonify({"success": False, "error": "Mission not found"})
    
    # Execute mission
    async def _execute_mission():
        global is_mission_running, mission_status
        
        try:
            # Load mission
            with open(mission_path, 'r') as f:
                mission_data = json.load(f)
            
            mission_type = mission_data.get("mission_type", "waypoint")
            mission_status["state"] = "executing"
            mission_status["current_mission"] = filename
            mission_status["progress"] = 0
            mission_status["waypoint_index"] = 0
            mission_status["total_waypoints"] = len(mission_data.get("waypoints", []))
            mission_status["errors"] = []
            is_mission_running = True
            
            # Execute based on mission type
            if mission_type == "waypoint":
                # Start waypoint mission
                command = {
                    "command": "start_waypoint_mission",
                    "parameters": {
                        "waypoints": mission_data.get("waypoints", []),
                        "speed": mission_data.get("speed", 5.0),
                        "finish_action": mission_data.get("finish_action", "no_action"),
                        "heading_mode": mission_data.get("heading_mode", "auto")
                    }
                }
                result = drone_adapter.send_command(command)
                
                if not result:
                    mission_status["state"] = "error"
                    mission_status["errors"].append("Failed to start waypoint mission")
                    is_mission_running = False
                    return {"success": False, "error": "Failed to start waypoint mission"}
                
                # Monitor mission progress
                mission_timeout = mission_data.get("timeout", 1800)  # 30 minutes default
                start_time = time.time()
                
                while time.time() - start_time < mission_timeout:
                    # Check if mission is still executing
                    mission_state = await drone_adapter.get_waypoint_mission_state()
                    if not mission_state["executing"]:
                        mission_status["state"] = "completed"
                        mission_status["progress"] = 100
                        is_mission_running = False
                        return {"success": True}
                    
                    # Update progress
                    if mission_status["total_waypoints"] > 0:
                        mission_status["waypoint_index"] = mission_state["current_waypoint_index"]
                        mission_status["progress"] = int((mission_state["current_waypoint_index"] + 1) / mission_status["total_waypoints"] * 100)
                    
                    # Check for critical conditions
                    telemetry = drone_adapter.receive_telemetry()
                    battery = telemetry['battery']['percent']
                    
                    if battery <= drone_adapter._critical_battery_warning:
                        mission_status["state"] = "aborted"
                        mission_status["errors"].append(f"Critical battery level: {battery}%")
                        
                        # Stop mission
                        command = {
                            "command": "stop_waypoint_mission",
                            "parameters": {}
                        }
                        drone_adapter.send_command(command)
                        
                        is_mission_running = False
                        return {"success": False, "error": "Mission aborted due to critical battery"}
                    
                    await asyncio.sleep(1)
                
                mission_status["state"] = "timeout"
                mission_status["errors"].append(f"Mission timeout after {mission_timeout} seconds")
                
                # Stop mission
                command = {
                    "command": "stop_waypoint_mission",
                    "parameters": {}
                }
                drone_adapter.send_command(command)
                
                is_mission_running = False
                return {"success": False, "error": "Mission timeout"}
            
            elif mission_type == "hotpoint":
                # Execute hotpoint mission
                # ... (implementation similar to waypoint mission)
                pass
            
            else:
                mission_status["state"] = "error"
                mission_status["errors"].append(f"Unsupported mission type: {mission_type}")
                is_mission_running = False
                return {"success": False, "error": f"Unsupported mission type: {mission_type}"}
        
        except Exception as e:
            logger.error(f"Error executing mission: {str(e)}")
            mission_status["state"] = "error"
            mission_status["errors"].append(str(e))
            is_mission_running = False
            return {"success": False, "error": str(e)}
    
    # Execute mission in background
    threading.Thread(target=lambda: asyncio.run_coroutine_threadsafe(_execute_mission(), loop)).start()
    
    return jsonify({"success": True, "message": "Mission execution started"})

@app.route('/api/stop_mission', methods=['POST'])
def stop_mission():
    """Stop the current mission."""
    global drone_adapter, is_connected, is_mission_running, mission_status
    
    if not is_connected:
        return jsonify({"success": False, "error": "Not connected to drone"})
    
    if not is_mission_running:
        return jsonify({"success": False, "error": "No mission running"})
    
    # Stop mission
    try:
        mission_type = mission_status.get("mission_type", "waypoint")
        
        if mission_type == "waypoint":
            command = {
                "command": "stop_waypoint_mission",
                "parameters": {}
            }
        elif mission_type == "hotpoint":
            command = {
                "command": "stop_hotpoint_mission",
                "parameters": {}
            }
        else:
            return jsonify({"success": False, "error": f"Unsupported mission type: {mission_type}"})
        
        result = drone_adapter.send_command(command)
        
        if result:
            mission_status["state"] = "stopped"
            is_mission_running = False
            return jsonify({"success": True})
        else:
            return jsonify({"success": False, "error": "Failed to stop mission"})
    except Exception as e:
        logger.error(f"Error stopping mission: {str(e)}")
        return jsonify({"success": False, "error": str(e)})
