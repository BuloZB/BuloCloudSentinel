#!/usr/bin/env python3
"""
Run the DJI Mission Planner web UI.
"""

import os
import sys
import subprocess

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Combine JavaScript files
print("Combining JavaScript files...")
subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), 'combine_js.py')])

# Run the web server
print("Starting DJI Mission Planner web server...")
from web.dji_mission_planner import app

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=False)
