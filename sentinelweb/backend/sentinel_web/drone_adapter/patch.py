"""
Patch script for SentinelWeb

This script patches SentinelWeb's main.py to integrate with BuloCloudSentinel.
It adds routes and dependencies for drone operations.
"""

import os
import sys
import re
from pathlib import Path

def patch_sentinelweb():
    """Patch SentinelWeb's main.py to integrate with BuloCloudSentinel."""
    # Get the path to SentinelWeb's main.py
    sentinelweb_path = Path("/app/backend/sentinel_web")
    main_py_path = sentinelweb_path / "main.py"
    
    if not main_py_path.exists():
        print(f"Error: {main_py_path} not found")
        return False
    
    # Read the main.py file
    with open(main_py_path, "r") as f:
        content = f.read()
    
    # Add import for drone adapter
    import_pattern = r"from sentinel_web.utils import"
    drone_import = "from sentinel_web.drone_adapter.adapter import get_drone_adapter\n"
    content = re.sub(import_pattern, drone_import + import_pattern, content)
    
    # Add drone routes
    routes_pattern = r"app.include_router\(routers.settings.router, prefix=\"/api/settings\", tags=\[\"settings\"\]\)"
    drone_routes = """
# Drone routes
app.include_router(routers.drones.router, prefix="/api/drones", tags=["drones"])
app.include_router(routers.missions.router, prefix="/api/missions", tags=["missions"])
app.include_router(routers.telemetry.router, prefix="/api/telemetry", tags=["telemetry"])
app.include_router(routers.video.router, prefix="/api/video", tags=["video"])
"""
    content = re.sub(routes_pattern, routes_pattern + drone_routes, content)
    
    # Add drone dependencies
    deps_pattern = r"@app.get\(\"/api/config\"\)"
    drone_deps = """
# Drone dependencies
@app.get("/api/drone/config")
async def get_drone_config():
    """Get drone configuration."""
    adapter = get_drone_adapter()
    return adapter.get_config()

"""
    content = re.sub(deps_pattern, drone_deps + deps_pattern, content)
    
    # Write the modified content back to main.py
    with open(main_py_path, "w") as f:
        f.write(content)
    
    print(f"Successfully patched {main_py_path}")
    return True

if __name__ == "__main__":
    patch_sentinelweb()
