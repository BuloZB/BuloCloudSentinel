"""
Patch script for OpenWebUI

This script patches OpenWebUI's main.py to integrate with SentinelWeb.
It adds routes and dependencies for drone operations.
"""

import os
import sys
import re
from pathlib import Path

def patch_openwebui():
    """Patch OpenWebUI's main.py to integrate with SentinelWeb."""
    # Get the path to OpenWebUI's main.py
    openwebui_path = Path("/app/backend/open_webui")
    main_py_path = openwebui_path / "main.py"
    
    if not main_py_path.exists():
        print(f"Error: {main_py_path} not found")
        return False
    
    # Read the main.py file
    with open(main_py_path, "r") as f:
        content = f.read()
    
    # Add import for SentinelWeb adapter
    import_pattern = r"from open_webui.utils import"
    sentinel_import = "from open_webui.sentinel_adapter.main import get_sentinel_adapter\n"
    content = re.sub(import_pattern, sentinel_import + import_pattern, content)
    
    # Add SentinelWeb routes
    routes_pattern = r"app.include_router\(routers.settings.router, prefix=\"/api/settings\", tags=\[\"settings\"\]\)"
    sentinel_routes = """
# SentinelWeb routes
app.include_router(routers.drones.router, prefix="/api/drones", tags=["drones"])
app.include_router(routers.missions.router, prefix="/api/missions", tags=["missions"])
app.include_router(routers.telemetry.router, prefix="/api/telemetry", tags=["telemetry"])
app.include_router(routers.video.router, prefix="/api/video", tags=["video"])
"""
    content = re.sub(routes_pattern, routes_pattern + sentinel_routes, content)
    
    # Add SentinelWeb dependencies
    deps_pattern = r"@app.get\(\"/api/config\"\)"
    sentinel_deps = """
# SentinelWeb dependencies
@app.get("/api/sentinel/config")
async def get_sentinel_config():
    """Get SentinelWeb configuration."""
    adapter = get_sentinel_adapter()
    return adapter.get_config()

"""
    content = re.sub(deps_pattern, sentinel_deps + deps_pattern, content)
    
    # Write the modified content back to main.py
    with open(main_py_path, "w") as f:
        f.write(content)
    
    print(f"Successfully patched {main_py_path}")
    return True

if __name__ == "__main__":
    patch_openwebui()
