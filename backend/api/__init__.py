from fastapi import FastAPI
from backend.api.incident_timeline import router as incident_timeline_router

app = FastAPI(
    title="Bulo.Cloud Sentinel Backend API",
    description="API for Bulo.Cloud Sentinel modules",
    version="0.1.0"
)

app.include_router(incident_timeline_router)
from dronecore.drone_command_telemetry_hub import router as drone_hub_router

app.include_router(drone_hub_router)
from ai_detection.ai_model_management import router as ai_model_management_router

app.include_router(ai_model_management_router)
from backend.api.device_inventory import router as device_inventory_router

app.include_router(device_inventory_router)
from backend.api.audit_log import router as audit_log_router

app.include_router(audit_log_router)
from backend.api.login import router as login_router

app.include_router(login_router)
from backend.api.me import router as me_router

app.include_router(me_router)
from backend.api.metrics import router as metrics_router

app.include_router(metrics_router)
