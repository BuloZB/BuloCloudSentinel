from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from backend.api.dependencies import get_current_user
from backend.api.audit_log import router as audit_log_router
from backend.api.device_inventory import router as device_inventory_router
from backend.api.drone import router as drone_router
from backend.api.incident_timeline import router as incident_timeline_router
from backend.api.login import router as login_router
from backend.api.me import router as me_router
from backend.api.metrics import router as metrics_router
from backend.api.missions import router as missions_router
from backend.api.analytics import router as analytics_router

app = FastAPI(
    title="Bulo.Cloud Sentinel Backend API",
    description="Backend API for Bulo.Cloud Sentinel platform",
    version="0.1.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust as needed for security
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(login_router)
app.include_router(me_router, dependencies=[Depends(get_current_user)])
app.include_router(drone_router, dependencies=[Depends(get_current_user)])
app.include_router(incident_timeline_router, dependencies=[Depends(get_current_user)])
app.include_router(device_inventory_router, dependencies=[Depends(get_current_user)])
app.include_router(audit_log_router, dependencies=[Depends(get_current_user)])
app.include_router(metrics_router)
app.include_router(missions_router, dependencies=[Depends(get_current_user)])
from backend.api.ws_missions import router as ws_missions_router
app.include_router(ws_missions_router, dependencies=[Depends(get_current_user)])
app.include_router(analytics_router, dependencies=[Depends(get_current_user)])
argon2-cffi==23.1.0  # For secure password hashing
pydantic==2.11.4  # For secure data validation
python-magic==0.4.27  # For secure file type detection
safety==2.3.5  # For dependency vulnerability scanning
bandit==1.7.7  # For security static analysis
cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119
pyopenssl==24.0.0  # Secure version
