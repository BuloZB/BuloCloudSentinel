from fastapi import APIRouter
from backend.api.incident_timeline import router as incident_timeline_router
from dronecore.drone_command_telemetry_hub import router as drone_hub_router
from ai_detection.ai_model_management import router as ai_model_management_router
from backend.api.device_inventory import router as device_inventory_router
from backend.api.audit_log import router as audit_log_router
from backend.api.login import router as login_router
from backend.api.me import router as me_router
from backend.api.metrics import router as metrics_router
from backend.api.analyze_video import router as analyze_video_router
from dronecore.mission_planner_service import router as mission_planner_router
from backend.api.health import router as health_router
from backend.api.sensor_fusion import router as sensor_fusion_router
from backend.api.mesh_networking import router as mesh_networking_router
from backend.mission_planning.autonomous import router as autonomous_mission_router
from backend.api.anduril_lattice import router as anduril_lattice_router, initialize_adapter

router = APIRouter()

router.include_router(incident_timeline_router)
router.include_router(drone_hub_router)
router.include_router(ai_model_management_router)
router.include_router(device_inventory_router)
router.include_router(audit_log_router)
router.include_router(login_router)
router.include_router(me_router)
router.include_router(metrics_router)
router.include_router(analyze_video_router, prefix="/ai")
router.include_router(mission_planner_router, prefix="/missions")
router.include_router(health_router)
router.include_router(sensor_fusion_router)
router.include_router(mesh_networking_router, prefix="/mesh")
router.include_router(anduril_lattice_router)
router.include_router(autonomous_mission_router, prefix="/autonomous-missions")
