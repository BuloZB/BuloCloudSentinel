from fastapi import APIRouter

router = APIRouter()

from backend.api import setup, login, me, detections

router.include_router(setup.router, prefix="/setup", tags=["setup"])
router.include_router(login.router, prefix="/login", tags=["login"])
router.include_router(me.router, prefix="/me", tags=["me"])
router.include_router(detections.router, prefix="/detections", tags=["detections"])
