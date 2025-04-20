from fastapi import FastAPI
from backend.api import router as api_router

app = FastAPI(title="Bulo.Cloud Sentinel Backend API")

app.include_router(api_router, prefix="/api")
