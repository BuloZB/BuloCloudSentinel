from fastapi import APIRouter, UploadFile, File, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import os
import shutil
from datetime import datetime
from backend.api.dependencies import get_current_user

router = APIRouter(
    prefix="/ai-model-management",
    tags=["AI Model Management & Training"]
)

MODEL_STORAGE_PATH = "/models"  # MinIO or local volume mount point

class ModelMetadata(BaseModel):
    model_name: str
    version: str
    active: bool
    upload_time: datetime
    description: Optional[str] = None

# In-memory store for demo purposes
models_db = []

@router.post("/upload", response_model=ModelMetadata)
async def upload_model(file: UploadFile = File(...), current_user: str = Depends(get_current_user)):
    # Validate file type and structure (simplified)
    if not file.filename.endswith((".pt", ".pth", ".zip", ".tar.gz")):
        raise HTTPException(status_code=400, detail="Invalid model file type")

    save_path = os.path.join(MODEL_STORAGE_PATH, file.filename)
    with open(save_path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)

    metadata = ModelMetadata(
        model_name=file.filename,
        version="1.0.0",
        active=False,
        upload_time=datetime.utcnow(),
        description="Uploaded model"
    )
    models_db.append(metadata)
    return metadata

@router.get("/models", response_model=List[ModelMetadata])
async def list_models(current_user: str = Depends(get_current_user)):
    return models_db

@router.post("/activate/{model_name}")
async def activate_model(model_name: str, current_user: str = Depends(get_current_user)):
    found = False
    for model in models_db:
        if model.model_name == model_name:
            model.active = True
            found = True
        else:
            model.active = False
    if not found:
        raise HTTPException(status_code=404, detail="Model not found")
    return {"status": f"Model {model_name} activated"}

@router.post("/rollback/{model_name}")
async def rollback_model(model_name: str, current_user: str = Depends(get_current_user)):
    # Simplified rollback logic
    for model in models_db:
        if model.model_name == model_name:
            model.active = True
        else:
            model.active = False
    return {"status": f"Rolled back to model {model_name}"}
