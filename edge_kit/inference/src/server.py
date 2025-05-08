#!/usr/bin/env python3
"""
Bulo.CloudSentinel Edge Inference Server

This module provides a FastAPI server for running inference with various backends.
It supports TensorRT, ONNX Runtime, and TinyGrad backends with automatic fallback.
"""

import os
import sys
import logging
import json
import time
from typing import Dict, List, Any, Optional, Union, Tuple
from enum import Enum
import asyncio
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from pathlib import Path
import io
from PIL import Image
import base64

from fastapi import FastAPI, File, UploadFile, Form, HTTPException, BackgroundTasks, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
import uvicorn
from pydantic import BaseModel, Field

# Import inference backends
from inference_engine import InferenceEngine, ModelInfo, InferenceResult, DetectionResult

# Configure logging
log_level = os.environ.get("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("edge_inference")

# Create FastAPI app
app = FastAPI(
    title="Bulo.CloudSentinel Edge Inference",
    description="Edge inference server for Bulo.CloudSentinel",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create inference engine
model_repository = os.environ.get("MODEL_REPOSITORY", "/models")
inference_backend = os.environ.get("INFERENCE_BACKEND", "auto")
device_id = int(os.environ.get("DEVICE_ID", "0"))
max_batch_size = int(os.environ.get("MAX_BATCH_SIZE", "4"))

# Initialize inference engine
engine = InferenceEngine(
    model_repository=model_repository,
    backend=inference_backend,
    device_id=device_id,
    max_batch_size=max_batch_size,
)

# Thread pool for inference
executor = ThreadPoolExecutor(max_workers=4)

# API models
class InferenceRequest(BaseModel):
    model_name: str
    inputs: Dict[str, Any]
    parameters: Optional[Dict[str, Any]] = None

class InferenceResponse(BaseModel):
    model_name: str
    outputs: Dict[str, Any]
    metrics: Dict[str, float]

class ModelListResponse(BaseModel):
    models: List[ModelInfo]

class HealthResponse(BaseModel):
    status: str
    version: str
    backends: Dict[str, bool]

# API routes
@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check the health of the inference server."""
    return {
        "status": "ok",
        "version": "0.1.0",
        "backends": engine.get_available_backends(),
    }

@app.get("/models", response_model=ModelListResponse)
async def list_models():
    """List all available models."""
    models = await asyncio.to_thread(engine.list_models)
    return {"models": models}

@app.get("/models/{model_name}", response_model=ModelInfo)
async def get_model(model_name: str):
    """Get information about a specific model."""
    try:
        model_info = await asyncio.to_thread(engine.get_model_info, model_name)
        return model_info
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Model not found: {str(e)}")

@app.post("/infer/{model_name}", response_model=InferenceResponse)
async def infer(model_name: str, request: InferenceRequest):
    """Run inference with a specific model."""
    try:
        start_time = time.time()
        
        # Run inference
        result = await asyncio.to_thread(
            engine.infer,
            model_name,
            request.inputs,
            request.parameters,
        )
        
        # Calculate metrics
        inference_time = time.time() - start_time
        
        return {
            "model_name": model_name,
            "outputs": result.outputs,
            "metrics": {
                "inference_time": inference_time,
                "preprocessing_time": result.metrics.get("preprocessing_time", 0.0),
                "inference_time_ms": result.metrics.get("inference_time_ms", 0.0),
                "postprocessing_time": result.metrics.get("postprocessing_time", 0.0),
            },
        }
    except Exception as e:
        logger.error(f"Inference error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Inference error: {str(e)}")

@app.post("/detect", response_model=Dict[str, Any])
async def detect(
    file: UploadFile = File(...),
    model_name: str = Form("yolov10n"),
    confidence: float = Form(0.25),
    iou_threshold: float = Form(0.45),
):
    """
    Detect objects in an image using a YOLO model.
    
    Args:
        file: Image file
        model_name: Name of the YOLO model to use
        confidence: Confidence threshold (0-1)
        iou_threshold: IoU threshold for NMS (0-1)
        
    Returns:
        Detection results
    """
    try:
        # Read image
        contents = await file.read()
        image = Image.open(io.BytesIO(contents))
        
        # Convert to RGB if needed
        if image.mode != "RGB":
            image = image.convert("RGB")
        
        # Convert to numpy array
        image_np = np.array(image)
        
        # Run detection
        result = await asyncio.to_thread(
            engine.detect,
            model_name,
            image_np,
            confidence_threshold=confidence,
            iou_threshold=iou_threshold,
        )
        
        # Return results
        return {
            "model_name": model_name,
            "detections": result.detections,
            "metrics": {
                "preprocessing_time": result.metrics.get("preprocessing_time", 0.0),
                "inference_time_ms": result.metrics.get("inference_time_ms", 0.0),
                "postprocessing_time": result.metrics.get("postprocessing_time", 0.0),
            },
        }
    except Exception as e:
        logger.error(f"Detection error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Detection error: {str(e)}")

@app.post("/detect/rtsp", response_model=Dict[str, Any])
async def detect_rtsp(
    request: Dict[str, Any],
):
    """
    Detect objects in an RTSP stream frame.
    
    Args:
        request: JSON request with the following fields:
            - rtsp_url: RTSP URL
            - model_name: Name of the YOLO model to use
            - confidence: Confidence threshold (0-1)
            - iou_threshold: IoU threshold for NMS (0-1)
            - frame_data: Base64-encoded frame data (optional)
            
    Returns:
        Detection results
    """
    try:
        # Extract parameters
        rtsp_url = request.get("rtsp_url")
        model_name = request.get("model_name", "yolov10n")
        confidence = request.get("confidence", 0.25)
        iou_threshold = request.get("iou_threshold", 0.45)
        frame_data = request.get("frame_data")
        
        # If frame data is provided, use it
        if frame_data:
            # Decode base64 frame data
            frame_bytes = base64.b64decode(frame_data)
            image = Image.open(io.BytesIO(frame_bytes))
            
            # Convert to RGB if needed
            if image.mode != "RGB":
                image = image.convert("RGB")
            
            # Convert to numpy array
            image_np = np.array(image)
        else:
            # Capture frame from RTSP stream
            # This is a placeholder - in a real implementation, you would use OpenCV or similar
            raise HTTPException(status_code=400, detail="Frame data is required")
        
        # Run detection
        result = await asyncio.to_thread(
            engine.detect,
            model_name,
            image_np,
            confidence_threshold=confidence,
            iou_threshold=iou_threshold,
        )
        
        # Return results
        return {
            "model_name": model_name,
            "rtsp_url": rtsp_url,
            "detections": result.detections,
            "metrics": {
                "preprocessing_time": result.metrics.get("preprocessing_time", 0.0),
                "inference_time_ms": result.metrics.get("inference_time_ms", 0.0),
                "postprocessing_time": result.metrics.get("postprocessing_time", 0.0),
            },
        }
    except Exception as e:
        logger.error(f"RTSP detection error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"RTSP detection error: {str(e)}")

@app.get("/metrics")
async def metrics():
    """Get metrics from the inference server."""
    try:
        metrics_data = await asyncio.to_thread(engine.get_metrics)
        return JSONResponse(content=metrics_data)
    except Exception as e:
        logger.error(f"Metrics error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Metrics error: {str(e)}")

# Main function
def main():
    """Run the inference server."""
    # Load models
    engine.load_models()
    
    # Start server
    port = int(os.environ.get("PORT", "8001"))
    uvicorn.run(app, host="0.0.0.0", port=port)

if __name__ == "__main__":
    main()
