"""
API routes for the inference service.

This module provides API endpoints for running inference with different ML backends.
"""

import logging
import os
import sys
import base64
import io
from typing import Dict, Any, Optional, List
import numpy as np
from fastapi import APIRouter, Depends, HTTPException, File, UploadFile, Form, BackgroundTasks, Request
from pydantic import BaseModel

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(
    prefix="/inference",
    tags=["inference"],
    responses={404: {"description": "Not found"}},
)


# Models
class InferenceRequest(BaseModel):
    """Inference request."""
    
    model_name: str
    inputs: Dict[str, List[float]]
    input_shapes: Optional[Dict[str, List[int]]] = None


class InferenceResponse(BaseModel):
    """Inference response."""
    
    outputs: Dict[str, List[float]]
    inference_time: float
    success: bool
    error: Optional[str] = None


class ImageInferenceRequest(BaseModel):
    """Image inference request."""
    
    model_name: str
    image_base64: str
    input_name: str = "input"


class StartTaskRequest(BaseModel):
    """Start task request."""
    
    model_name: str
    interval: float = 1.0


class StartTaskResponse(BaseModel):
    """Start task response."""
    
    task_id: str
    success: bool
    error: Optional[str] = None


class StopTaskRequest(BaseModel):
    """Stop task request."""
    
    task_id: str


class StopTaskResponse(BaseModel):
    """Stop task response."""
    
    success: bool
    error: Optional[str] = None


class BackendInfoResponse(BaseModel):
    """Backend info response."""
    
    backend: str
    device: str
    models: List[str]


class ModelInfoResponse(BaseModel):
    """Model info response."""
    
    name: str
    backend: str
    device: str
    input_shapes: Dict[str, List[int]]
    output_shapes: Dict[str, List[int]]


# Helper functions
def get_inference_service(request: Request):
    """Get the inference service from the app state."""
    return request.app.state.inference_service


# Routes
@router.post("/run", response_model=InferenceResponse)
async def run_inference(
    request: InferenceRequest,
    inference_service = Depends(get_inference_service)
):
    """
    Run inference on a model.
    
    Args:
        request: Inference request
        inference_service: Inference service
        
    Returns:
        Inference response
    """
    try:
        # Convert inputs to numpy arrays
        inputs = {}
        for name, values in request.inputs.items():
            # Get shape if provided
            shape = None
            if request.input_shapes and name in request.input_shapes:
                shape = tuple(request.input_shapes[name])
            
            # Convert to numpy array
            array = np.array(values, dtype=np.float32)
            
            # Reshape if shape is provided
            if shape:
                array = array.reshape(shape)
            
            inputs[name] = array
        
        # Run inference
        result = await inference_service.run_inference(request.model_name, inputs)
        
        # Check if inference was successful
        if not result["success"]:
            raise HTTPException(status_code=500, detail=result["error"])
        
        # Convert outputs to lists
        outputs = {}
        for name, array in result["outputs"].items():
            outputs[name] = array.flatten().tolist()
        
        # Return response
        return InferenceResponse(
            outputs=outputs,
            inference_time=result["inference_time"],
            success=True
        )
    except Exception as e:
        logger.error(f"Error running inference: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/image", response_model=InferenceResponse)
async def run_image_inference(
    request: ImageInferenceRequest,
    inference_service = Depends(get_inference_service)
):
    """
    Run inference on an image.
    
    Args:
        request: Image inference request
        inference_service: Inference service
        
    Returns:
        Inference response
    """
    try:
        # Decode base64 image
        try:
            image_data = base64.b64decode(request.image_base64)
            image = np.array(bytearray(image_data), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            
            if image is None:
                raise ValueError("Failed to decode image")
        except Exception as e:
            logger.error(f"Error decoding image: {str(e)}")
            raise HTTPException(status_code=400, detail=f"Error decoding image: {str(e)}")
        
        # Preprocess image
        # This is a simplified implementation for demonstration purposes
        # In a real implementation, you would need to preprocess the image based on the model requirements
        image = cv2.resize(image, (224, 224))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32) / 255.0
        image = np.expand_dims(image, axis=0)
        image = np.transpose(image, (0, 3, 1, 2))  # NCHW format for PyTorch models
        
        # Run inference
        result = await inference_service.run_inference(
            request.model_name,
            {request.input_name: image}
        )
        
        # Check if inference was successful
        if not result["success"]:
            raise HTTPException(status_code=500, detail=result["error"])
        
        # Convert outputs to lists
        outputs = {}
        for name, array in result["outputs"].items():
            outputs[name] = array.flatten().tolist()
        
        # Return response
        return InferenceResponse(
            outputs=outputs,
            inference_time=result["inference_time"],
            success=True
        )
    except Exception as e:
        logger.error(f"Error running image inference: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/upload", response_model=InferenceResponse)
async def run_upload_inference(
    model_name: str = Form(...),
    file: UploadFile = File(...),
    inference_service = Depends(get_inference_service)
):
    """
    Run inference on an uploaded file.
    
    Args:
        model_name: Name of the model to use
        file: Uploaded file
        inference_service: Inference service
        
    Returns:
        Inference response
    """
    try:
        # Read file
        contents = await file.read()
        
        # Check file type
        if file.content_type.startswith("image/"):
            # Process image
            image = np.array(bytearray(contents), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            
            if image is None:
                raise ValueError("Failed to decode image")
            
            # Preprocess image
            # This is a simplified implementation for demonstration purposes
            # In a real implementation, you would need to preprocess the image based on the model requirements
            image = cv2.resize(image, (224, 224))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = image.astype(np.float32) / 255.0
            image = np.expand_dims(image, axis=0)
            image = np.transpose(image, (0, 3, 1, 2))  # NCHW format for PyTorch models
            
            # Run inference
            result = await inference_service.run_inference(
                model_name,
                {"input": image}
            )
        else:
            # Unsupported file type
            raise HTTPException(status_code=400, detail=f"Unsupported file type: {file.content_type}")
        
        # Check if inference was successful
        if not result["success"]:
            raise HTTPException(status_code=500, detail=result["error"])
        
        # Convert outputs to lists
        outputs = {}
        for name, array in result["outputs"].items():
            outputs[name] = array.flatten().tolist()
        
        # Return response
        return InferenceResponse(
            outputs=outputs,
            inference_time=result["inference_time"],
            success=True
        )
    except Exception as e:
        logger.error(f"Error running upload inference: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/task/start", response_model=StartTaskResponse)
async def start_inference_task(
    request: StartTaskRequest,
    background_tasks: BackgroundTasks,
    inference_service = Depends(get_inference_service)
):
    """
    Start an inference task.
    
    Args:
        request: Start task request
        background_tasks: Background tasks
        inference_service: Inference service
        
    Returns:
        Start task response
    """
    try:
        # This is a simplified implementation for demonstration purposes
        # In a real implementation, you would need to define input generators and result handlers
        
        # Define input generator
        async def input_generator():
            # Generate random input
            return {"input": np.random.random((1, 3, 224, 224)).astype(np.float32)}
        
        # Define result handler
        async def result_handler(result):
            # Log result
            logger.info(f"Inference result: {result}")
        
        # Start task
        task_id = await inference_service.start_inference_task(
            request.model_name,
            input_generator,
            result_handler,
            request.interval
        )
        
        # Return response
        return StartTaskResponse(
            task_id=task_id,
            success=True
        )
    except Exception as e:
        logger.error(f"Error starting inference task: {str(e)}")
        return StartTaskResponse(
            task_id="",
            success=False,
            error=str(e)
        )


@router.post("/task/stop", response_model=StopTaskResponse)
async def stop_inference_task(
    request: StopTaskRequest,
    inference_service = Depends(get_inference_service)
):
    """
    Stop an inference task.
    
    Args:
        request: Stop task request
        inference_service: Inference service
        
    Returns:
        Stop task response
    """
    try:
        # Stop task
        success = await inference_service.stop_inference_task(request.task_id)
        
        # Return response
        return StopTaskResponse(
            success=success,
            error=None if success else f"Task {request.task_id} not found"
        )
    except Exception as e:
        logger.error(f"Error stopping inference task: {str(e)}")
        return StopTaskResponse(
            success=False,
            error=str(e)
        )


@router.get("/task/list")
async def list_inference_tasks(
    inference_service = Depends(get_inference_service)
):
    """
    List active inference tasks.
    
    Args:
        inference_service: Inference service
        
    Returns:
        List of active tasks
    """
    try:
        # Get active tasks
        tasks = await inference_service.get_active_tasks()
        
        # Return tasks
        return tasks
    except Exception as e:
        logger.error(f"Error listing inference tasks: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/backend/info", response_model=BackendInfoResponse)
async def get_backend_info(
    inference_service = Depends(get_inference_service)
):
    """
    Get information about the current backend.
    
    Args:
        inference_service: Inference service
        
    Returns:
        Backend information
    """
    try:
        # Get backend info
        info = inference_service.get_backend_info()
        
        # Return info
        return BackendInfoResponse(
            backend=info["backend"],
            device=info["device"],
            models=info["models"]
        )
    except Exception as e:
        logger.error(f"Error getting backend info: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/backend/list")
async def list_available_backends(
    inference_service = Depends(get_inference_service)
):
    """
    List available ML backends.
    
    Args:
        inference_service: Inference service
        
    Returns:
        List of available backends
    """
    try:
        # Get available backends
        backends = inference_service.get_available_backends()
        
        # Return backends
        return backends
    except Exception as e:
        logger.error(f"Error listing available backends: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/model/info/{model_name}", response_model=ModelInfoResponse)
async def get_model_info(
    model_name: str,
    inference_service = Depends(get_inference_service)
):
    """
    Get information about a model.
    
    Args:
        model_name: Name of the model
        inference_service: Inference service
        
    Returns:
        Model information
    """
    try:
        # Get model info
        info = inference_service.get_model_info(model_name)
        
        # Check if model exists
        if info is None:
            raise HTTPException(status_code=404, detail=f"Model {model_name} not found")
        
        # Return info
        return ModelInfoResponse(
            name=info["name"],
            backend=info["backend"],
            device=info["device"],
            input_shapes={name: list(shape) for name, shape in info["input_shapes"].items()},
            output_shapes={name: list(shape) for name, shape in info["output_shapes"].items()}
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting model info: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/model/list")
async def list_available_models(
    inference_service = Depends(get_inference_service)
):
    """
    List available models.
    
    Args:
        inference_service: Inference service
        
    Returns:
        List of available models
    """
    try:
        # Get backend info
        info = inference_service.get_backend_info()
        
        # Return models
        return info["models"]
    except Exception as e:
        logger.error(f"Error listing available models: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
