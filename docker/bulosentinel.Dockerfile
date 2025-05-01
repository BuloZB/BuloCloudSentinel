FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    libopencv-dev \
    python3-opencv \
    ffmpeg \
    libsm6 \
    libxext6 \
    # OpenCL dependencies
    ocl-icd-opencl-dev \
    opencl-headers \
    clinfo \
    # CUDA dependencies (if needed)
    # nvidia-cuda-toolkit \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Install tinygrad and its dependencies
RUN pip install --no-cache-dir \
    tinygrad==0.9.0 \
    safetensors==0.4.3 \
    torch==2.2.2 \
    onnx==1.16.1 \
    onnxruntime==1.20.1 \
    tensorflow-lite==2.15.0

# Copy application code
COPY . .

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV ML_BACKEND=torch

# Expose port
EXPOSE 8000

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
