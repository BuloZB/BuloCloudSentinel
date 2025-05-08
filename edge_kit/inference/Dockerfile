# Bulo.CloudSentinel Edge Inference Container
# Multi-stage build for Jetson Orin Nano and Raspberry Pi 5

ARG DEVICE_TYPE=jetson

# ===== Base image for Jetson =====
FROM nvcr.io/nvidia/l4t-tensorrt:8.6.1-runtime AS jetson-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    libopenblas-dev \
    libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for Jetson
RUN pip3 install --no-cache-dir \
    numpy \
    pillow \
    pycuda \
    onnx \
    onnxruntime-gpu \
    tritonclient[all]

# ===== Base image for Raspberry Pi =====
FROM python:3.9-slim-bullseye AS rpi-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    libopenblas-dev \
    libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for Raspberry Pi
RUN pip3 install --no-cache-dir \
    numpy \
    pillow \
    onnx \
    onnxruntime \
    tritonclient[all]

# ===== TinyGrad installation (common) =====
FROM ${DEVICE_TYPE}-base AS tinygrad-install
RUN pip3 install --no-cache-dir \
    tinygrad \
    safetensors

# ===== Final image =====
FROM ${DEVICE_TYPE}-base AS final

# Copy TinyGrad from the tinygrad-install stage
COPY --from=tinygrad-install /usr/local/lib/python3.9/site-packages/tinygrad /usr/local/lib/python3.9/site-packages/tinygrad
COPY --from=tinygrad-install /usr/local/lib/python3.9/site-packages/safetensors /usr/local/lib/python3.9/site-packages/safetensors

# Install additional dependencies
RUN pip3 install --no-cache-dir \
    fastapi \
    uvicorn \
    pydantic \
    prometheus-client \
    python-multipart \
    httpx

# Create directories
RUN mkdir -p /models /config /app

# Copy application code
COPY ./src /app/
WORKDIR /app

# Set environment variables
ENV PYTHONPATH=/app
ENV MODEL_REPOSITORY=/models
ENV INFERENCE_BACKEND=auto
ENV METRICS_PORT=8002
ENV LOG_LEVEL=INFO

# Expose ports
EXPOSE 8001
EXPOSE 8002

# Set entrypoint
ENTRYPOINT ["python3", "/app/server.py"]
