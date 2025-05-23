FROM python:3.10-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    gdal-bin \
    python3-gdal \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install 3D Tiles converter
RUN pip install --no-cache-dir 3d-tiles-converter

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create temporary directory
RUN mkdir -p /tmp/mapping_service

# Set environment variables
ENV PYTHONPATH=/app
ENV PYTHONUNBUFFERED=1

# Run the worker
CMD ["python", "-m", "mapping_service.worker_main"]
