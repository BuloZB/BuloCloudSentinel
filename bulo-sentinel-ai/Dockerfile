# Use official Python image as base
FROM python:3.13.3-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY bulo-sentinel-ai/requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy AI integrations source code
COPY bulo-sentinel-ai/ ./bulo-sentinel-ai/

# Expose port
EXPOSE 8002

# Healthcheck endpoint
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:8002/health || exit 1

# Command to run the FastAPI app with Uvicorn
CMD ["uvicorn", "bulo-sentinel-ai.main:app", "--host", "0.0.0.0", "--port", "8002"]
