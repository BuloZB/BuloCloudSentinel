# Use official Python image as base
FROM python:3.13.2-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY ai_detection/requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy AI detection source code
COPY ai_detection/ ./ai_detection/

# Expose port
EXPOSE 8001

# Healthcheck endpoint
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:8001/health || exit 1

# Command to run the FastAPI app with Uvicorn
CMD ["uvicorn", "ai_detection.ai_model_management:router", "--host", "0.0.0.0", "--port", "8001", "--reload"]
