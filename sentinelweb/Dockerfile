FROM node:18-alpine AS frontend-builder

WORKDIR /app

# Copy OpenWebUI frontend files
COPY temp-openwebui/package*.json ./
RUN npm ci

COPY temp-openwebui/. ./
RUN npm run build

# Second stage: Python backend
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy OpenWebUI backend requirements
COPY temp-openwebui/backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy OpenWebUI backend code
COPY temp-openwebui/backend /app/backend

# Copy built frontend
COPY --from=frontend-builder /app/build /app/frontend/dist

# Create necessary directories
RUN mkdir -p /app/backend/data

# Copy SentinelWeb adaptation files
COPY backend/sentinel_adapter /app/backend/open_webui/sentinel_adapter

# Set environment variables
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV PORT=3000

# Expose port
EXPOSE 3000

# Start the application
CMD ["python", "-m", "backend.open_webui.main"]
