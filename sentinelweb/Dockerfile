FROM node:20-slim AS frontend-builder

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3 \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Clone OpenWebUI repository
RUN git clone https://github.com/open-webui/open-webui.git /app/open-webui

# Build frontend
WORKDIR /app/open-webui
RUN npm install
RUN npm run build

# Second stage: Python backend
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy OpenWebUI backend code
COPY --from=frontend-builder /app/open-webui/backend /app/backend
COPY --from=frontend-builder /app/open-webui/build /app/frontend/dist

# Copy SentinelWeb adaptation files
COPY backend/sentinel_web /app/backend/sentinel_web

# Copy requirements file and install dependencies
COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# Create necessary directories
RUN mkdir -p /app/backend/data

# Set environment variables
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV PORT=3000

# Expose port
EXPOSE 3000

# Start the application
CMD ["python", "-m", "backend.sentinel_web.main"]
