FROM python:3.11-slim@sha256:d310a35c94e8adf1138b8b8f8b9e0a2eaa3c0396d3bfb2a3f3a1c471d1a3bab1

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN groupadd -r dockuser && useradd -r -g dockuser dockuser

# Copy requirements first to leverage Docker cache
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Create necessary directories
RUN mkdir -p /app/logs /app/config

# Copy application code
COPY . .

# Set permissions
RUN chown -R dockuser:dockuser /app

# Switch to non-root user
USER dockuser

# Set environment variables
ENV PYTHONPATH=/app
ENV DOCK_DRIVER_CONFIG=/app/config/config.yaml
ENV LOG_LEVEL=INFO
ENV ENVIRONMENT=production

# Expose port
EXPOSE 8060

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8060/health || exit 1

# Run the application with security options
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8060"]
