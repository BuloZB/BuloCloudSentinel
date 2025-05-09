FROM python:3.12-slim

WORKDIR /app

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements
COPY pyproject.toml .

# Install dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir .

# Copy application code
COPY . .

# Expose port
EXPOSE 8090

# Run the application
CMD ["uvicorn", "weather_guard.main:app", "--host", "0.0.0.0", "--port", "8090"]
