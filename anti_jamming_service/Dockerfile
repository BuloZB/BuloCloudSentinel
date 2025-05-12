FROM python:3.13.2-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libusb-1.0-0-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install GNU Radio dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnuradio \
    gnuradio-dev \
    python3-gnuradio \
    && rm -rf /var/lib/apt/lists/*

# Install RTL-SDR dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    rtl-sdr \
    librtlsdr-dev \
    && rm -rf /var/lib/apt/lists/*

# Install HackRF dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    hackrf \
    libhackrf-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Install KrakenSDR GNU Radio blocks
RUN git clone https://github.com/krakenrf/gr-krakensdr.git && \
    cd gr-krakensdr && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig && \
    cd ../.. && \
    rm -rf gr-krakensdr

# Install GNSS-SDR
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnss-sdr \
    && rm -rf /var/lib/apt/lists/*

# Copy application code
COPY . .

# Create necessary directories
RUN mkdir -p /app/config /app/logs /app/certs

# Set environment variables
ENV PYTHONPATH=/app
ENV ANTI_JAMMING_CONFIG=/app/config/config.yaml

# Expose API port
EXPOSE 8080

# Run the application
CMD ["python", "-m", "anti_jamming_service.main"]
