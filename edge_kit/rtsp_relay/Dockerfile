# Bulo.CloudSentinel RTSP Relay Container
# Multi-stage build for Jetson Orin Nano and Raspberry Pi 5

ARG DEVICE_TYPE=jetson

# ===== Base image for Jetson =====
FROM nvcr.io/nvidia/l4t-base:r35.4.1 AS jetson-base
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    git \
    curl \
    wget \
    ca-certificates \
    python3 \
    python3-pip \
    python3-dev \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    libavfilter-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ===== Base image for Raspberry Pi =====
FROM python:3.9-slim-bullseye AS rpi-base
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    git \
    curl \
    wget \
    ca-certificates \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    libavfilter-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ===== Common build stage =====
FROM ${DEVICE_TYPE}-base AS build

# Install Go
RUN curl -L https://go.dev/dl/go1.21.0.linux-arm64.tar.gz | tar -C /usr/local -xzf -
ENV PATH=$PATH:/usr/local/go/bin

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Clone and build rtsp-simple-server
WORKDIR /build
RUN git clone https://github.com/aler9/rtsp-simple-server.git && \
    cd rtsp-simple-server && \
    go build -o rtsp-simple-server

# Clone and build WebRTC relay
WORKDIR /build
RUN git clone https://github.com/pion/webrtc.git && \
    cd webrtc/examples/broadcast && \
    go build -o webrtc-broadcast

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    fastapi \
    uvicorn \
    pydantic \
    prometheus-client \
    python-multipart \
    httpx \
    pyyaml \
    av \
    aiortc

# ===== Final image =====
FROM ${DEVICE_TYPE}-base AS final

# Copy binaries from build stage
COPY --from=build /build/rtsp-simple-server/rtsp-simple-server /usr/local/bin/
COPY --from=build /build/webrtc/examples/broadcast/webrtc-broadcast /usr/local/bin/

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    fastapi \
    uvicorn \
    pydantic \
    prometheus-client \
    python-multipart \
    httpx \
    pyyaml \
    av \
    aiortc

# Create directories
RUN mkdir -p /config /storage /app

# Copy application code
COPY ./src /app/
WORKDIR /app

# Set environment variables
ENV PYTHONPATH=/app
ENV RTSP_SOURCES_CONFIG=/config/rtsp_sources.yaml
ENV HLS_SEGMENT_DURATION=2
ENV WEBRTC_ICE_SERVERS=stun:stun.l.google.com:19302
ENV STREAM_BUFFER_SIZE=1024
ENV LOG_LEVEL=INFO

# Expose ports
EXPOSE 8554  # RTSP server
EXPOSE 8888  # HTTP server for HLS
EXPOSE 8889  # WebRTC signaling

# Set entrypoint
ENTRYPOINT ["python3", "/app/server.py"]
