# Bulo.CloudSentinel Edge Agent Container
# Multi-stage build for Rust application

# ===== Builder stage =====
FROM rust:1.73-slim-bullseye AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a new empty shell project
WORKDIR /app
RUN USER=root cargo new --bin edge_agent
WORKDIR /app/edge_agent

# Copy manifests
COPY ./Cargo.toml ./Cargo.lock ./

# Build dependencies - this is the caching Docker layer
RUN cargo build --release
RUN rm src/*.rs

# Copy source code
COPY ./src ./src

# Build for release
RUN touch src/main.rs && cargo build --release

# ===== Runtime stage =====
FROM debian:bullseye-slim

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install Docker CLI
RUN curl -fsSL https://download.docker.com/linux/debian/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/debian \
    $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null \
    && apt-get update \
    && apt-get install -y docker-ce-cli \
    && rm -rf /var/lib/apt/lists/*

# Copy the binary from builder
COPY --from=builder /app/edge_agent/target/release/edge_agent /usr/local/bin/edge_agent

# Create directories
RUN mkdir -p /config /storage /certs

# Set environment variables
ENV DEVICE_ID=edge-device-001
ENV DEVICE_TYPE=jetson
ENV MQTT_BROKER=mqtt://bulocloud-mqtt:1883
ENV MQTT_USERNAME=
ENV MQTT_PASSWORD=
ENV MQTT_CLIENT_ID=edge-device-001
ENV MQTT_TOPIC_PREFIX=bulocloud/edge
ENV VAULT_ADDR=http://bulocloud-vault:8200
ENV VAULT_TOKEN=
ENV VAULT_PATH=secret/edge
ENV UPDATE_CHECK_INTERVAL=3600
ENV LOG_LEVEL=INFO

# Expose port
EXPOSE 9090

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/edge_agent"]
