# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    gnuradio \
    gnuradio-dev \
    librtlsdr-dev \
    git \
    cmake \
    build-essential \
    pkg-config \
    libboost-all-dev \
    liblog4cpp5-dev \
    libspdlog-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install gr-kerberos
RUN git clone https://github.com/rfjohnso/gr-kerberos /tmp/gr-kerberos \
    && cd /tmp/gr-kerberos \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig \
    && rm -rf /tmp/gr-kerberos

# Create app directory
WORKDIR /app

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Install the package
RUN pip3 install -e .

# Create configuration directory
RUN mkdir -p /config

# Set default configuration path
ENV COUNTER_UAS_CONFIG=/config/counter_uas.yaml

# Copy default configuration
RUN mkdir -p /config && cp /app/config/counter_uas.yaml /config/

# Expose API port
EXPOSE 8000

# Run the application
CMD ["python3", "-m", "counter_uas.main", "--config", "/config/counter_uas.yaml"]
