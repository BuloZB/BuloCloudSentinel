#!/bin/bash
set -e

# Bulo.CloudSentinel Edge Kit Installer
# This script installs and configures the Edge Kit on Jetson Orin Nano or Raspberry Pi 5

# Text formatting
BOLD='\033[1m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Print banner
echo -e "${BOLD}Bulo.CloudSentinel Edge Kit Installer${NC}"
echo "=================================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}Error: Please run as root${NC}"
  exit 1
fi

# Detect device type
detect_device_type() {
  if [ -f /etc/nv_tegra_release ]; then
    echo "jetson"
  elif grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "rpi"
  else
    echo "unknown"
  fi
}

DEVICE_TYPE=$(detect_device_type)

if [ "$DEVICE_TYPE" == "unknown" ]; then
  echo -e "${RED}Error: Unsupported device. This installer only works on Jetson Orin Nano or Raspberry Pi 5.${NC}"
  exit 1
fi

echo -e "${GREEN}Detected device type: ${BOLD}${DEVICE_TYPE}${NC}"

# Check for Docker and Docker Compose
check_docker() {
  if ! command -v docker &> /dev/null; then
    echo -e "${YELLOW}Docker not found. Installing Docker...${NC}"
    curl -fsSL https://get.docker.com | sh
  else
    echo -e "${GREEN}Docker is already installed.${NC}"
  fi

  if ! command -v docker compose &> /dev/null; then
    echo -e "${YELLOW}Docker Compose not found. Installing Docker Compose...${NC}"
    apt-get update
    apt-get install -y docker-compose-plugin
  else
    echo -e "${GREEN}Docker Compose is already installed.${NC}"
  fi
}

# Install dependencies
install_dependencies() {
  echo -e "${YELLOW}Installing dependencies...${NC}"
  apt-get update
  apt-get install -y curl jq git ca-certificates gnupg lsb-release

  if [ "$DEVICE_TYPE" == "jetson" ]; then
    # Jetson-specific dependencies
    apt-get install -y python3-pip libopenblas-dev
  elif [ "$DEVICE_TYPE" == "rpi" ]; then
    # Raspberry Pi-specific dependencies
    apt-get install -y python3-pip libopenblas-dev libatlas-base-dev
  fi

  echo -e "${GREEN}Dependencies installed.${NC}"
}

# Create directory structure
create_directories() {
  echo -e "${YELLOW}Creating directory structure...${NC}"
  
  # Create main directory
  mkdir -p /opt/bulocloud/edge_kit
  
  # Create subdirectories
  mkdir -p /opt/bulocloud/edge_kit/models
  mkdir -p /opt/bulocloud/edge_kit/config
  mkdir -p /opt/bulocloud/edge_kit/storage
  mkdir -p /opt/bulocloud/edge_kit/certs
  
  echo -e "${GREEN}Directory structure created.${NC}"
}

# Download Edge Kit files
download_edge_kit() {
  echo -e "${YELLOW}Downloading Edge Kit files...${NC}"
  
  # Clone or download the repository
  if [ -d "/opt/bulocloud/edge_kit/.git" ]; then
    # Git repository exists, pull latest changes
    cd /opt/bulocloud/edge_kit
    git pull
  else
    # Download the Edge Kit
    cd /opt/bulocloud
    git clone https://github.com/your-org/bulo-cloud-sentinel.git temp
    cp -r temp/edge_kit/* /opt/bulocloud/edge_kit/
    rm -rf temp
  fi
  
  echo -e "${GREEN}Edge Kit files downloaded.${NC}"
}

# Configure the Edge Kit
configure_edge_kit() {
  echo -e "${YELLOW}Configuring Edge Kit...${NC}"
  
  # Generate device ID if not exists
  if [ ! -f "/opt/bulocloud/edge_kit/config/device_id" ]; then
    DEVICE_ID="edge-$(cat /proc/sys/kernel/random/uuid | cut -d'-' -f1)"
    echo "$DEVICE_ID" > /opt/bulocloud/edge_kit/config/device_id
  else
    DEVICE_ID=$(cat /opt/bulocloud/edge_kit/config/device_id)
  fi
  
  # Create .env file
  cat > /opt/bulocloud/edge_kit/.env << EOF
DEVICE_ID=$DEVICE_ID
DEVICE_TYPE=$DEVICE_TYPE
INFERENCE_BACKEND=$([ "$DEVICE_TYPE" == "jetson" ] && echo "tensorrt" || echo "onnxruntime")
MAX_BATCH_SIZE=$([ "$DEVICE_TYPE" == "jetson" ] && echo "4" || echo "1")
LOG_LEVEL=INFO
EOF
  
  # Create default RTSP sources config if not exists
  if [ ! -f "/opt/bulocloud/edge_kit/config/rtsp_sources.yaml" ]; then
    cat > /opt/bulocloud/edge_kit/config/rtsp_sources.yaml << EOF
sources:
  - name: camera1
    url: rtsp://localhost:8554/camera1
    enabled: true
EOF
  fi
  
  echo -e "${GREEN}Edge Kit configured.${NC}"
}

# Start the Edge Kit
start_edge_kit() {
  echo -e "${YELLOW}Starting Edge Kit...${NC}"
  
  cd /opt/bulocloud/edge_kit
  docker compose pull
  docker compose up -d
  
  echo -e "${GREEN}Edge Kit started.${NC}"
}

# Create systemd service
create_service() {
  echo -e "${YELLOW}Creating systemd service...${NC}"
  
  cat > /etc/systemd/system/edge-kit.service << EOF
[Unit]
Description=Bulo.CloudSentinel Edge Kit
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/opt/bulocloud/edge_kit
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF
  
  systemctl daemon-reload
  systemctl enable edge-kit.service
  
  echo -e "${GREEN}Systemd service created.${NC}"
}

# Main installation process
main() {
  echo -e "${YELLOW}Starting installation...${NC}"
  
  check_docker
  install_dependencies
  create_directories
  download_edge_kit
  configure_edge_kit
  start_edge_kit
  create_service
  
  echo -e "${GREEN}${BOLD}Installation complete!${NC}"
  echo -e "Edge Kit is now running on your device."
  echo -e "Device ID: ${BOLD}$DEVICE_ID${NC}"
  echo -e "Management interface: ${BOLD}http://localhost:9090${NC}"
  echo ""
  echo -e "To view logs: ${BOLD}docker compose -f /opt/bulocloud/edge_kit/docker-compose.yml logs -f${NC}"
  echo -e "To stop: ${BOLD}systemctl stop edge-kit${NC}"
  echo -e "To start: ${BOLD}systemctl start edge-kit${NC}"
}

# Run the installation
main
