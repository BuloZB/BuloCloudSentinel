# SentinelWeb Installation Guide

This guide provides instructions for installing and running SentinelWeb.

## Prerequisites

- Python 3.9 or higher
- Git
- Docker and Docker Compose (optional, for containerized deployment)

## Installation

### Option 1: Local Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/sentinelweb.git
   cd sentinelweb
   ```

2. Run the setup script:
   - On Windows:
     ```bash
     run.bat
     ```
   - On Linux/macOS:
     ```bash
     chmod +x run.sh
     ./run.sh
     ```

3. Access the web interface at http://localhost:8080

### Option 2: Docker Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/sentinelweb.git
   cd sentinelweb
   ```

2. Configure environment variables in `docker-compose.yml`:
   ```yaml
   environment:
     - SENTINEL_API_URL=http://bulocloud-sentinel-api:8000
     - SENTINEL_API_TOKEN=your-api-token
     - RTMP_SERVER=rtmp://rtmp-server:1935
     - SECRET_KEY=your-secret-key
   ```

3. Start the services:
   ```bash
   docker-compose up -d
   ```

4. Access the web interface at http://localhost:8080

## Configuration

SentinelWeb can be configured using environment variables:

- `SENTINEL_API_URL`: URL of the BuloCloudSentinel API (default: http://localhost:8000)
- `SENTINEL_API_TOKEN`: API token for authentication (default: empty)
- `RTMP_SERVER`: URL of the RTMP server for video streaming (default: rtmp://localhost:1935)
- `SECRET_KEY`: Secret key for session encryption (default: sentinelweb-secret-key)

## Testing

Run the tests to verify the installation:

- On Windows:
  ```bash
  run_tests.bat
  ```
- On Linux/macOS:
  ```bash
  chmod +x run_tests.sh
  ./run_tests.sh
  ```

## Troubleshooting

### Common Issues

1. **OpenWebUI not found**
   - Make sure you have internet access to clone the OpenWebUI repository
   - Check if the OpenWebUI directory exists in the project root

2. **Connection to BuloCloudSentinel API failed**
   - Verify that the BuloCloudSentinel API is running
   - Check the `SENTINEL_API_URL` environment variable

3. **Video streaming not working**
   - Verify that the RTMP server is running
   - Check the `RTMP_SERVER` environment variable

### Logs

Check the logs for more information:

- Local installation: Console output
- Docker installation: `docker-compose logs sentinelweb`

## Support

For support, please contact the BuloCloud support team or open an issue on the GitHub repository.
