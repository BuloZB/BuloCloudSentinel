# Voice & Gesture Co-Pilot Module for Bulo.Cloud Sentinel

<div align="center">
  <img src="../docs/images/voice_gesture_copilot_logo.png" alt="Voice & Gesture Co-Pilot Logo" width="200"/>
  <h3>Hands-Free Drone Control</h3>
  <p>Control drones through voice commands and hand gestures for enhanced operational efficiency</p>
</div>

## 🌟 Overview

The Voice & Gesture Co-Pilot module enables hands-free drone control through voice commands and hand gestures, enhancing operational efficiency in field conditions. It integrates with the Bulo.Cloud Sentinel platform to provide a seamless and intuitive control experience.

## ✨ Features

- **🎤 Voice Recognition**: On-device Automatic Speech Recognition (ASR) using Whisper-cpp tiny-int8 model
- **👋 Gesture Recognition**: Full-body gesture tracking using MediaPipe Holistic
- **🧠 Intent Classification**: Natural language understanding with Rasa NLU
- **📊 Mission Graph**: Graph-based command sequence system for complex operations
- **📱 User Interface**: Non-intrusive transcript overlay and gesture recognition indicators
- **⚙️ Configuration**: Customizable gestures and voice command sensitivity
- **📊 Performance Monitoring**: Real-time monitoring of recognition accuracy and latency
- **🔌 API Integration**: RESTful API and WebSocket for real-time communication
- **🔒 Security**: JWT authentication and secure communication

## 🚀 Technical Stack

### Voice Recognition
- **Whisper-cpp**: Tiny-int8 model for on-device ASR
- **WebRTC VAD**: Voice activity detection for improved recognition
- **Offline Operation**: Works without internet connection

### Gesture Recognition
- **MediaPipe Holistic**: Full-body pose, face, and hand tracking
- **Predefined Gestures**: Common drone operations (takeoff, land, move, return-to-home)
- **Confidence Scoring**: Gesture recognition with confidence levels

### Command Processing
- **Rasa NLU**: Intent classification and entity extraction
- **Mission Graph**: Graph-based command sequence system
- **Fallback Mechanisms**: Handling ambiguous commands

### User Interface
- **Transcript Overlay**: Non-intrusive display of recognized commands
- **Gesture Indicators**: Visual feedback for recognized gestures
- **Status Panel**: Recognition confidence levels and system status
- **Configuration Panel**: Customizable gestures and voice command sensitivity

## 📋 Requirements

- Python 3.10+
- FastAPI
- MediaPipe
- Whisper-cpp
- Rasa NLU
- OpenCV
- NumPy
- SoundDevice
- WebRTC VAD

## 🛠️ Installation

### Using Docker (Recommended)

1. Clone the repository:
```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/voice_gesture_copilot
```

2. Create a `.env` file with the required environment variables:
```bash
SENTINEL_API_TOKEN=your_api_token
JWT_SECRET_KEY=your_jwt_secret
```

3. Build and run the Docker container:
```bash
docker-compose up -d
```

### Manual Installation

1. Clone the repository:
```bash
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/voice_gesture_copilot
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set environment variables:
```bash
export SENTINEL_API_URL=http://bulocloud-sentinel-api:8000
export SENTINEL_API_TOKEN=your_api_token
export JWT_SECRET_KEY=your_jwt_secret
```

4. Run the application:
```bash
uvicorn voice_gesture_copilot.main:app --host 0.0.0.0 --port 8080
```

## 📚 API Documentation

The Voice & Gesture Co-Pilot module provides a RESTful API for voice and gesture recognition, command processing, and configuration management. The API documentation is available at `http://localhost:8080/docs` when the module is running.

### Voice Recognition Endpoints

- `POST /api/voice/recognize`: Recognize voice commands from audio data
- `POST /api/voice/process`: Process a voice command
- `POST /api/voice/recognize-and-process`: Recognize and process voice commands from audio data
- `GET /api/voice/history`: Get voice command history
- `GET /api/voice/status`: Get voice recognition status

### Gesture Recognition Endpoints

- `POST /api/gesture/recognize`: Recognize gestures from image data
- `POST /api/gesture/recognize-base64`: Recognize gestures from base64-encoded image data
- `POST /api/gesture/process`: Process a recognized gesture
- `POST /api/gesture/recognize-and-process`: Recognize and process gestures from image data
- `POST /api/gesture/recognize-and-process-base64`: Recognize and process gestures from base64-encoded image data
- `GET /api/gesture/history`: Get gesture command history
- `GET /api/gesture/status`: Get gesture recognition status

### Command Endpoints

- `POST /api/command/execute`: Execute a drone command
- `GET /api/command/history`: Get command execution history
- `GET /api/command/drone-history`: Get drone command execution history
- `GET /api/command/drones`: Get a list of available drones
- `GET /api/command/drones/{drone_id}`: Get information about a specific drone
- `GET /api/command/drones/{drone_id}/telemetry`: Get telemetry data for a drone
- `GET /api/command/status`: Get command service status
- `GET /api/command/drone-service-status`: Get drone service status

### Configuration Endpoints

- `GET /api/config/voice-commands`: Get voice command definitions
- `POST /api/config/voice-commands`: Update voice command definitions
- `GET /api/config/gestures`: Get gesture definitions
- `POST /api/config/gestures`: Update gesture definitions
- `GET /api/config/performance`: Get performance settings
- `POST /api/config/performance`: Update performance settings
- `GET /api/config/all`: Get all configuration settings

## 📊 Performance

The Voice & Gesture Co-Pilot module is designed to achieve high performance in field conditions:

- **Recognition Accuracy**: ≥95% intent recognition accuracy in standard operating conditions
- **Latency**: End-to-end latency under 400ms from command to drone response
- **Resource Usage**: Optimized for minimal CPU/GPU usage to preserve drone battery life

## 🧪 Testing

The module includes comprehensive test suites for unit testing, integration testing, and performance testing:

```bash
# Run all tests
pytest

# Run unit tests
pytest voice_gesture_copilot/tests/unit/

# Run integration tests
pytest voice_gesture_copilot/tests/integration/

# Run performance tests
pytest voice_gesture_copilot/tests/performance/
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.
