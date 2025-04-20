# 🚀 Bulo.Cloud Sentinel

![Architecture](https://img.shields.io/badge/Architecture-Clean%20Architecture-blue)
![Language](https://img.shields.io/badge/Language-Python%20%26%20JavaScript-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Production%20Ready-brightgreen)

---

## Overview

Bulo.Cloud Sentinel is a **highly modular, scalable, and secure** cloud-based system designed for **real-time video streaming**, **AI-powered object detection**, and **dynamic alerting**. Built with enterprise-grade principles, it emphasizes:

- **Effective separation of layers and components**
- **Robust security and authentication**
- **Extensible and maintainable architecture**
- **Optimized performance and scalability**

---

## 🏗️ Architecture

The system follows a **layered architecture** with strict separation of concerns:

- **Frontend:** React SPA with Tailwind CSS, JWT authentication, and real-time HLS video streaming.
- **Backend API:** FastAPI with domain-driven design, repository pattern, JWT security, and async database access.
- **AI Detection Service:** Containerized Python service running YOLOv8 for real-time object detection.
- **RTMP Server:** Nginx with RTMP module for video ingestion and HLS streaming.
- **Configuration:** Externalized and validated via Pydantic.
- **Deployment:** Docker Compose orchestration for seamless multi-service deployment.

---

## ⚙️ Features

- **Authentication & Authorization:** Secure JWT-based login and user management.
- **Real-Time Video Streaming:** RTMP ingestion with HLS playback in frontend.
- **AI-Powered Detection:** Scalable AI service for object detection on video streams.
- **Extensible API:** Clean, well-documented RESTful endpoints.
- **Robust Configuration:** Environment-based, validated, and secure.
- **Comprehensive Testing:** Unit, integration, and E2E test readiness.
- **Production Ready:** Security best practices, async performance, and modular design.

---

## 📚 Documentation

For detailed technical documentation, architecture diagrams, and development guidelines, see [DOCUMENTATION.md](./DOCUMENTATION.md).

---

## 🚀 Getting Started

### Prerequisites

- Docker & Docker Compose
- Python 3.11+ (for local development)
- Node.js 18+ (for frontend development)

### Running the System

```bash
docker compose up -d --build
```

This command builds and starts all services: frontend, backend, AI detection, and RTMP server.

---

## 🛠️ Development

- Backend code follows **Clean Architecture** with domain, application, infrastructure, and API layers.
- Frontend uses React with Tailwind CSS and React Router.
- AI Detection service is containerized for scalability.
- RTMP server configured for secure streaming and HLS support.

---

## 🔐 Security

- Passwords hashed with bcrypt.
- JWT tokens with configurable expiration.
- Input validation with Pydantic.
- Recommendations for rate limiting and brute force protection.

---

## 📈 Future Enhancements

- Rate limiting and security middleware.
- Advanced AI detection features.
- Cloud storage integration.
- Telegram alerts and geospatial mapping.
- CI/CD pipeline integration.

---

## 🤝 Contributing

Contributions are welcome! Please open issues or pull requests on the GitHub repository.

---

## 📄 License

This project is licensed under the MIT License.

---

*Crafted with ❤️ for high-quality enterprise systems.*
