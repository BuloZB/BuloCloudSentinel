# Architecture Overview

Bulo.Cloud Sentinel is designed as a set of microservices that communicate over REST APIs and event streams, offering modularity and independent scaling.

```mermaid
flowchart LR
  subgraph Frontend
    FE1[React UI]
    FE2[AI Tools Panel]
    FE3[Incident Timeline]
    FE1 --> FE2
    FE1 --> FE3
  end

  subgraph Backend Services
    BE1[Auth & User Service]
    BE2[Incident API]
    BE3[Drone Hub API]
    BE4[Device Inventory API]
    BE5[AI Detection Service]
    BE6[AI Integrations Service]
    BE7[Audit Log Service]
  end

  subgraph Infrastructure
    DB[(PostgreSQL)]
    REDIS[(Redis)]
    MINIO[(MinIO)]
    PROM[(Prometheus)]
  end

  FE1 --> BE1
  FE1 --> BE2
  FE1 --> BE3
  FE1 --> BE4
  BE5 --> BE6
  BE6 --> BE7
  BE2 --> DB
  BE3 --> DB
  BE4 --> DB
  BE6 --> MINIO
  BE6 --> PROM
  BE7 --> MINIO
  BE1 --> REDIS
```

- **Frontend** handles all user interactions and renders UI components.
- **Backend Services** are independent FastAPI containers, each responsible for a specific domain.
- **AI Detection** uses custom ML models for inference.
- **AI Integrations** proxies to external AI APIs via adapter classes.
- **Audit Log** persists request/response metadata for compliance.
- **Infrastructure** components provide data persistence and observability.
