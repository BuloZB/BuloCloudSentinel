# Deployment Guide for Drone Show Microservice

This document provides instructions for deploying the Drone Show microservice in various environments.

## Prerequisites

- Docker and Docker Compose (for local development and testing)
- Kubernetes cluster (for production deployment)
- PostgreSQL database
- Redis cache
- MinIO object storage
- Access to the Bulo.Cloud Sentinel API

## Local Development

### Using Docker Compose

The easiest way to run the Drone Show microservice locally is using Docker Compose:

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/drone_show_service

# Start the services
docker-compose up -d

# Initialize the database
docker-compose exec api python -m drone_show_service.init_db

# Generate sample data (optional)
docker-compose exec api python -m drone_show_service.generate_sample_data
```

This will start the following services:

- API service (FastAPI application)
- PostgreSQL database
- Redis cache
- MinIO object storage
- RTMP server for video streaming

The API will be available at http://localhost:8000.

### Using Python Directly

You can also run the Drone Show microservice directly using Python:

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/drone_show_service

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export DATABASE_URL=postgresql+asyncpg://postgres:postgres@localhost:5432/drone_show
export REDIS_URL=redis://localhost:6379/0
export MINIO_URL=localhost:9000
export MINIO_ACCESS_KEY=minioadmin
export MINIO_SECRET_KEY=minioadmin
export MINIO_BUCKET=drone-show
export SENTINEL_API_URL=http://localhost:8000
export SENTINEL_API_TOKEN=your-token
export RTMP_SERVER=rtmp://localhost:1935

# Initialize the database
python -m drone_show_service.init_db

# Generate sample data (optional)
python -m drone_show_service.generate_sample_data

# Start the API
python -m drone_show_service.run
```

The API will be available at http://localhost:8000.

## Production Deployment

### Using Kubernetes

The Drone Show microservice can be deployed to a Kubernetes cluster using the provided Kubernetes manifests:

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/drone_show_service

# Create the namespace
kubectl create namespace bulocloud-sentinel

# Create the secrets
kubectl create secret generic drone-show-secrets \
  --namespace bulocloud-sentinel \
  --from-literal=database-url=postgresql+asyncpg://postgres:postgres@db-drone-show.bulocloud-sentinel.svc.cluster.local:5432/drone_show \
  --from-literal=redis-url=redis://redis-drone-show.bulocloud-sentinel.svc.cluster.local:6379/0 \
  --from-literal=minio-access-key=minioadmin \
  --from-literal=minio-secret-key=minioadmin \
  --from-literal=sentinel-api-token=your-token

# Apply the Kubernetes manifests
kubectl apply -f kubernetes/
```

This will deploy the following resources:

- Deployment for the API service
- Service for the API service
- ConfigMap for configuration
- Secret for sensitive data
- Ingress for external access

The API will be available at https://bulocloud-sentinel.example.com/api/drone-show.

### Using Helm

A Helm chart for the Drone Show microservice is also available:

```bash
# Clone the repository
git clone https://github.com/BuloZB/BuloCloudSentinel.git
cd BuloCloudSentinel/drone_show_service

# Install the Helm chart
helm install drone-show kubernetes/helm \
  --namespace bulocloud-sentinel \
  --create-namespace \
  --set database.url=postgresql+asyncpg://postgres:postgres@db-drone-show.bulocloud-sentinel.svc.cluster.local:5432/drone_show \
  --set redis.url=redis://redis-drone-show.bulocloud-sentinel.svc.cluster.local:6379/0 \
  --set minio.accessKey=minioadmin \
  --set minio.secretKey=minioadmin \
  --set sentinel.apiToken=your-token
```

## Environment Variables

The Drone Show microservice can be configured using the following environment variables:

| Variable | Description | Default |
| --- | --- | --- |
| `DATABASE_URL` | PostgreSQL connection string | `postgresql+asyncpg://postgres:postgres@db:5432/drone_show` |
| `REDIS_URL` | Redis connection string | `redis://redis:6379/0` |
| `MINIO_URL` | MinIO connection string | `minio:9000` |
| `MINIO_ACCESS_KEY` | MinIO access key | `minioadmin` |
| `MINIO_SECRET_KEY` | MinIO secret key | `minioadmin` |
| `MINIO_BUCKET` | MinIO bucket name | `drone-show` |
| `SENTINEL_API_URL` | Bulo.Cloud Sentinel API URL | `http://bulocloud-sentinel-api:8000` |
| `SENTINEL_API_TOKEN` | API token for authentication | - |
| `RTMP_SERVER` | RTMP server URL for video streaming | `rtmp://rtmp-server:1935` |
| `LOG_LEVEL` | Logging level | `INFO` |

## Health Checks

The Drone Show microservice provides a health check endpoint at `/health`. This endpoint returns a 200 OK response if the service is healthy.

## Monitoring

The Drone Show microservice can be monitored using standard Kubernetes monitoring tools such as Prometheus and Grafana. The service exposes metrics at the `/metrics` endpoint.

## Scaling

The Drone Show microservice can be scaled horizontally by increasing the number of replicas in the Kubernetes deployment:

```bash
kubectl scale deployment drone-show-api --replicas=3 --namespace=bulocloud-sentinel
```

## Backup and Restore

### Database Backup

To backup the PostgreSQL database:

```bash
kubectl exec -it $(kubectl get pods -l app=db-drone-show -n bulocloud-sentinel -o jsonpath='{.items[0].metadata.name}') -n bulocloud-sentinel -- pg_dump -U postgres drone_show > drone_show_backup.sql
```

### Database Restore

To restore the PostgreSQL database:

```bash
kubectl exec -it $(kubectl get pods -l app=db-drone-show -n bulocloud-sentinel -o jsonpath='{.items[0].metadata.name}') -n bulocloud-sentinel -- psql -U postgres -d drone_show -f - < drone_show_backup.sql
```

### MinIO Backup

To backup the MinIO data:

```bash
kubectl exec -it $(kubectl get pods -l app=minio-drone-show -n bulocloud-sentinel -o jsonpath='{.items[0].metadata.name}') -n bulocloud-sentinel -- mc mirror /data /backup
```

### MinIO Restore

To restore the MinIO data:

```bash
kubectl exec -it $(kubectl get pods -l app=minio-drone-show -n bulocloud-sentinel -o jsonpath='{.items[0].metadata.name}') -n bulocloud-sentinel -- mc mirror /backup /data
```

## Troubleshooting

### Common Issues

- **Database Connection Error**: Ensure that the PostgreSQL database is running and accessible.
- **Redis Connection Error**: Ensure that the Redis cache is running and accessible.
- **MinIO Connection Error**: Ensure that the MinIO object storage is running and accessible.
- **Sentinel API Connection Error**: Ensure that the Bulo.Cloud Sentinel API is running and accessible.
- **Authentication Error**: Ensure that the API token is valid and has the necessary permissions.

### Logs

To view the logs of the Drone Show microservice:

```bash
# Docker Compose
docker-compose logs -f api

# Kubernetes
kubectl logs -f deployment/drone-show-api -n bulocloud-sentinel
```

### Support

If you encounter issues with the Drone Show microservice, please contact the Bulo.Cloud Sentinel support team or refer to the following resources:

- [Drone Show Microservice Documentation](https://bulocloud-sentinel.example.com/docs/drone-show)
- [Bulo.Cloud Sentinel Documentation](https://bulocloud-sentinel.example.com/docs)
- [Bulo.Cloud Sentinel Support](https://bulocloud-sentinel.example.com/support)
