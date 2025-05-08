# Bulo.CloudSentinel Model Hub

The Model Hub is a microservice for versioned model management in Bulo.CloudSentinel. It provides a centralized registry for storing, tagging, and deploying vision models with support for blue/green deployments and automatic rollback on degradation.

## Features

- **Model Registry**: Store and version AI models with MLflow + MinIO backend
- **Metadata Tracking**: Track model accuracy, hash, size, and hardware compatibility
- **Blue/Green Deployments**: Deploy models with zero downtime using Argo Rollouts
- **Automatic Rollback**: Automatically roll back to previous version if performance degrades
- **CLI & UI**: Command-line and web interfaces for model management
- **Security**: Signed models with hash verification and seccomp confinement

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│                 │     │                 │     │                 │
│  Model Registry │     │   API Service   │     │   SentinelWeb   │
│   (MLflow+S3)   │◄────┤   (FastAPI)     │◄────┤    Interface    │
│                 │     │                 │     │                 │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │   Deployment    │
                        │  Orchestrator   │
                        │ (Argo Rollouts) │
                        └────────┬────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │                 │
                        │   Edge Devices  │
                        │                 │
                        └─────────────────┘
```

## Setup

1. Clone the repository and navigate to the `model_hub_service` directory.

2. Create a `.env` file with the following environment variables:

```
MLFLOW_S3_ENDPOINT_URL=http://minio:9000
AWS_ACCESS_KEY_ID=minioadmin
AWS_SECRET_ACCESS_KEY=minioadmin
MLFLOW_TRACKING_URI=http://mlflow:5000
POSTGRES_USER=postgres
POSTGRES_PASSWORD=postgres
POSTGRES_DB=model_hub
POSTGRES_HOST=postgres
```

3. Build and run the Docker container:

```bash
docker-compose up -d
```

## API Endpoints

- `GET /api/v1/models` - List all models
- `GET /api/v1/models/{model_id}` - Get model details
- `POST /api/v1/models` - Register a new model
- `PUT /api/v1/models/{model_id}` - Update model metadata
- `DELETE /api/v1/models/{model_id}` - Delete a model
- `POST /api/v1/models/{model_id}/deploy` - Deploy a model
- `POST /api/v1/models/{model_id}/rollback` - Rollback to a previous version
- `GET /api/v1/deployments` - List all deployments
- `GET /api/v1/deployments/{deployment_id}` - Get deployment details

## CLI Usage

```bash
# Push a model to the registry
sentinel model push path/to/model.pt --name yolov10 --stage staging

# List all models
sentinel model list

# Deploy a model
sentinel model deploy yolov10 --version 1.0.0 --target production

# Rollback to a previous version
sentinel model rollback yolov10 --version 0.9.0
```

## Security

The Model Hub implements several security features:

- **Model Signing**: All models are signed using cosign
- **Hash Verification**: Model hash is verified before loading
- **Seccomp Confinement**: Models run in a seccomp-confined runtime
- **Access Control**: Role-based access control for model management

## Integration with Edge Kit

The Model Hub integrates with the Edge Kit to provide over-the-air updates to edge devices:

1. Edge devices subscribe to the gRPC stream for model updates
2. When a new model is deployed, the Edge Kit receives a notification
3. The Edge Kit downloads the new model and verifies its hash
4. The model is loaded in a blue/green deployment
5. If performance metrics degrade, the Edge Kit automatically rolls back to the previous version
