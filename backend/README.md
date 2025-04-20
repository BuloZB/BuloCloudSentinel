# Backend Architecture for Bulo.Cloud Sentinel

This backend is designed following clean architecture principles with clear separation of concerns:

- **API Layer**: FastAPI routes and request/response models.
- **Business Logic Layer**: Domain services encapsulating business rules.
- **Data Access Layer (DAL)**: Repository interfaces and implementations using SQLAlchemy.
- **Infrastructure Layer**: Database setup, configuration, and external integrations.

## Key Patterns

- Repository Pattern for data access abstraction.
- Dependency Injection for decoupling components.
- JWT Authentication with secure token handling.
- Configuration management with environment variables and validation.

## Project Structure

```
backend/
├── api/                # FastAPI routes and schemas
├── core/               # Business logic and domain services
├── db/                 # Database models and repository implementations
├── infra/              # Infrastructure concerns (config, security)
├── main.py             # FastAPI app entrypoint
├── requirements.txt    # Python dependencies
├── Dockerfile
└── README.md
```

## Architectural Decisions

- Use SQLite for MVP with easy migration to PostgreSQL.
- Use SQLAlchemy ORM for database interactions.
- Secure JWT tokens with expiration and refresh mechanisms.
- Implement unit and integration tests for all layers.
