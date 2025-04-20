from pydantic import BaseSettings, Field, validator
from typing import Optional

class Settings(BaseSettings):
    database_url: str = Field(..., env="DATABASE_URL")
    jwt_secret: str = Field(..., env="JWT_SECRET")
    jwt_algorithm: str = "HS256"
    jwt_expiration_minutes: int = 60

    @validator("database_url")
    def validate_database_url(cls, v):
        if not v.startswith("sqlite:///") and not v.startswith("postgresql://") and not v.startswith("mysql://"):
            raise ValueError("DATABASE_URL must start with sqlite:///, postgresql:// or mysql://")
        return v

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"

settings = Settings()
