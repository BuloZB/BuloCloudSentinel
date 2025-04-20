from fastapi import Depends
from sqlalchemy.ext.asyncio import AsyncSession
from backend.infrastructure.database import get_session
from backend.infrastructure.persistence.sqlalchemy_user_repository import SqlAlchemyUserRepository
from backend.domain.repositories.user_repository import IUserRepository

async def get_user_repository(session: AsyncSession = Depends(get_session)) -> IUserRepository:
    return SqlAlchemyUserRepository(session)
