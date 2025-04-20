from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from backend.domain.entities.user import User
from backend.domain.repositories.user_repository import IUserRepository
from backend.infrastructure.persistence.models import UserModel

class SqlAlchemyUserRepository(IUserRepository):
    def __init__(self, session: AsyncSession):
        self._session = session

    async def get_by_id(self, user_id: int) -> Optional[User]:
        result = await self._session.execute(select(UserModel).filter_by(id=user_id))
        user_model = result.scalars().first()
        if user_model:
            return self._to_entity(user_model)
        return None

    async def get_by_username(self, username: str) -> Optional[User]:
        result = await self._session.execute(select(UserModel).filter_by(username=username))
        user_model = result.scalars().first()
        if user_model:
            return self._to_entity(user_model)
        return None

    async def add(self, user: User) -> None:
        user_model = UserModel(
            username=user.username,
            email=user.email,
            hashed_password=user.hashed_password,
            is_active=user.is_active,
            is_superuser=user.is_superuser
        )
        self._session.add(user_model)
        await self._session.commit()
        user.id = user_model.id  # Update entity id after insert

    async def update(self, user: User) -> None:
        user_model = await self._session.get(UserModel, user.id)
        if user_model:
            user_model.username = user.username
            user_model.email = user.email
            user_model.hashed_password = user.hashed_password
            user_model.is_active = user.is_active
            user_model.is_superuser = user.is_superuser
            await self._session.commit()

    async def delete(self, user_id: int) -> None:
        user_model = await self._session.get(UserModel, user_id)
        if user_model:
            await self._session.delete(user_model)
            await self._session.commit()

    def _to_entity(self, user_model: UserModel) -> User:
        return User(
            id=user_model.id,
            username=user_model.username,
            email=user_model.email,
            hashed_password=user_model.hashed_password,
            is_active=user_model.is_active,
            is_superuser=user_model.is_superuser
        )
