from typing import Optional
from backend.domain.entities.user import User
from backend.domain.repositories.user_repository import IUserRepository
from backend.domain.value_objects.email import Email

class UserService:
    def __init__(self, user_repository: IUserRepository):
        self._user_repository = user_repository

    async def register_user(self, username: str, email_str: str, hashed_password: str) -> User:
        email = Email(email_str)
        existing_user = await self._user_repository.get_by_username(username)
        if existing_user:
            raise ValueError("Username already exists")
        # Additional business rules can be added here
        user = User(id=0, username=username, email=email.address, hashed_password=hashed_password)
        await self._user_repository.add(user)
        return user

    async def get_user_by_id(self, user_id: int) -> Optional[User]:
        return await self._user_repository.get_by_id(user_id)

    async def authenticate(self, username: str, password: str) -> Optional[User]:
        user = await self._user_repository.get_by_username(username)
        if user and self._verify_password(password, user.hashed_password):
            return user
        return None

    def _verify_password(self, plain_password: str, hashed_password: str) -> bool:
        # Implement password verification logic, e.g., bcrypt
        import bcrypt
        return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))
