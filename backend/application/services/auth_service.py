from datetime import datetime, timedelta, timezone
from typing import Optional
# Replacing jose with PyJWT for security (CVE-2024-33664, CVE-2024-33663)
# from jose import jwt
import jwt
import os
from passlib.context import CryptContext
from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError
from backend.domain.repositories.user_repository import IUserRepository
from backend.domain.entities.user import User
from backend.infrastructure.config.config import settings
from security.auth.token_blacklist import add_user_token

# Initialize Argon2 password hasher with secure parameters
# These parameters are based on OWASP recommendations
argon2_hasher = PasswordHasher(
    time_cost=3,  # Number of iterations
    memory_cost=65536,  # 64 MB
    parallelism=4,  # Number of parallel threads
    hash_len=32,  # Length of the hash in bytes
    salt_len=16,  # Length of the salt in bytes
)

# Keep bcrypt for backward compatibility with existing passwords
pwd_context = CryptContext(
    schemes=["argon2", "bcrypt"],
    deprecated=["bcrypt"],
    argon2__hasher=argon2_hasher
)

class AuthService:
    def __init__(self, user_repository: IUserRepository):
        self.user_repository = user_repository

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify a password against a hash.
        
        This method supports both Argon2id and bcrypt hashes for backward compatibility.
        
        Args:
            plain_password: Plain text password
            hashed_password: Hashed password
            
        Returns:
            True if password matches, False otherwise
        """
        try:
            return pwd_context.verify(plain_password, hashed_password)
        except Exception:
            return False

    def hash_password(self, password: str) -> str:
        """
        Hash a password using Argon2id.
        
        Args:
            password: Plain text password
            
        Returns:
            Hashed password
        """
        return pwd_context.hash(password)
    
    def check_password_needs_rehash(self, hashed_password: str) -> bool:
        """
        Check if a password hash needs to be rehashed.
        
        This is useful for upgrading from bcrypt to Argon2id.
        
        Args:
            hashed_password: Hashed password
            
        Returns:
            True if password needs rehashing, False otherwise
        """
        return pwd_context.needs_update(hashed_password)

    async def authenticate_user(self, username: str, password: str) -> Optional[User]:
        """
        Authenticate a user with username and password.
        
        Args:
            username: Username
            password: Password
            
        Returns:
            User object if authentication successful, None otherwise
        """
        user = await self.user_repository.get_by_username(username)
        if not user:
            return None
        
        if not self.verify_password(password, user.hashed_password):
            return None
        
        # Check if password needs rehashing (e.g., upgrading from bcrypt to Argon2id)
        if self.check_password_needs_rehash(user.hashed_password):
            # Rehash password with Argon2id
            new_hash = self.hash_password(password)
            # Update user's password hash
            user.hashed_password = new_hash
            await self.user_repository.update(user)
        
        return user

    def create_access_token(self, user: User, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create a new JWT access token.
        
        Args:
            user: User object
            expires_delta: Optional custom expiration time
            
        Returns:
            JWT token string
        """
        # Set expiration time
        expires_delta = expires_delta or timedelta(minutes=settings.JWT_EXPIRATION_MINUTES)
        expire = datetime.now(timezone.utc) + expires_delta
        
        # Generate a unique token ID
        jti = os.urandom(16).hex()
        
        # Create token payload
        to_encode = {
            "sub": str(user.id),
            "username": user.username,
            "exp": expire,
            "iat": datetime.now(timezone.utc),
            "jti": jti,
            "type": "access",
            "roles": user.roles if hasattr(user, 'roles') else []
        }
        
        # Encode token
        encoded_jwt = jwt.encode(
            to_encode,
            settings.JWT_SECRET,
            algorithm=settings.JWT_ALGORITHM
        )
        
        # Add token to user's token list for tracking
        add_user_token(str(user.id), jti, expire.timestamp())
        
        return encoded_jwt
    
    def create_refresh_token(self, user: User, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create a new JWT refresh token.
        
        Args:
            user: User object
            expires_delta: Optional custom expiration time
            
        Returns:
            JWT token string
        """
        # Set expiration time
        expires_delta = expires_delta or timedelta(days=settings.JWT_REFRESH_TOKEN_EXPIRES_DAYS)
        expire = datetime.now(timezone.utc) + expires_delta
        
        # Generate a unique token ID
        jti = os.urandom(16).hex()
        
        # Create token payload
        to_encode = {
            "sub": str(user.id),
            "exp": expire,
            "iat": datetime.now(timezone.utc),
            "jti": jti,
            "type": "refresh"
        }
        
        # Encode token
        encoded_jwt = jwt.encode(
            to_encode,
            settings.JWT_SECRET,
            algorithm=settings.JWT_ALGORITHM
        )
        
        # Add token to user's token list for tracking
        add_user_token(str(user.id), jti, expire.timestamp())
        
        return encoded_jwt
    
    def create_token_response(self, user: User) -> dict:
        """
        Create a complete token response with access and refresh tokens.
        
        Args:
            user: User object
            
        Returns:
            Token response dictionary
        """
        access_token = self.create_access_token(user)
        refresh_token = self.create_refresh_token(user)
        
        return {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer",
            "expires_in": settings.JWT_EXPIRATION_MINUTES * 60
        }
    
    async def refresh_access_token(self, refresh_token: str) -> Optional[dict]:
        """
        Refresh an access token using a refresh token.
        
        Args:
            refresh_token: Refresh token
            
        Returns:
            New token response dictionary or None if refresh token is invalid
        """
        try:
            # Decode and validate refresh token
            payload = jwt.decode(
                refresh_token,
                settings.JWT_SECRET,
                algorithms=[settings.JWT_ALGORITHM],
                options={
                    "verify_signature": True,
                    "verify_exp": True,
                    "verify_iat": True,
                    "require": ["sub", "exp", "iat", "jti", "type"]
                }
            )
            
            # Check token type
            if payload.get("type") != "refresh":
                return None
            
            # Get user ID from token
            user_id = payload.get("sub")
            if not user_id:
                return None
            
            # Get user from database
            user = await self.user_repository.get_by_id(user_id)
            if not user:
                return None
            
            # Create new tokens
            return self.create_token_response(user)
        
        except Exception:
            return None
