"""
SentinelWeb Backend - User Service

This module provides services for user management.
"""

import logging
from datetime import datetime
from typing import Optional, List, Dict, Any

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from passlib.context import CryptContext

from backend.db.models import User, Role, Permission

logger = logging.getLogger(__name__)

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

class UserService:
    """Service for user management."""
    
    def __init__(self, db: AsyncSession):
        """
        Initialize the user service.
        
        Args:
            db: Database session
        """
        self.db = db
    
    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify a password against a hash.
        
        Args:
            plain_password: Plain text password
            hashed_password: Hashed password
            
        Returns:
            True if the password matches the hash
        """
        return pwd_context.verify(plain_password, hashed_password)
    
    def get_password_hash(self, password: str) -> str:
        """
        Hash a password.
        
        Args:
            password: Plain text password
            
        Returns:
            Hashed password
        """
        return pwd_context.hash(password)
    
    async def get_user_by_username(self, username: str) -> Optional[User]:
        """
        Get a user by username.
        
        Args:
            username: Username
            
        Returns:
            User if found, None otherwise
        """
        result = await self.db.execute(
            select(User).where(User.username == username)
        )
        return result.scalars().first()
    
    async def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Get a user by email.
        
        Args:
            email: Email address
            
        Returns:
            User if found, None otherwise
        """
        result = await self.db.execute(
            select(User).where(User.email == email)
        )
        return result.scalars().first()
    
    async def create_user(
        self,
        username: str,
        email: str,
        password: Optional[str] = None,
        full_name: Optional[str] = None,
        is_active: bool = True,
        is_superuser: bool = False,
        roles: Optional[List[str]] = None
    ) -> User:
        """
        Create a new user.
        
        Args:
            username: Username
            email: Email address
            password: Optional password (for local authentication)
            full_name: Optional full name
            is_active: Whether the user is active
            is_superuser: Whether the user is a superuser
            roles: Optional list of role names
            
        Returns:
            Created user
        """
        # Hash password if provided
        hashed_password = None
        if password:
            hashed_password = self.get_password_hash(password)
        
        # Create user
        user = User(
            username=username,
            email=email,
            hashed_password=hashed_password,
            full_name=full_name,
            is_active=is_active,
            is_superuser=is_superuser
        )
        
        # Add roles if provided
        if roles:
            for role_name in roles:
                # Get or create role
                role = await self.get_or_create_role(role_name)
                user.roles.append(role)
        
        # Add user to database
        self.db.add(user)
        await self.db.commit()
        await self.db.refresh(user)
        
        return user
    
    async def get_or_create_user(
        self,
        username: str,
        email: str,
        roles: Optional[List[str]] = None
    ) -> User:
        """
        Get a user by username or create a new one if not found.
        
        Args:
            username: Username
            email: Email address
            roles: Optional list of role names
            
        Returns:
            User
        """
        # Try to get user
        user = await self.get_user_by_username(username)
        
        # Create user if not found
        if not user:
            user = await self.create_user(
                username=username,
                email=email,
                roles=roles or ["user"]
            )
        
        return user
    
    async def authenticate(self, username: str, password: str) -> Optional[User]:
        """
        Authenticate a user with username and password.
        
        Args:
            username: Username
            password: Password
            
        Returns:
            User if authentication succeeds, None otherwise
        """
        user = await self.get_user_by_username(username)
        
        if not user or not user.hashed_password:
            return None
        
        if not self.verify_password(password, user.hashed_password):
            return None
        
        return user
    
    async def update_last_login(self, username: str) -> None:
        """
        Update a user's last login timestamp.
        
        Args:
            username: Username
        """
        user = await self.get_user_by_username(username)
        
        if user:
            user.last_login = datetime.utcnow()
            await self.db.commit()
    
    async def get_or_create_role(self, role_name: str) -> Role:
        """
        Get a role by name or create a new one if not found.
        
        Args:
            role_name: Role name
            
        Returns:
            Role
        """
        # Try to get role
        result = await self.db.execute(
            select(Role).where(Role.name == role_name)
        )
        role = result.scalars().first()
        
        # Create role if not found
        if not role:
            role = Role(name=role_name)
            self.db.add(role)
            await self.db.commit()
            await self.db.refresh(role)
        
        return role
    
    async def get_or_create_permission(
        self,
        name: str,
        resource: str,
        action: str,
        description: Optional[str] = None
    ) -> Permission:
        """
        Get a permission by name or create a new one if not found.
        
        Args:
            name: Permission name
            resource: Resource name
            action: Action name
            description: Optional description
            
        Returns:
            Permission
        """
        # Try to get permission
        result = await self.db.execute(
            select(Permission).where(Permission.name == name)
        )
        permission = result.scalars().first()
        
        # Create permission if not found
        if not permission:
            permission = Permission(
                name=name,
                resource=resource,
                action=action,
                description=description
            )
            self.db.add(permission)
            await self.db.commit()
            await self.db.refresh(permission)
        
        return permission
