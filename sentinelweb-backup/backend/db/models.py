"""
SentinelWeb Backend - Database Models

This module defines the SQLAlchemy ORM models for the SentinelWeb database.
"""

import uuid
from datetime import datetime, timezone
from typing import List, Optional

from sqlalchemy import Column, String, Integer, Boolean, DateTime, ForeignKey, Table, Text, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID

from backend.db.session import Base

# Association tables for many-to-many relationships
user_role_association = Table(
    'user_role_association',
    Base.metadata,
    Column('user_id', UUID(as_uuid=True), ForeignKey('users.id')),
    Column('role_id', UUID(as_uuid=True), ForeignKey('roles.id'))
)

role_permission_association = Table(
    'role_permission_association',
    Base.metadata,
    Column('role_id', UUID(as_uuid=True), ForeignKey('roles.id')),
    Column('permission_id', UUID(as_uuid=True), ForeignKey('permissions.id'))
)

class User(Base):
    """User model for SentinelWeb."""
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    username = Column(String(50), unique=True, index=True, nullable=False)
    email = Column(String(100), unique=True, index=True, nullable=False)
    hashed_password = Column(String(100), nullable=True)  # Nullable for SSO users
    full_name = Column(String(100), nullable=True)
    is_active = Column(Boolean, default=True)
    is_superuser = Column(Boolean, default=False)
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))
    last_login = Column(DateTime, nullable=True)

    # Relationships
    roles = relationship("Role", secondary=user_role_association, back_populates="users")
    preferences = relationship("UserPreference", back_populates="user", cascade="all, delete-orphan")
    dashboard_layouts = relationship("DashboardLayout", back_populates="user", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<User {self.username}>"

class Role(Base):
    """Role model for RBAC."""
    __tablename__ = "roles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(50), unique=True, index=True, nullable=False)
    description = Column(String(200), nullable=True)
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    users = relationship("User", secondary=user_role_association, back_populates="roles")
    permissions = relationship("Permission", secondary=role_permission_association, back_populates="roles")

    def __repr__(self):
        return f"<Role {self.name}>"

class Permission(Base):
    """Permission model for RBAC."""
    __tablename__ = "permissions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(50), unique=True, index=True, nullable=False)
    description = Column(String(200), nullable=True)
    resource = Column(String(50), nullable=False)  # e.g., "drone", "mission", "user"
    action = Column(String(50), nullable=False)    # e.g., "read", "write", "delete"
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))

    # Relationships
    roles = relationship("Role", secondary=role_permission_association, back_populates="permissions")

    def __repr__(self):
        return f"<Permission {self.name}>"

class UserPreference(Base):
    """User preferences model."""
    __tablename__ = "user_preferences"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    theme = Column(String(20), default="light")
    language = Column(String(10), default="en")
    notifications_enabled = Column(Boolean, default=True)
    telemetry_refresh_rate = Column(Integer, default=5)  # in seconds
    map_provider = Column(String(20), default="osm")
    default_view = Column(String(20), default="dashboard")
    custom_settings = Column(JSON, nullable=True)

    # Relationships
    user = relationship("User", back_populates="preferences")

    def __repr__(self):
        return f"<UserPreference {self.id}>"

class DashboardLayout(Base):
    """Dashboard layout model for storing user dashboard configurations."""
    __tablename__ = "dashboard_layouts"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    name = Column(String(50), nullable=False)
    is_default = Column(Boolean, default=False)
    layout_data = Column(JSON, nullable=False)  # Stores widget positions, sizes, etc.
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    user = relationship("User", back_populates="dashboard_layouts")

    def __repr__(self):
        return f"<DashboardLayout {self.name}>"

class Plugin(Base):
    """Plugin model for storing installed plugins."""
    __tablename__ = "plugins"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(50), unique=True, nullable=False)
    version = Column(String(20), nullable=False)
    description = Column(Text, nullable=True)
    entry_point = Column(String(100), nullable=False)
    is_enabled = Column(Boolean, default=True)
    config = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    def __repr__(self):
        return f"<Plugin {self.name} v{self.version}>"

class Widget(Base):
    """Widget model for dashboard widgets."""
    __tablename__ = "widgets"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(50), nullable=False)
    description = Column(Text, nullable=True)
    widget_type = Column(String(50), nullable=False)  # e.g., "telemetry", "map", "video"
    default_config = Column(JSON, nullable=True)
    icon = Column(String(50), nullable=True)
    created_at = Column(DateTime, default=lambda: datetime.now(timezone.utc))

    def __repr__(self):
        return f"<Widget {self.name}>"

class AuditLog(Base):
    """Audit log model for tracking user actions."""
    __tablename__ = "audit_logs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=True)
    action = Column(String(50), nullable=False)
    resource_type = Column(String(50), nullable=False)
    resource_id = Column(String(50), nullable=True)
    details = Column(JSON, nullable=True)
    ip_address = Column(String(50), nullable=True)
    user_agent = Column(String(200), nullable=True)
    timestamp = Column(DateTime, default=lambda: datetime.now(timezone.utc))

    def __repr__(self):
        return f"<AuditLog {self.action} on {self.resource_type}>"
