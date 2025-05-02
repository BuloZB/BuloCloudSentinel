"""
Models for the Secure Development Lifecycle (SDL) module.

This module defines the data models used by the SDL module.
"""

import uuid
from datetime import datetime
from enum import Enum, auto
from typing import Dict, List, Optional, Set, Union, Any
from pydantic import BaseModel, Field, validator


class SDLPhase(str, Enum):
    """SDL phase enumeration."""
    REQUIREMENTS = "requirements"
    DESIGN = "design"
    IMPLEMENTATION = "implementation"
    VERIFICATION = "verification"
    RELEASE = "release"
    RESPONSE = "response"


class SDLStatus(str, Enum):
    """SDL status enumeration."""
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    BLOCKED = "blocked"
    DEFERRED = "deferred"


class SDLSeverity(str, Enum):
    """SDL severity enumeration."""
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    INFO = "info"


class SDLRequirement(BaseModel):
    """SDL security requirement."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    phase: SDLPhase
    status: SDLStatus = SDLStatus.NOT_STARTED
    priority: SDLSeverity
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SDLThreatModel(BaseModel):
    """SDL threat model."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    threat_type: str
    assets: List[str]
    vulnerabilities: List[str]
    mitigations: List[str]
    severity: SDLSeverity
    status: SDLStatus = SDLStatus.NOT_STARTED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SDLCodeReview(BaseModel):
    """SDL code review."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    files: List[str]
    findings: List[str] = Field(default_factory=list)
    status: SDLStatus = SDLStatus.NOT_STARTED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    reviewers: List[str] = Field(default_factory=list)
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SDLSecurityTest(BaseModel):
    """SDL security test."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    test_type: str
    target: str
    results: List[str] = Field(default_factory=list)
    status: SDLStatus = SDLStatus.NOT_STARTED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SDLVulnerability(BaseModel):
    """SDL vulnerability."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    severity: SDLSeverity
    status: SDLStatus = SDLStatus.NOT_STARTED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    affected_components: List[str] = Field(default_factory=list)
    mitigations: List[str] = Field(default_factory=list)
    cve_id: Optional[str] = None
    cvss_score: Optional[float] = None
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SDLProject(BaseModel):
    """SDL project."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    phase: SDLPhase = SDLPhase.REQUIREMENTS
    status: SDLStatus = SDLStatus.NOT_STARTED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    created_by: Optional[str] = None
    assigned_to: Optional[str] = None
    requirements: List[SDLRequirement] = Field(default_factory=list)
    threats: List[SDLThreatModel] = Field(default_factory=list)
    code_reviews: List[SDLCodeReview] = Field(default_factory=list)
    security_tests: List[SDLSecurityTest] = Field(default_factory=list)
    vulnerabilities: List[SDLVulnerability] = Field(default_factory=list)
    tags: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    
    @validator('updated_at', always=True)
    def set_updated_at(cls, v, values):
        """Set updated_at to current time."""
        return datetime.now()
