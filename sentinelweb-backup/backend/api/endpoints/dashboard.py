"""
SentinelWeb Backend - Dashboard Endpoints

This module provides endpoints for dashboard management.
"""

from typing import Any, List, Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel, UUID4

from backend.core.auth import get_current_user, User
from backend.db.session import get_db_session
from backend.db.models import DashboardLayout, Widget

router = APIRouter()

# Pydantic models
class WidgetPosition(BaseModel):
    """Widget position in dashboard layout."""
    i: str  # Widget ID
    x: int  # X position
    y: int  # Y position
    w: int  # Width
    h: int  # Height
    minW: Optional[int] = None  # Minimum width
    minH: Optional[int] = None  # Minimum height
    maxW: Optional[int] = None  # Maximum width
    maxH: Optional[int] = None  # Maximum height
    static: Optional[bool] = False  # Whether the widget is static

class WidgetConfig(BaseModel):
    """Widget configuration."""
    id: str
    type: str
    title: str
    position: WidgetPosition
    settings: Optional[dict] = None

class DashboardLayoutCreate(BaseModel):
    """Dashboard layout creation model."""
    name: str
    is_default: bool = False
    layout_data: List[WidgetConfig]

class DashboardLayoutUpdate(BaseModel):
    """Dashboard layout update model."""
    name: Optional[str] = None
    is_default: Optional[bool] = None
    layout_data: Optional[List[WidgetConfig]] = None

class DashboardLayoutResponse(BaseModel):
    """Dashboard layout response model."""
    id: UUID4
    user_id: UUID4
    name: str
    is_default: bool
    layout_data: List[WidgetConfig]
    created_at: Any
    updated_at: Any

@router.get("/layouts", response_model=List[DashboardLayoutResponse])
async def get_dashboard_layouts(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Get all dashboard layouts for the current user.
    
    Args:
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of dashboard layouts
    """
    # Get user ID from database
    user_result = await db.execute(
        f"SELECT id FROM users WHERE username = '{current_user.username}'"
    )
    user_id = user_result.scalar_one_or_none()
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # Get dashboard layouts
    result = await db.execute(
        f"SELECT * FROM dashboard_layouts WHERE user_id = '{user_id}'"
    )
    layouts = result.fetchall()
    
    return layouts

@router.post("/layouts", response_model=DashboardLayoutResponse)
async def create_dashboard_layout(
    layout: DashboardLayoutCreate,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Create a new dashboard layout.
    
    Args:
        layout: Dashboard layout data
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Created dashboard layout
    """
    # Get user ID from database
    user_result = await db.execute(
        f"SELECT id FROM users WHERE username = '{current_user.username}'"
    )
    user_id = user_result.scalar_one_or_none()
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # If this layout is default, unset default for other layouts
    if layout.is_default:
        await db.execute(
            f"UPDATE dashboard_layouts SET is_default = FALSE WHERE user_id = '{user_id}'"
        )
    
    # Create dashboard layout
    db_layout = DashboardLayout(
        user_id=user_id,
        name=layout.name,
        is_default=layout.is_default,
        layout_data=layout.layout_data.dict()
    )
    
    db.add(db_layout)
    await db.commit()
    await db.refresh(db_layout)
    
    return db_layout

@router.get("/layouts/{layout_id}", response_model=DashboardLayoutResponse)
async def get_dashboard_layout(
    layout_id: UUID4,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Get a dashboard layout by ID.
    
    Args:
        layout_id: Dashboard layout ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Dashboard layout
    """
    # Get user ID from database
    user_result = await db.execute(
        f"SELECT id FROM users WHERE username = '{current_user.username}'"
    )
    user_id = user_result.scalar_one_or_none()
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # Get dashboard layout
    result = await db.execute(
        f"SELECT * FROM dashboard_layouts WHERE id = '{layout_id}' AND user_id = '{user_id}'"
    )
    layout = result.fetchone()
    
    if not layout:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Dashboard layout not found"
        )
    
    return layout

@router.put("/layouts/{layout_id}", response_model=DashboardLayoutResponse)
async def update_dashboard_layout(
    layout_id: UUID4,
    layout: DashboardLayoutUpdate,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Update a dashboard layout.
    
    Args:
        layout_id: Dashboard layout ID
        layout: Dashboard layout data
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Updated dashboard layout
    """
    # Get user ID from database
    user_result = await db.execute(
        f"SELECT id FROM users WHERE username = '{current_user.username}'"
    )
    user_id = user_result.scalar_one_or_none()
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # Get dashboard layout
    result = await db.execute(
        f"SELECT * FROM dashboard_layouts WHERE id = '{layout_id}' AND user_id = '{user_id}'"
    )
    db_layout = result.fetchone()
    
    if not db_layout:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Dashboard layout not found"
        )
    
    # If this layout is default, unset default for other layouts
    if layout.is_default:
        await db.execute(
            f"UPDATE dashboard_layouts SET is_default = FALSE WHERE user_id = '{user_id}'"
        )
    
    # Update dashboard layout
    update_data = layout.dict(exclude_unset=True)
    for key, value in update_data.items():
        setattr(db_layout, key, value)
    
    await db.commit()
    await db.refresh(db_layout)
    
    return db_layout

@router.delete("/layouts/{layout_id}")
async def delete_dashboard_layout(
    layout_id: UUID4,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Delete a dashboard layout.
    
    Args:
        layout_id: Dashboard layout ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Deletion confirmation
    """
    # Get user ID from database
    user_result = await db.execute(
        f"SELECT id FROM users WHERE username = '{current_user.username}'"
    )
    user_id = user_result.scalar_one_or_none()
    
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )
    
    # Get dashboard layout
    result = await db.execute(
        f"SELECT * FROM dashboard_layouts WHERE id = '{layout_id}' AND user_id = '{user_id}'"
    )
    db_layout = result.fetchone()
    
    if not db_layout:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Dashboard layout not found"
        )
    
    # Delete dashboard layout
    await db.delete(db_layout)
    await db.commit()
    
    return {"detail": "Dashboard layout deleted"}

@router.get("/widgets", response_model=List[Widget])
async def get_available_widgets(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    Get all available widgets.
    
    Args:
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of available widgets
    """
    # Get widgets
    result = await db.execute("SELECT * FROM widgets")
    widgets = result.fetchall()
    
    return widgets
