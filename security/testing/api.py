"""
Security testing API for Bulo.Cloud Sentinel.

This module provides API endpoints for security testing.
"""

import logging
import os
from typing import Dict, List, Optional, Any, Union
from fastapi import APIRouter, Depends, HTTPException, status, Security, Query, BackgroundTasks
from pydantic import BaseModel, Field

from ..auth.jwt_handler import has_role, has_permission
from .security_scanner import (
    SecurityIssue,
    SecurityScanResult,
    scan,
    get_result,
    get_results,
    save_result,
    load_result
)

# Configure logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(
    prefix="/security/testing",
    tags=["security_testing"]
)


# Models
class ScanRequest(BaseModel):
    """Model for requesting a security scan."""
    target: str = Field(..., description="Target to scan")
    scanner_name: Optional[str] = Field(None, description="Scanner name")
    options: Optional[Dict[str, Any]] = Field(None, description="Scan options")


class ScanResultResponse(BaseModel):
    """Model for scan result response."""
    scan_id: str = Field(..., description="Scan ID")
    scanner: str = Field(..., description="Scanner name")
    start_time: float = Field(..., description="Scan start time")
    start_time_iso: str = Field(..., description="Scan start time (ISO format)")
    end_time: Optional[float] = Field(None, description="Scan end time")
    end_time_iso: Optional[str] = Field(None, description="Scan end time (ISO format)")
    duration: float = Field(..., description="Scan duration")
    is_complete: bool = Field(..., description="Whether scan is complete")
    scan_options: Dict[str, Any] = Field(..., description="Scan options")
    issue_count: int = Field(..., description="Number of issues")
    severity_counts: Dict[str, int] = Field(..., description="Severity counts")


class SecurityIssueResponse(BaseModel):
    """Model for security issue response."""
    scanner: str = Field(..., description="Scanner name")
    issue_type: str = Field(..., description="Issue type")
    severity: str = Field(..., description="Issue severity")
    file_path: str = Field(..., description="File path")
    line_number: Optional[int] = Field(None, description="Line number")
    message: str = Field(..., description="Issue message")
    code: Optional[str] = Field(None, description="Code snippet")
    confidence: Optional[str] = Field(None, description="Confidence level")
    cwe: Optional[Union[str, int]] = Field(None, description="CWE identifier")
    fix: Optional[str] = Field(None, description="Suggested fix")
    references: List[str] = Field(..., description="References")


class SaveResultRequest(BaseModel):
    """Model for saving a scan result."""
    scan_id: str = Field(..., description="Scan ID")
    file_path: str = Field(..., description="File path")


class LoadResultRequest(BaseModel):
    """Model for loading a scan result."""
    file_path: str = Field(..., description="File path")


# Endpoints
@router.post("/scan", status_code=status.HTTP_202_ACCEPTED)
async def start_security_scan(
    scan_request: ScanRequest,
    background_tasks: BackgroundTasks,
    current_user: str = Security(has_permission("security:run_scan"))
) -> Dict[str, Any]:
    """
    Start a security scan.
    
    Args:
        scan_request: Scan request
        background_tasks: Background tasks
        current_user: Current user
        
    Returns:
        Scan ID
    """
    try:
        # Validate target
        if not os.path.exists(scan_request.target):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Target not found: {scan_request.target}"
            )
        
        # Create a function to run the scan in the background
        def run_scan():
            try:
                scan(
                    target=scan_request.target,
                    scanner_name=scan_request.scanner_name,
                    options=scan_request.options
                )
                logger.info(f"Completed security scan of {scan_request.target}")
            except Exception as e:
                logger.error(f"Error running security scan: {str(e)}")
        
        # Add scan to background tasks
        background_tasks.add_task(run_scan)
        
        return {
            "success": True,
            "message": f"Security scan of {scan_request.target} started"
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error starting security scan: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error starting security scan: {str(e)}"
        )


@router.get("/results")
async def list_scan_results(
    current_user: str = Security(has_permission("security:view_scan_results"))
) -> List[ScanResultResponse]:
    """
    List scan results.
    
    Args:
        current_user: Current user
        
    Returns:
        List of scan results
    """
    try:
        # Get all results
        results = get_results()
        
        # Convert to response models
        return [
            ScanResultResponse(
                scan_id=result.scan_id,
                scanner=result.scanner,
                start_time=result.start_time,
                start_time_iso=result.to_dict()["start_time_iso"],
                end_time=result.end_time,
                end_time_iso=result.to_dict().get("end_time_iso"),
                duration=result.duration,
                is_complete=result.is_complete,
                scan_options=result.scan_options,
                issue_count=result.issue_count,
                severity_counts=result.severity_counts
            )
            for result in results
        ]
    
    except Exception as e:
        logger.error(f"Error listing scan results: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing scan results: {str(e)}"
        )


@router.get("/results/{scan_id}")
async def get_scan_result(
    scan_id: str,
    current_user: str = Security(has_permission("security:view_scan_results"))
) -> ScanResultResponse:
    """
    Get a scan result.
    
    Args:
        scan_id: Scan ID
        current_user: Current user
        
    Returns:
        Scan result
    """
    try:
        # Get result
        result = get_result(scan_id)
        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Scan result not found: {scan_id}"
            )
        
        # Convert to response model
        return ScanResultResponse(
            scan_id=result.scan_id,
            scanner=result.scanner,
            start_time=result.start_time,
            start_time_iso=result.to_dict()["start_time_iso"],
            end_time=result.end_time,
            end_time_iso=result.to_dict().get("end_time_iso"),
            duration=result.duration,
            is_complete=result.is_complete,
            scan_options=result.scan_options,
            issue_count=result.issue_count,
            severity_counts=result.severity_counts
        )
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error getting scan result: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting scan result: {str(e)}"
        )


@router.get("/results/{scan_id}/issues")
async def get_scan_issues(
    scan_id: str,
    severity: Optional[str] = Query(None, description="Filter by severity"),
    file_path: Optional[str] = Query(None, description="Filter by file path"),
    current_user: str = Security(has_permission("security:view_scan_results"))
) -> List[SecurityIssueResponse]:
    """
    Get issues from a scan result.
    
    Args:
        scan_id: Scan ID
        severity: Filter by severity
        file_path: Filter by file path
        current_user: Current user
        
    Returns:
        List of security issues
    """
    try:
        # Get result
        result = get_result(scan_id)
        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Scan result not found: {scan_id}"
            )
        
        # Filter issues
        issues = result.issues
        
        if severity:
            issues = [i for i in issues if i.severity.lower() == severity.lower()]
        
        if file_path:
            issues = [i for i in issues if i.file_path == file_path]
        
        # Convert to response models
        return [
            SecurityIssueResponse(
                scanner=issue.scanner,
                issue_type=issue.issue_type,
                severity=issue.severity,
                file_path=issue.file_path,
                line_number=issue.line_number,
                message=issue.message,
                code=issue.code,
                confidence=issue.confidence,
                cwe=issue.cwe,
                fix=issue.fix,
                references=issue.references
            )
            for issue in issues
        ]
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error getting scan issues: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting scan issues: {str(e)}"
        )


@router.post("/results/save")
async def save_scan_result(
    request: SaveResultRequest,
    current_user: str = Security(has_permission("security:manage_scan_results"))
) -> Dict[str, Any]:
    """
    Save a scan result to a file.
    
    Args:
        request: Save request
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Check if result exists
        result = get_result(request.scan_id)
        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Scan result not found: {request.scan_id}"
            )
        
        # Save result
        save_result(request.scan_id, request.file_path)
        
        return {
            "success": True,
            "message": f"Scan result saved to {request.file_path}"
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error saving scan result: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error saving scan result: {str(e)}"
        )


@router.post("/results/load")
async def load_scan_result(
    request: LoadResultRequest,
    current_user: str = Security(has_permission("security:manage_scan_results"))
) -> Dict[str, Any]:
    """
    Load a scan result from a file.
    
    Args:
        request: Load request
        current_user: Current user
        
    Returns:
        Success status
    """
    try:
        # Load result
        result = load_result(request.file_path)
        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Failed to load scan result from {request.file_path}"
            )
        
        return {
            "success": True,
            "message": f"Scan result loaded from {request.file_path}",
            "scan_id": result.scan_id
        }
    
    except HTTPException:
        raise
    
    except Exception as e:
        logger.error(f"Error loading scan result: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error loading scan result: {str(e)}"
        )
