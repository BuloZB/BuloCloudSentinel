"""
Content Security Policy (CSP) reporting endpoint.

This module provides an API endpoint for receiving CSP violation reports.
"""

import json

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
import logging
from typing import Dict, Any, Optional

from fastapi import APIRouter, Request, Response, status
from pydantic import BaseModel, Field

from security.logging.secure_logging import get_secure_logger

# Configure secure logger
logger = get_secure_logger("csp_reports")

router = APIRouter()

class CSPViolation(BaseModel):
    """Model for CSP violation report."""
    document_uri: Optional[str] = Field(None, alias="document-uri")
    referrer: Optional[str] = None
    blocked_uri: Optional[str] = Field(None, alias="blocked-uri")
    violated_directive: Optional[str] = Field(None, alias="violated-directive")
    effective_directive: Optional[str] = Field(None, alias="effective-directive")
    original_policy: Optional[str] = Field(None, alias="original-policy")
    disposition: Optional[str] = None
    source_file: Optional[str] = Field(None, alias="source-file")
    line_number: Optional[int] = Field(None, alias="line-number")
    column_number: Optional[int] = Field(None, alias="column-number")
    status_code: Optional[int] = Field(None, alias="status-code")
    script_sample: Optional[str] = Field(None, alias="script-sample")

    class Config:
        allow_population_by_field_name = True

class CSPReport(BaseModel):
    """Model for CSP report."""
    csp_report: CSPViolation = Field(..., alias="csp-report")

    class Config:
        allow_population_by_field_name = True

@router.post("/csp-report")
async def receive_csp_report(request: Request):
    """
    Receive and log CSP violation reports.
    
    Args:
        request: The HTTP request containing the CSP report
        
    Returns:
        Empty response with 204 status code
    """
    try:
        # Get client IP and user agent
        client_ip = request.client.host if request.client else "unknown"
        user_agent = request.headers.get("user-agent", "unknown")
        
        # Get report body
        body = await request.body()
        
        # Parse report
        try:
            report_data = json.loads(body)
            report = CSPReport(**report_data)
            
            # Log the violation
            logger.warning(
                "CSP violation detected",
                {
                    "client_ip": client_ip,
                    "user_agent": user_agent,
                    "document_uri": report.csp_report.document_uri,
                    "blocked_uri": report.csp_report.blocked_uri,
                    "violated_directive": report.csp_report.violated_directive,
                    "effective_directive": report.csp_report.effective_directive,
                    "source_file": report.csp_report.source_file,
                    "line_number": report.csp_report.line_number,
                    "column_number": report.csp_report.column_number
                }
            )
        except Exception as e:
            # If parsing fails, log the raw report
            logger.warning(
                "Failed to parse CSP report",
                {
                    "client_ip": client_ip,
                    "user_agent": user_agent,
                    "error": str(e),
                    "raw_report": body.decode("utf-8", errors="replace")
                }
            )
    except Exception as e:
        logger.error(f"Error processing CSP report: {str(e)}")
    
    # Return empty response
    return Response(status_code=status.HTTP_204_NO_CONTENT)

@router.post("/csp-report-uri")
async def receive_csp_report_uri(request: Request):
    """
    Legacy endpoint for receiving CSP violation reports via report-uri directive.
    
    Args:
        request: The HTTP request containing the CSP report
        
    Returns:
        Empty response with 204 status code
    """
    return await receive_csp_report(request)

@router.post("/report-to")
async def receive_report_to(request: Request):
    """
    Endpoint for receiving reports via the Report-To header.
    
    Args:
        request: The HTTP request containing the report
        
    Returns:
        Empty response with 204 status code
    """
    try:
        # Get client IP and user agent
        client_ip = request.client.host if request.client else "unknown"
        user_agent = request.headers.get("user-agent", "unknown")
        
        # Get report body
        body = await request.body()
        
        # Parse report
        try:
            reports = json.loads(body)
            
            # Log each report
            for report in reports:
                report_type = report.get("type", "unknown")
                
                if report_type == "csp-violation":
                    logger.warning(
                        "CSP violation detected (Report-To)",
                        {
                            "client_ip": client_ip,
                            "user_agent": user_agent,
                            "report": report
                        }
                    )
                else:
                    logger.info(
                        f"Received {report_type} report",
                        {
                            "client_ip": client_ip,
                            "user_agent": user_agent,
                            "report": report
                        }
                    )
        except Exception as e:
            # If parsing fails, log the raw report
            logger.warning(
                "Failed to parse Report-To report",
                {
                    "client_ip": client_ip,
                    "user_agent": user_agent,
                    "error": str(e),
                    "raw_report": body.decode("utf-8", errors="replace")
                }
            )
    except Exception as e:
        logger.error(f"Error processing Report-To report: {str(e)}")
    
    # Return empty response
    return Response(status_code=status.HTTP_204_NO_CONTENT)