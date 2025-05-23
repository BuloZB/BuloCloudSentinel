"""
EASA SORA adapter for the Remote ID & Regulatory Compliance Service.

This module provides an adapter for the EASA SORA API.
"""

import logging
import json
from datetime import datetime
from typing import Dict, List, Optional, Any

import httpx
from zeep import Client
from zeep.transports import Transport

from remoteid_service.api.schemas.flightplans import (
    FlightPlanSubmissionResponse,
    FlightPlanStatus,
    EASASoraRiskClass,
    EASASoraSubmission,
)

# Configure logging
logger = logging.getLogger(__name__)

class EASASoraAdapter:
    """
    EASA SORA adapter.
    
    This adapter provides functionality for submitting flight plans to the
    EASA SORA API.
    """
    
    def __init__(self, api_url: str, api_key: str):
        """
        Initialize the EASA SORA adapter.
        
        Args:
            api_url: API URL
            api_key: API key
        """
        self.api_url = api_url
        self.api_key = api_key
        self.client = None
        
        # Initialize SOAP client if URL is provided
        if api_url:
            try:
                # Create transport with timeout and authentication
                transport = Transport(timeout=30)
                
                # Create SOAP client
                self.client = Client(
                    f"{api_url}?wsdl",
                    transport=transport,
                )
                
                logger.info("Initialized EASA SORA adapter")
            except Exception as e:
                logger.error(f"Error initializing EASA SORA adapter: {str(e)}")
    
    async def submit_flight_plan(
        self,
        submission: EASASoraSubmission,
    ) -> FlightPlanSubmissionResponse:
        """
        Submit a flight plan to the EASA SORA API.
        
        Args:
            submission: EASA SORA submission
            
        Returns:
            FlightPlanSubmissionResponse: Submission response
        """
        try:
            # Check if client is initialized
            if not self.client:
                raise ValueError("EASA SORA adapter not initialized")
            
            # Convert submission to SOAP request
            request = self._create_soap_request(submission)
            
            # Submit request
            async with httpx.AsyncClient() as http_client:
                # Call SOAP service
                response = await http_client.post(
                    self.api_url,
                    headers={
                        "Content-Type": "text/xml",
                        "SOAPAction": "submitFlightPlan",
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    content=request,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                soap_response = self.client.process_reply(response.text)
                
                # Extract submission ID and status
                submission_id = soap_response["submissionId"]
                status = soap_response["status"]
                
                # Map status to FlightPlanStatus
                if status == "APPROVED":
                    flight_plan_status = FlightPlanStatus.APPROVED
                elif status == "REJECTED":
                    flight_plan_status = FlightPlanStatus.REJECTED
                else:
                    flight_plan_status = FlightPlanStatus.SUBMITTED
                
                # Create response
                return FlightPlanSubmissionResponse(
                    flight_plan_id=submission.flight_plan_id,
                    submission_id=submission_id,
                    submission_time=datetime.utcnow(),
                    status=flight_plan_status,
                    message=soap_response.get("message"),
                    details=soap_response,
                )
        except Exception as e:
            logger.error(f"Error submitting flight plan to EASA SORA: {str(e)}")
            raise
    
    async def check_submission_status(
        self,
        submission_id: str,
    ) -> FlightPlanSubmissionResponse:
        """
        Check the status of a flight plan submission.
        
        Args:
            submission_id: Submission ID
            
        Returns:
            FlightPlanSubmissionResponse: Submission response
        """
        try:
            # Check if client is initialized
            if not self.client:
                raise ValueError("EASA SORA adapter not initialized")
            
            # Create request
            request = self.client.create_message(
                self.client.service,
                "checkSubmissionStatus",
                submissionId=submission_id,
            )
            
            # Submit request
            async with httpx.AsyncClient() as http_client:
                # Call SOAP service
                response = await http_client.post(
                    self.api_url,
                    headers={
                        "Content-Type": "text/xml",
                        "SOAPAction": "checkSubmissionStatus",
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    content=request,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                soap_response = self.client.process_reply(response.text)
                
                # Extract status
                status = soap_response["status"]
                
                # Map status to FlightPlanStatus
                if status == "APPROVED":
                    flight_plan_status = FlightPlanStatus.APPROVED
                elif status == "REJECTED":
                    flight_plan_status = FlightPlanStatus.REJECTED
                else:
                    flight_plan_status = FlightPlanStatus.SUBMITTED
                
                # Create response
                return FlightPlanSubmissionResponse(
                    flight_plan_id=soap_response.get("flightPlanId"),
                    submission_id=submission_id,
                    submission_time=soap_response.get("submissionTime"),
                    status=flight_plan_status,
                    message=soap_response.get("message"),
                    details=soap_response,
                )
        except Exception as e:
            logger.error(f"Error checking submission status: {str(e)}")
            raise
    
    async def cancel_submission(
        self,
        submission_id: str,
        reason: Optional[str] = None,
    ) -> FlightPlanSubmissionResponse:
        """
        Cancel a flight plan submission.
        
        Args:
            submission_id: Submission ID
            reason: Cancellation reason
            
        Returns:
            FlightPlanSubmissionResponse: Submission response
        """
        try:
            # Check if client is initialized
            if not self.client:
                raise ValueError("EASA SORA adapter not initialized")
            
            # Create request
            request = self.client.create_message(
                self.client.service,
                "cancelSubmission",
                submissionId=submission_id,
                reason=reason or "Cancelled by user",
            )
            
            # Submit request
            async with httpx.AsyncClient() as http_client:
                # Call SOAP service
                response = await http_client.post(
                    self.api_url,
                    headers={
                        "Content-Type": "text/xml",
                        "SOAPAction": "cancelSubmission",
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    content=request,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                soap_response = self.client.process_reply(response.text)
                
                # Create response
                return FlightPlanSubmissionResponse(
                    flight_plan_id=soap_response.get("flightPlanId"),
                    submission_id=submission_id,
                    submission_time=datetime.utcnow(),
                    status=FlightPlanStatus.CANCELLED,
                    message=soap_response.get("message"),
                    details=soap_response,
                )
        except Exception as e:
            logger.error(f"Error cancelling submission: {str(e)}")
            raise
    
    def _create_soap_request(self, submission: EASASoraSubmission) -> str:
        """
        Create a SOAP request for a flight plan submission.
        
        Args:
            submission: EASA SORA submission
            
        Returns:
            str: SOAP request
        """
        # Create request
        request = self.client.create_message(
            self.client.service,
            "submitFlightPlan",
            flightPlanId=str(submission.flight_plan_id),
            riskClass=submission.risk_class,
            operationalVolume={
                "area": self._convert_polygon_to_soap(submission.operational_volume.area),
                "minHeight": submission.operational_volume.min_height,
                "maxHeight": submission.operational_volume.max_height,
                "buffer": submission.operational_volume.buffer,
            },
            startTime=submission.start_time.isoformat(),
            endTime=submission.end_time.isoformat(),
            droneDetails=json.dumps(submission.drone_details),
            operatorDetails=json.dumps(submission.operator_details),
            operationDetails=json.dumps(submission.operation_details),
            mitigations=json.dumps(submission.mitigations) if submission.mitigations else None,
        )
        
        return request
    
    def _convert_polygon_to_soap(self, polygon: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert a polygon to SOAP format.
        
        Args:
            polygon: Polygon
            
        Returns:
            Dict[str, Any]: SOAP polygon
        """
        # Extract points
        points = polygon.get("points", [])
        
        # Convert to SOAP format
        soap_points = []
        for point in points:
            soap_points.append({
                "latitude": point.get("latitude"),
                "longitude": point.get("longitude"),
                "altitude": point.get("altitude"),
            })
        
        return {"points": soap_points}
