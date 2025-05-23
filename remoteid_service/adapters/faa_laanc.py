"""
FAA LAANC adapter for the Remote ID & Regulatory Compliance Service.

This module provides an adapter for the FAA LAANC API.
"""

import logging
import json
from datetime import datetime
from typing import Dict, List, Optional, Any

import httpx

from remoteid_service.api.schemas.flightplans import (
    FlightPlanSubmissionResponse,
    FlightPlanStatus,
    FAALaancOperationType,
    FAALaancSubmission,
)

# Configure logging
logger = logging.getLogger(__name__)

class FAALaancAdapter:
    """
    FAA LAANC adapter.
    
    This adapter provides functionality for submitting flight plans to the
    FAA LAANC API.
    """
    
    def __init__(self, api_url: str, api_key: str):
        """
        Initialize the FAA LAANC adapter.
        
        Args:
            api_url: API URL
            api_key: API key
        """
        self.api_url = api_url
        self.api_key = api_key
    
    async def submit_flight_plan(
        self,
        submission: FAALaancSubmission,
    ) -> FlightPlanSubmissionResponse:
        """
        Submit a flight plan to the FAA LAANC API.
        
        Args:
            submission: FAA LAANC submission
            
        Returns:
            FlightPlanSubmissionResponse: Submission response
        """
        try:
            # Convert submission to API request
            request_data = self._create_api_request(submission)
            
            # Submit request
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    f"{self.api_url}/flight-plans",
                    headers={
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    json=request_data,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                response_data = response.json()
                
                # Extract submission ID and status
                submission_id = response_data.get("submissionId")
                status = response_data.get("status")
                
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
                    message=response_data.get("message"),
                    details=response_data,
                )
        except Exception as e:
            logger.error(f"Error submitting flight plan to FAA LAANC: {str(e)}")
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
            # Submit request
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/flight-plans/{submission_id}",
                    headers={
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                response_data = response.json()
                
                # Extract status
                status = response_data.get("status")
                
                # Map status to FlightPlanStatus
                if status == "APPROVED":
                    flight_plan_status = FlightPlanStatus.APPROVED
                elif status == "REJECTED":
                    flight_plan_status = FlightPlanStatus.REJECTED
                else:
                    flight_plan_status = FlightPlanStatus.SUBMITTED
                
                # Create response
                return FlightPlanSubmissionResponse(
                    flight_plan_id=response_data.get("flightPlanId"),
                    submission_id=submission_id,
                    submission_time=datetime.fromisoformat(response_data.get("submissionTime")),
                    status=flight_plan_status,
                    message=response_data.get("message"),
                    details=response_data,
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
            # Submit request
            async with httpx.AsyncClient() as client:
                response = await client.delete(
                    f"{self.api_url}/flight-plans/{submission_id}",
                    headers={
                        "Authorization": f"Bearer {self.api_key}",
                    },
                    params={
                        "reason": reason or "Cancelled by user",
                    },
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse response
                response_data = response.json()
                
                # Create response
                return FlightPlanSubmissionResponse(
                    flight_plan_id=response_data.get("flightPlanId"),
                    submission_id=submission_id,
                    submission_time=datetime.utcnow(),
                    status=FlightPlanStatus.CANCELLED,
                    message=response_data.get("message"),
                    details=response_data,
                )
        except Exception as e:
            logger.error(f"Error cancelling submission: {str(e)}")
            raise
    
    def _create_api_request(self, submission: FAALaancSubmission) -> Dict[str, Any]:
        """
        Create an API request for a flight plan submission.
        
        Args:
            submission: FAA LAANC submission
            
        Returns:
            Dict[str, Any]: API request
        """
        # Convert area to GeoJSON
        area_geojson = {
            "type": "Polygon",
            "coordinates": [
                [
                    [point.longitude, point.latitude]
                    for point in submission.area.points
                ]
            ]
        }
        
        # Create request
        request = {
            "flightPlanId": str(submission.flight_plan_id),
            "operationType": submission.operation_type,
            "airspaceClass": submission.airspace_class,
            "area": area_geojson,
            "maxAltitude": submission.max_altitude,
            "startTime": submission.start_time.isoformat(),
            "endTime": submission.end_time.isoformat(),
            "pilotDetails": submission.pilot_details,
            "aircraftDetails": submission.aircraft_details,
            "operationDetails": submission.operation_details,
        }
        
        return request
