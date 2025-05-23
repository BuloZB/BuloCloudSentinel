"""
NOTAM AIXM adapter for the Remote ID & Regulatory Compliance Service.

This module provides an adapter for NOTAM AIXM data.
"""

import logging
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any

import httpx
import xmltodict
from lxml import etree

from remoteid_service.api.schemas.notams import (
    NOTAMType,
    NOTAMSource,
)

# Configure logging
logger = logging.getLogger(__name__)

class NOTAMAIXMAdapter:
    """
    NOTAM AIXM adapter.
    
    This adapter provides functionality for fetching and parsing NOTAM data
    in AIXM 5.1 format.
    """
    
    def __init__(self, source: NOTAMSource):
        """
        Initialize the NOTAM AIXM adapter.
        
        Args:
            source: NOTAM source
        """
        self.source = source
        
        # Set up API URLs based on source
        if source == NOTAMSource.FAA:
            self.api_url = "https://external-api.faa.gov/notams/v2"
        elif source == NOTAMSource.EASA:
            self.api_url = "https://api.easa.europa.eu/notams/v1"
        elif source == NOTAMSource.EUROCONTROL:
            self.api_url = "https://api.eurocontrol.int/notams/v1"
        elif source == NOTAMSource.ICAO:
            self.api_url = "https://api.icao.int/notams/v1"
        else:
            self.api_url = None
    
    async def fetch_notams(
        self,
        region: Optional[str] = None,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        location_code: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        Fetch NOTAMs from the source.
        
        Args:
            region: Region
            start_time: Start time
            end_time: End time
            location_code: Location code
            
        Returns:
            List[Dict[str, Any]]: NOTAMs
        """
        try:
            # Check if API URL is set
            if not self.api_url:
                # For demonstration, return simulated NOTAMs
                return self._get_simulated_notams(
                    region=region,
                    start_time=start_time,
                    end_time=end_time,
                    location_code=location_code,
                )
            
            # Set up query parameters
            params = {}
            
            if region:
                params["region"] = region
            
            if start_time:
                params["startTime"] = start_time.isoformat()
            
            if end_time:
                params["endTime"] = end_time.isoformat()
            
            if location_code:
                params["location"] = location_code
            
            # Fetch NOTAMs
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    f"{self.api_url}/aixm",
                    params=params,
                    timeout=30,
                )
                
                # Check response
                response.raise_for_status()
                
                # Parse AIXM XML
                aixm_data = response.text
                notams = self._parse_aixm(aixm_data)
                
                return notams
        except Exception as e:
            logger.error(f"Error fetching NOTAMs: {str(e)}")
            
            # For demonstration, return simulated NOTAMs on error
            return self._get_simulated_notams(
                region=region,
                start_time=start_time,
                end_time=end_time,
                location_code=location_code,
            )
    
    def _parse_aixm(self, aixm_data: str) -> List[Dict[str, Any]]:
        """
        Parse AIXM data.
        
        Args:
            aixm_data: AIXM XML data
            
        Returns:
            List[Dict[str, Any]]: NOTAMs
        """
        try:
            # Parse XML
            root = etree.fromstring(aixm_data.encode("utf-8"))
            
            # Convert to dict
            data = xmltodict.parse(aixm_data)
            
            # Extract NOTAMs
            notams = []
            
            # Process AIXM messages
            messages = data.get("AIXMBasicMessage", {}).get("hasMember", [])
            if not isinstance(messages, list):
                messages = [messages]
            
            for message in messages:
                try:
                    # Extract NOTAM data
                    notam = self._extract_notam_from_aixm(message)
                    
                    if notam:
                        notams.append(notam)
                except Exception as e:
                    logger.error(f"Error extracting NOTAM from AIXM: {str(e)}")
            
            return notams
        except Exception as e:
            logger.error(f"Error parsing AIXM data: {str(e)}")
            return []
    
    def _extract_notam_from_aixm(self, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Extract NOTAM data from AIXM message.
        
        Args:
            message: AIXM message
            
        Returns:
            Optional[Dict[str, Any]]: NOTAM data
        """
        try:
            # Extract NOTAM ID
            notam_id = message.get("NOTAM", {}).get("identifier", {}).get("value")
            
            if not notam_id:
                return None
            
            # Extract location
            location = message.get("NOTAM", {}).get("location", {}).get("value")
            
            # Extract effective times
            effective_start = message.get("NOTAM", {}).get("effectiveStart", {}).get("value")
            effective_end = message.get("NOTAM", {}).get("effectiveEnd", {}).get("value")
            
            # Convert times to datetime
            if effective_start:
                effective_start = datetime.fromisoformat(effective_start.replace("Z", "+00:00"))
            else:
                effective_start = datetime.utcnow()
            
            if effective_end:
                effective_end = datetime.fromisoformat(effective_end.replace("Z", "+00:00"))
            else:
                effective_end = effective_start + timedelta(days=30)
            
            # Extract NOTAM type
            notam_type_str = message.get("NOTAM", {}).get("type", {}).get("value")
            
            # Map NOTAM type
            if notam_type_str == "AIRSPACE":
                notam_type = NOTAMType.AIRSPACE
            elif notam_type_str == "OBSTACLE":
                notam_type = NOTAMType.OBSTACLE
            elif notam_type_str == "AIRPORT":
                notam_type = NOTAMType.AIRPORT
            elif notam_type_str == "PROCEDURE":
                notam_type = NOTAMType.PROCEDURE
            else:
                notam_type = NOTAMType.OTHER
            
            # Extract altitude limits
            altitude_lower = message.get("NOTAM", {}).get("altitudeLower", {}).get("value")
            altitude_upper = message.get("NOTAM", {}).get("altitudeUpper", {}).get("value")
            
            # Convert altitude to float
            if altitude_lower:
                altitude_lower = float(altitude_lower)
            
            if altitude_upper:
                altitude_upper = float(altitude_upper)
            
            # Extract area
            area = message.get("NOTAM", {}).get("geometry", {}).get("Polygon", {})
            
            # Extract point
            point = message.get("NOTAM", {}).get("geometry", {}).get("Point", {})
            
            # Extract description
            description = message.get("NOTAM", {}).get("text", {}).get("value")
            
            # Extract raw text
            raw_text = message.get("NOTAM", {}).get("rawText", {}).get("value")
            
            # Create NOTAM data
            notam_data = {
                "id": uuid.uuid4(),
                "notam_id": notam_id,
                "source": self.source,
                "notam_type": notam_type,
                "location": location,
                "effective_start": effective_start,
                "effective_end": effective_end,
                "altitude_lower": altitude_lower,
                "altitude_upper": altitude_upper,
                "area": area,
                "point": point,
                "description": description,
                "raw_text": raw_text,
                "metadata": message,
                "created_at": datetime.utcnow(),
                "updated_at": datetime.utcnow(),
            }
            
            return notam_data
        except Exception as e:
            logger.error(f"Error extracting NOTAM from AIXM: {str(e)}")
            return None
    
    def _get_simulated_notams(
        self,
        region: Optional[str] = None,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        location_code: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        Get simulated NOTAMs for demonstration.
        
        Args:
            region: Region
            start_time: Start time
            end_time: End time
            location_code: Location code
            
        Returns:
            List[Dict[str, Any]]: Simulated NOTAMs
        """
        # Set default times
        if not start_time:
            start_time = datetime.utcnow()
        
        if not end_time:
            end_time = start_time + timedelta(days=30)
        
        # Set default location
        if not location_code:
            if region == "us-east":
                location_code = "KJFK"
            elif region == "us-west":
                location_code = "KLAX"
            elif region == "eu":
                location_code = "EHAM"
            else:
                location_code = "KJFK"
        
        # Create simulated NOTAMs
        notams = []
        
        # NOTAM 1: Airspace restriction
        notams.append({
            "id": uuid.uuid4(),
            "notam_id": f"{location_code}A001/23",
            "source": self.source,
            "notam_type": NOTAMType.AIRSPACE,
            "location": location_code,
            "effective_start": start_time,
            "effective_end": end_time,
            "altitude_lower": 0,
            "altitude_upper": 5000,
            "area": {
                "type": "Polygon",
                "coordinates": [
                    [
                        [-74.0, 40.7],
                        [-74.1, 40.7],
                        [-74.1, 40.8],
                        [-74.0, 40.8],
                        [-74.0, 40.7],
                    ]
                ]
            },
            "point": None,
            "description": "Temporary flight restriction due to special event",
            "raw_text": f"{location_code}A001/23 NOTAM TEMPORARY FLIGHT RESTRICTION DUE TO SPECIAL EVENT",
            "metadata": {},
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        })
        
        # NOTAM 2: Obstacle
        notams.append({
            "id": uuid.uuid4(),
            "notam_id": f"{location_code}O002/23",
            "source": self.source,
            "notam_type": NOTAMType.OBSTACLE,
            "location": location_code,
            "effective_start": start_time,
            "effective_end": end_time,
            "altitude_lower": 0,
            "altitude_upper": 500,
            "area": None,
            "point": {
                "type": "Point",
                "coordinates": [-74.05, 40.75]
            },
            "description": "Temporary crane operation",
            "raw_text": f"{location_code}O002/23 NOTAM TEMPORARY CRANE OPERATION",
            "metadata": {},
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        })
        
        # NOTAM 3: Airport
        notams.append({
            "id": uuid.uuid4(),
            "notam_id": f"{location_code}A003/23",
            "source": self.source,
            "notam_type": NOTAMType.AIRPORT,
            "location": location_code,
            "effective_start": start_time,
            "effective_end": end_time,
            "altitude_lower": 0,
            "altitude_upper": 1000,
            "area": {
                "type": "Polygon",
                "coordinates": [
                    [
                        [-74.0, 40.7],
                        [-74.05, 40.7],
                        [-74.05, 40.75],
                        [-74.0, 40.75],
                        [-74.0, 40.7],
                    ]
                ]
            },
            "point": None,
            "description": "Runway closure for maintenance",
            "raw_text": f"{location_code}A003/23 NOTAM RUNWAY CLOSURE FOR MAINTENANCE",
            "metadata": {},
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        })
        
        return notams
