"""
Threat detector service for the SIGINT service.

This service is responsible for detecting threats from signal intelligence, including:
- Matching signals against known threat profiles
- Detecting jamming attempts
- Identifying unauthorized transmissions
- Detecting anomalous signal behavior
"""

import logging
from typing import Dict, List, Optional, Any
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from core.config import settings
from db.models import KnownSignalProfile, SigintAlert

logger = logging.getLogger(__name__)

class ThreatDetector:
    """Threat detector service."""
    
    def __init__(self):
        """Initialize the threat detector."""
        self.confidence_threshold = settings.THREAT_DETECTION_CONFIDENCE_THRESHOLD
    
    async def check_detection(self, detection: Dict[str, Any], signal_analyzer, db: AsyncSession) -> Optional[Dict[str, Any]]:
        """
        Check a signal detection for threats.
        
        Args:
            detection: Signal detection
            signal_analyzer: Signal analyzer service
            db: Database session
            
        Returns:
            Threat alert if a threat is detected, None otherwise
        """
        # Extract signal features
        features = detection.get("metadata", {}).get("features")
        
        if not features:
            logger.warning(f"No features available for detection {detection.get('id')}")
            return None
        
        # Get known signal profiles
        result = await db.execute(select(KnownSignalProfile))
        profiles = result.scalars().all()
        
        # Match against known profiles
        profile, confidence = await signal_analyzer.match_signal_profile(
            features,
            [
                {
                    "id": str(p.id),
                    "name": p.name,
                    "signal_type": p.signal_type,
                    "features": p.features,
                    "threat_level": p.threat_level
                }
                for p in profiles
            ]
        )
        
        # Check if match is a threat
        if profile and profile["threat_level"] in ["medium", "high", "critical"] and confidence >= self.confidence_threshold:
            # Create threat alert
            alert = {
                "type": "known_threat",
                "severity": self._map_threat_level_to_severity(profile["threat_level"]),
                "message": f"Detected {profile['name']} signal with {profile['threat_level']} threat level",
                "timestamp": datetime.utcnow(),
                "frequency": detection.get("frequency"),
                "location": detection.get("location"),
                "source_id": detection.get("source_id"),
                "detection_id": detection.get("id"),
                "metadata": {
                    "profile_id": profile["id"],
                    "profile_name": profile["name"],
                    "confidence": confidence,
                    "signal_type": detection.get("signal_type")
                }
            }
            
            return alert
        
        # Check for jamming
        jamming_alert = await self._check_for_jamming(detection)
        if jamming_alert:
            return jamming_alert
        
        # Check for unauthorized transmission
        unauthorized_alert = await self._check_for_unauthorized(detection)
        if unauthorized_alert:
            return unauthorized_alert
        
        return None
    
    async def check_source(self, source: Dict[str, Any], db: AsyncSession) -> Optional[Dict[str, Any]]:
        """
        Check a signal source for threats.
        
        Args:
            source: Signal source
            db: Database session
            
        Returns:
            Threat alert if a threat is detected, None otherwise
        """
        # Check if source is already identified as a threat
        if source.get("threat_level") in ["medium", "high", "critical"]:
            # Create threat alert
            alert = {
                "type": "known_threat",
                "severity": self._map_threat_level_to_severity(source["threat_level"]),
                "message": f"Signal source with {source['threat_level']} threat level active",
                "timestamp": datetime.utcnow(),
                "frequency": source.get("frequency_range", {}).get("min_freq"),
                "location": source.get("location"),
                "source_id": source.get("id"),
                "metadata": {
                    "identification": source.get("identification"),
                    "signal_type": source.get("signal_type"),
                    "frequency_range": source.get("frequency_range")
                }
            }
            
            return alert
        
        return None
    
    async def detect_anomalies(self, detections: List[Dict[str, Any]], db: AsyncSession) -> List[Dict[str, Any]]:
        """
        Detect anomalous signal behavior.
        
        Args:
            detections: List of signal detections
            db: Database session
            
        Returns:
            List of anomaly alerts
        """
        # In a real implementation, this would use more sophisticated anomaly detection
        # For now, we'll use a simple approach
        
        alerts = []
        
        # Group detections by frequency
        frequency_groups = {}
        for detection in detections:
            freq = detection.get("frequency")
            if freq:
                if freq not in frequency_groups:
                    frequency_groups[freq] = []
                frequency_groups[freq].append(detection)
        
        # Check for sudden appearance of strong signals
        for freq, group in frequency_groups.items():
            if len(group) < 2:
                continue
            
            # Sort by timestamp
            sorted_group = sorted(group, key=lambda d: d.get("timestamp", datetime.min))
            
            # Check for sudden increase in signal strength
            for i in range(1, len(sorted_group)):
                prev_strength = sorted_group[i-1].get("signal_strength", -100)
                curr_strength = sorted_group[i].get("signal_strength", -100)
                
                if curr_strength > prev_strength + 20:  # 20 dB increase
                    # Create anomaly alert
                    alert = {
                        "type": "anomaly",
                        "severity": "warning",
                        "message": f"Sudden increase in signal strength at {freq/1e6:.3f} MHz",
                        "timestamp": sorted_group[i].get("timestamp", datetime.utcnow()),
                        "frequency": freq,
                        "location": sorted_group[i].get("location"),
                        "detection_id": sorted_group[i].get("id"),
                        "metadata": {
                            "previous_strength": prev_strength,
                            "current_strength": curr_strength,
                            "increase": curr_strength - prev_strength
                        }
                    }
                    
                    alerts.append(alert)
        
        return alerts
    
    async def _check_for_jamming(self, detection: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Check for jamming signals.
        
        Args:
            detection: Signal detection
            
        Returns:
            Jamming alert if jamming is detected, None otherwise
        """
        # In a real implementation, this would use more sophisticated jamming detection
        # For now, we'll use a simple approach based on signal characteristics
        
        # Check for wideband, high-power signals
        if (detection.get("bandwidth", 0) > 1e6 and  # > 1 MHz bandwidth
            detection.get("signal_strength", -100) > -50):  # > -50 dBm
            
            # Create jamming alert
            alert = {
                "type": "jamming",
                "severity": "critical",
                "message": f"Possible jamming signal detected at {detection.get('frequency')/1e6:.3f} MHz",
                "timestamp": detection.get("timestamp", datetime.utcnow()),
                "frequency": detection.get("frequency"),
                "location": detection.get("location"),
                "detection_id": detection.get("id"),
                "metadata": {
                    "bandwidth": detection.get("bandwidth"),
                    "signal_strength": detection.get("signal_strength"),
                    "signal_type": detection.get("signal_type")
                }
            }
            
            return alert
        
        return None
    
    async def _check_for_unauthorized(self, detection: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Check for unauthorized transmissions.
        
        Args:
            detection: Signal detection
            
        Returns:
            Unauthorized alert if unauthorized transmission is detected, None otherwise
        """
        # In a real implementation, this would check against authorized frequency allocations
        # For now, we'll use a simple approach with hardcoded restricted frequencies
        
        # Define some example restricted frequencies (in Hz)
        restricted_frequencies = [
            (430e6, 432e6),  # 430-432 MHz
            (902e6, 928e6),  # 902-928 MHz
        ]
        
        # Check if detection frequency is in a restricted range
        freq = detection.get("frequency", 0)
        for min_freq, max_freq in restricted_frequencies:
            if min_freq <= freq <= max_freq:
                # Create unauthorized alert
                alert = {
                    "type": "unauthorized",
                    "severity": "warning",
                    "message": f"Unauthorized transmission detected at {freq/1e6:.3f} MHz",
                    "timestamp": detection.get("timestamp", datetime.utcnow()),
                    "frequency": freq,
                    "location": detection.get("location"),
                    "detection_id": detection.get("id"),
                    "metadata": {
                        "restricted_range": f"{min_freq/1e6:.3f}-{max_freq/1e6:.3f} MHz",
                        "signal_strength": detection.get("signal_strength"),
                        "signal_type": detection.get("signal_type")
                    }
                }
                
                return alert
        
        return None
    
    def _map_threat_level_to_severity(self, threat_level: str) -> str:
        """
        Map threat level to alert severity.
        
        Args:
            threat_level: Threat level
            
        Returns:
            Alert severity
        """
        if threat_level == "critical":
            return "critical"
        elif threat_level == "high":
            return "critical"
        elif threat_level == "medium":
            return "warning"
        else:
            return "info"
