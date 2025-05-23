"""
Feedback Service for Voice & Gesture Co-Pilot.

This service provides user feedback mechanisms for voice and gesture recognition,
including visual and audio feedback.
"""

import os
import time
import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from pathlib import Path

import aiofiles
from loguru import logger

from voice_gesture_copilot.core.config import settings

class FeedbackService:
    """
    Service for providing user feedback.
    
    This service provides methods for generating visual and audio feedback
    for voice and gesture recognition.
    """
    
    def __init__(self):
        """Initialize the feedback service."""
        self.is_initialized = True
        self.feedback_history = []
        
        # Create feedback directory if it doesn't exist
        self.feedback_dir = Path("data/feedback")
        self.feedback_dir.mkdir(parents=True, exist_ok=True)
    
    async def cleanup(self):
        """
        Clean up resources used by the feedback service.
        """
        logger.info("Cleaning up feedback service resources")
        self.is_initialized = False
        logger.info("Feedback service resources cleaned up")
    
    async def generate_visual_feedback(self, feedback_type: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate visual feedback for the user.
        
        Args:
            feedback_type: Type of feedback (e.g., "voice_recognition", "gesture_recognition")
            data: Feedback data
            
        Returns:
            Dictionary containing feedback information
        """
        if not self.is_initialized:
            logger.error("Feedback service not initialized")
            return {"error": "Feedback service not initialized"}
        
        try:
            # Generate feedback based on type
            feedback = {
                "type": feedback_type,
                "timestamp": datetime.now().isoformat(),
                "data": data,
            }
            
            # Add to history
            self.feedback_history.append(feedback)
            
            # Save feedback to file for analysis
            await self._save_feedback(feedback)
            
            return feedback
        
        except Exception as e:
            logger.error(f"Error generating visual feedback: {str(e)}")
            return {"error": str(e)}
    
    async def generate_audio_feedback(self, feedback_type: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate audio feedback for the user.
        
        Args:
            feedback_type: Type of feedback (e.g., "command_confirmation", "error")
            data: Feedback data
            
        Returns:
            Dictionary containing feedback information
        """
        if not self.is_initialized:
            logger.error("Feedback service not initialized")
            return {"error": "Feedback service not initialized"}
        
        try:
            # Generate feedback based on type
            feedback = {
                "type": feedback_type,
                "timestamp": datetime.now().isoformat(),
                "data": data,
            }
            
            # Add to history
            self.feedback_history.append(feedback)
            
            # Save feedback to file for analysis
            await self._save_feedback(feedback)
            
            return feedback
        
        except Exception as e:
            logger.error(f"Error generating audio feedback: {str(e)}")
            return {"error": str(e)}
    
    async def _save_feedback(self, feedback: Dict[str, Any]) -> None:
        """
        Save feedback to file for analysis.
        
        Args:
            feedback: Feedback data
        """
        try:
            # Generate filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{feedback['type']}_{timestamp}.json"
            filepath = self.feedback_dir / filename
            
            # Save to file
            import json
            async with aiofiles.open(filepath, "w") as f:
                await f.write(json.dumps(feedback, indent=2))
        
        except Exception as e:
            logger.error(f"Error saving feedback: {str(e)}")
    
    async def get_feedback_history(self, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Get feedback history.
        
        Args:
            limit: Maximum number of feedback items to return
            
        Returns:
            List of feedback items
        """
        if not self.is_initialized:
            logger.error("Feedback service not initialized")
            return []
        
        # Return most recent feedback items
        return self.feedback_history[-limit:]
    
    async def status(self) -> Dict[str, Any]:
        """
        Get the status of the feedback service.
        
        Returns:
            Dictionary containing service status
        """
        status_info = {
            "initialized": self.is_initialized,
            "feedback_history_count": len(self.feedback_history),
            "feedback_dir": str(self.feedback_dir),
        }
        
        return status_info
