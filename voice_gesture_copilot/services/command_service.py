"""
Command Service for Voice & Gesture Co-Pilot.

This service processes recognized voice commands and gestures,
translating them into drone operations.
"""

import os
import time
import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime

import networkx as nx
from loguru import logger

from voice_gesture_copilot.core.config import settings
from voice_gesture_copilot.models.rasa_nlu.intent_classifier import IntentClassifier
from voice_gesture_copilot.models.mission_graph.mission_graph import MissionGraph
from voice_gesture_copilot.services.drone_service import DroneService

class CommandService:
    """
    Service for processing commands and translating them into drone operations.
    
    This service provides methods for processing voice commands and gestures,
    validating them, and sending appropriate commands to drones.
    """
    
    def __init__(self, drone_service: DroneService):
        """
        Initialize the command service.
        
        Args:
            drone_service: Service for interacting with drones
        """
        self.drone_service = drone_service
        self.intent_classifier = None
        self.mission_graph = None
        self.is_initialized = False
        
        # Command history
        self.command_history = []
        
        # Command cooldowns
        self.command_cooldowns = {}
        
        # Performance metrics
        self.performance_metrics = {
            "voice_command_processing": {
                "latency_ms": [],
                "success_rate": [],
            },
            "gesture_processing": {
                "latency_ms": [],
                "success_rate": [],
            },
        }
    
    async def load_models(self):
        """
        Load the command processing models.
        
        This method initializes the Rasa NLU intent classifier and
        Mission Graph system.
        """
        logger.info("Loading command processing models")
        
        # Initialize Rasa NLU intent classifier
        self.intent_classifier = IntentClassifier(
            model_dir=settings.RASA_MODEL_DIR,
            confidence_threshold=settings.COMMAND_CONFIDENCE_THRESHOLD,
            voice_commands=settings.VOICE_COMMANDS,
        )
        await self.intent_classifier.initialize()
        
        # Initialize Mission Graph
        self.mission_graph = MissionGraph()
        await self.mission_graph.initialize()
        
        self.is_initialized = True
        logger.info("Command processing models loaded successfully")
    
    async def cleanup(self):
        """
        Clean up resources used by the command service.
        
        This method releases resources used by the Rasa NLU intent classifier
        and Mission Graph system.
        """
        logger.info("Cleaning up command service resources")
        
        if self.intent_classifier:
            await self.intent_classifier.cleanup()
        
        if self.mission_graph:
            await self.mission_graph.cleanup()
        
        self.is_initialized = False
        logger.info("Command service resources cleaned up")
    
    async def process_voice_command(self, command_text: str) -> Dict[str, Any]:
        """
        Process a voice command.
        
        Args:
            command_text: Text of the voice command
            
        Returns:
            Dictionary containing processing results
        """
        if not self.is_initialized:
            logger.error("Command service not initialized")
            return {"error": "Command service not initialized"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # Classify intent
            intent_result = await self.intent_classifier.classify(command_text)
            
            # Extract intent and confidence
            intent = intent_result.get("intent", "")
            confidence = intent_result.get("confidence", 0.0)
            
            # Check if intent is recognized with sufficient confidence
            if not intent or confidence < settings.COMMAND_CONFIDENCE_THRESHOLD:
                logger.warning(f"Intent not recognized with sufficient confidence: {intent} ({confidence:.2f})")
                return {
                    "success": False,
                    "message": "Command not recognized with sufficient confidence",
                    "intent": intent,
                    "confidence": confidence,
                    "latency_ms": (time.time() - start_time) * 1000,
                }
            
            # Check if command requires confirmation
            requires_confirmation = False
            if intent in settings.VOICE_COMMANDS:
                requires_confirmation = settings.VOICE_COMMANDS[intent].get("requires_confirmation", False)
            
            # Check command cooldown
            current_time = time.time()
            if intent in self.command_cooldowns:
                cooldown_time = self.command_cooldowns[intent]
                if current_time < cooldown_time:
                    remaining_cooldown = cooldown_time - current_time
                    logger.warning(f"Command {intent} is on cooldown for {remaining_cooldown:.1f} seconds")
                    return {
                        "success": False,
                        "message": f"Command is on cooldown for {remaining_cooldown:.1f} seconds",
                        "intent": intent,
                        "confidence": confidence,
                        "latency_ms": (time.time() - start_time) * 1000,
                    }
            
            # Process command through mission graph
            mission_result = await self.mission_graph.process_command(intent, intent_result)
            
            # If mission graph processing was successful, send command to drone
            if mission_result.get("success", False):
                # Get drone command from mission result
                drone_command = mission_result.get("drone_command", {})
                
                # Send command to drone if not in confirmation mode
                if not requires_confirmation:
                    drone_result = await self.drone_service.send_command(
                        drone_command.get("drone_id", ""),
                        drone_command.get("command", ""),
                        drone_command.get("parameters", {}),
                    )
                    
                    # Update mission result with drone result
                    mission_result["drone_result"] = drone_result
                else:
                    # If confirmation is required, store command for later execution
                    mission_result["requires_confirmation"] = True
                    mission_result["pending_command"] = drone_command
            
            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000
            self.performance_metrics["voice_command_processing"]["latency_ms"].append(latency_ms)
            
            # Update success rate
            self.performance_metrics["voice_command_processing"]["success_rate"].append(
                1.0 if mission_result.get("success", False) else 0.0
            )
            
            # Add command to history
            self.command_history.append({
                "timestamp": datetime.now().isoformat(),
                "type": "voice",
                "command_text": command_text,
                "intent": intent,
                "confidence": confidence,
                "success": mission_result.get("success", False),
                "latency_ms": latency_ms,
            })
            
            # Set command cooldown if successful
            if mission_result.get("success", False) and intent in settings.VOICE_COMMANDS:
                cooldown_seconds = 1.0  # Default cooldown
                self.command_cooldowns[intent] = current_time + cooldown_seconds
            
            # Add latency to result
            mission_result["latency_ms"] = latency_ms
            
            return mission_result
        
        except Exception as e:
            logger.error(f"Error processing voice command: {str(e)}")
            return {"error": str(e)}
    
    async def process_gesture(self, gesture_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process a recognized gesture.
        
        Args:
            gesture_data: Gesture recognition data
            
        Returns:
            Dictionary containing processing results
        """
        if not self.is_initialized:
            logger.error("Command service not initialized")
            return {"error": "Command service not initialized"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # Extract gesture and confidence
            gesture = gesture_data.get("gesture", "")
            confidence = gesture_data.get("confidence", 0.0)
            
            # Check if gesture is recognized with sufficient confidence
            if not gesture or confidence < settings.GESTURE_DEFINITIONS.get(gesture, {}).get("confidence_threshold", 0.8):
                logger.warning(f"Gesture not recognized with sufficient confidence: {gesture} ({confidence:.2f})")
                return {
                    "success": False,
                    "message": "Gesture not recognized with sufficient confidence",
                    "gesture": gesture,
                    "confidence": confidence,
                    "latency_ms": (time.time() - start_time) * 1000,
                }
            
            # Check gesture cooldown
            current_time = time.time()
            cooldown_key = f"gesture_{gesture}"
            if cooldown_key in self.command_cooldowns:
                cooldown_time = self.command_cooldowns[cooldown_key]
                if current_time < cooldown_time:
                    remaining_cooldown = cooldown_time - current_time
                    logger.warning(f"Gesture {gesture} is on cooldown for {remaining_cooldown:.1f} seconds")
                    return {
                        "success": False,
                        "message": f"Gesture is on cooldown for {remaining_cooldown:.1f} seconds",
                        "gesture": gesture,
                        "confidence": confidence,
                        "latency_ms": (time.time() - start_time) * 1000,
                    }
            
            # Map gesture to intent
            intent = gesture  # In this simple implementation, gesture name is the intent
            
            # Process command through mission graph
            mission_result = await self.mission_graph.process_command(intent, {
                "intent": intent,
                "confidence": confidence,
                "gesture_data": gesture_data,
            })
            
            # If mission graph processing was successful, send command to drone
            if mission_result.get("success", False):
                # Get drone command from mission result
                drone_command = mission_result.get("drone_command", {})
                
                # Send command to drone
                drone_result = await self.drone_service.send_command(
                    drone_command.get("drone_id", ""),
                    drone_command.get("command", ""),
                    drone_command.get("parameters", {}),
                )
                
                # Update mission result with drone result
                mission_result["drone_result"] = drone_result
            
            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000
            self.performance_metrics["gesture_processing"]["latency_ms"].append(latency_ms)
            
            # Update success rate
            self.performance_metrics["gesture_processing"]["success_rate"].append(
                1.0 if mission_result.get("success", False) else 0.0
            )
            
            # Add command to history
            self.command_history.append({
                "timestamp": datetime.now().isoformat(),
                "type": "gesture",
                "gesture": gesture,
                "confidence": confidence,
                "success": mission_result.get("success", False),
                "latency_ms": latency_ms,
            })
            
            # Set gesture cooldown if successful
            if mission_result.get("success", False) and gesture in settings.GESTURE_DEFINITIONS:
                cooldown_seconds = settings.GESTURE_DEFINITIONS[gesture].get("cooldown_seconds", 1.0)
                self.command_cooldowns[cooldown_key] = current_time + cooldown_seconds
            
            # Add latency to result
            mission_result["latency_ms"] = latency_ms
            
            return mission_result
        
        except Exception as e:
            logger.error(f"Error processing gesture: {str(e)}")
            return {"error": str(e)}
    
    async def status(self) -> Dict[str, Any]:
        """
        Get the status of the command service.
        
        Returns:
            Dictionary containing service status
        """
        import numpy as np
        
        status_info = {
            "initialized": self.is_initialized,
            "intent_classifier": {
                "available": self.intent_classifier is not None and self.intent_classifier.is_initialized,
                "model_dir": settings.RASA_MODEL_DIR if self.intent_classifier else None,
            },
            "mission_graph": {
                "available": self.mission_graph is not None and self.mission_graph.is_initialized,
            },
            "performance": {
                "voice_command_processing": {
                    "avg_latency_ms": np.mean(self.performance_metrics["voice_command_processing"]["latency_ms"]).item() if self.performance_metrics["voice_command_processing"]["latency_ms"] else None,
                    "success_rate": np.mean(self.performance_metrics["voice_command_processing"]["success_rate"]).item() if self.performance_metrics["voice_command_processing"]["success_rate"] else None,
                },
                "gesture_processing": {
                    "avg_latency_ms": np.mean(self.performance_metrics["gesture_processing"]["latency_ms"]).item() if self.performance_metrics["gesture_processing"]["latency_ms"] else None,
                    "success_rate": np.mean(self.performance_metrics["gesture_processing"]["success_rate"]).item() if self.performance_metrics["gesture_processing"]["success_rate"] else None,
                },
            },
            "command_history_count": len(self.command_history),
        }
        
        return status_info
