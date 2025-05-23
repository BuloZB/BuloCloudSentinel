"""
Rasa NLU intent classifier for Voice & Gesture Co-Pilot.

This module provides a wrapper for Rasa NLU for intent classification and entity extraction.
"""

import os
import time
import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from pathlib import Path

from loguru import logger

class IntentClassifier:
    """
    Wrapper for Rasa NLU intent classifier.
    
    This class provides methods for classifying intents and extracting entities
    from text using Rasa NLU.
    """
    
    def __init__(
        self,
        model_dir: str = "models/rasa",
        confidence_threshold: float = 0.7,
        voice_commands: Dict[str, Dict[str, Any]] = None,
    ):
        """
        Initialize the intent classifier.
        
        Args:
            model_dir: Directory containing Rasa NLU model
            confidence_threshold: Minimum confidence for intent classification
            voice_commands: Dictionary of voice command definitions
        """
        self.model_dir = model_dir
        self.confidence_threshold = confidence_threshold
        self.voice_commands = voice_commands or {}
        self.model = None
        self.is_initialized = False
        
        # Create models directory if it doesn't exist
        self.models_dir = Path(model_dir)
        self.models_dir.mkdir(parents=True, exist_ok=True)
    
    async def initialize(self):
        """
        Initialize the Rasa NLU model.
        
        This method loads the Rasa NLU model for intent classification.
        """
        logger.info("Initializing Rasa NLU model")
        
        try:
            # In a real implementation, you would load the Rasa NLU model here
            # For example:
            # from rasa.nlu.model import Interpreter
            # self.model = Interpreter.load(self.model_dir)
            
            # For now, we'll just simulate model loading
            logger.info(f"Loading Rasa NLU model from {self.model_dir}")
            await asyncio.sleep(1.0)  # Simulate model loading time
            
            # If model doesn't exist, create training data and train model
            if not list(self.models_dir.glob("*")):
                await self._create_training_data()
                await self._train_model()
            
            self.is_initialized = True
            logger.info("Rasa NLU model initialized")
            
            return True
        
        except Exception as e:
            logger.error(f"Error initializing Rasa NLU model: {str(e)}")
            return False
    
    async def cleanup(self):
        """
        Clean up resources used by the Rasa NLU model.
        """
        logger.info("Cleaning up Rasa NLU model resources")
        
        # In a real implementation, you would release model resources here
        
        self.is_initialized = False
        logger.info("Rasa NLU model resources cleaned up")
    
    async def _create_training_data(self):
        """
        Create training data for Rasa NLU.
        
        This method creates training data for Rasa NLU based on voice command definitions.
        """
        logger.info("Creating Rasa NLU training data")
        
        try:
            # Create training data directory
            training_data_dir = self.models_dir / "training_data"
            training_data_dir.mkdir(exist_ok=True)
            
            # Create training data file
            training_data_file = training_data_dir / "nlu.yml"
            
            # Generate training data from voice commands
            training_data = "version: '3.1'\n\nnlu:\n"
            
            for intent, command_config in self.voice_commands.items():
                training_data += f"- intent: {intent}\n  examples: |\n"
                for phrase in command_config.get("phrases", []):
                    training_data += f"    - {phrase}\n"
            
            # Write training data to file
            with open(training_data_file, "w") as f:
                f.write(training_data)
            
            logger.info(f"Rasa NLU training data created: {training_data_file}")
            
            return True
        
        except Exception as e:
            logger.error(f"Error creating Rasa NLU training data: {str(e)}")
            return False
    
    async def _train_model(self):
        """
        Train Rasa NLU model.
        
        This method trains a Rasa NLU model using the generated training data.
        """
        logger.info("Training Rasa NLU model")
        
        try:
            # In a real implementation, you would train the Rasa NLU model here
            # For example:
            # from rasa.nlu.train import train
            # from rasa.nlu.config import RasaNLUModelConfig
            # from rasa.nlu.components import ComponentBuilder
            # config = RasaNLUModelConfig({"language": "en", "pipeline": "supervised_embeddings"})
            # training_data_file = self.models_dir / "training_data" / "nlu.yml"
            # self.model = train(config, training_data_file, self.model_dir, ComponentBuilder())
            
            # For now, we'll just simulate model training
            logger.info("Training Rasa NLU model...")
            await asyncio.sleep(2.0)  # Simulate training time
            
            # Create a dummy model file to simulate the trained model
            model_file = self.models_dir / "model.pkl"
            with open(model_file, "wb") as f:
                f.write(b"SIMULATED_MODEL_DATA")
            
            logger.info("Rasa NLU model trained")
            
            return True
        
        except Exception as e:
            logger.error(f"Error training Rasa NLU model: {str(e)}")
            return False
    
    async def classify(self, text: str) -> Dict[str, Any]:
        """
        Classify intent and extract entities from text.
        
        Args:
            text: Text to classify
            
        Returns:
            Dictionary containing classification results
        """
        if not self.is_initialized:
            logger.error("Rasa NLU model not initialized")
            return {"error": "Rasa NLU model not initialized"}
        
        try:
            # Measure latency
            start_time = time.time()
            
            # In a real implementation, you would use the Rasa NLU model to classify intent
            # For example:
            # result = self.model.parse(text)
            
            # For now, we'll just simulate intent classification
            logger.info(f"Classifying intent: {text}")
            
            # Simple keyword-based intent classification
            intent = ""
            confidence = 0.0
            
            # Check each intent
            for intent_name, command_config in self.voice_commands.items():
                for phrase in command_config.get("phrases", []):
                    # Check if phrase is in text (case-insensitive)
                    if phrase.lower() in text.lower():
                        # Calculate confidence based on phrase length
                        phrase_confidence = len(phrase) / len(text) if len(text) > 0 else 0.0
                        
                        # Update intent if confidence is higher
                        if phrase_confidence > confidence:
                            intent = intent_name
                            confidence = phrase_confidence
            
            # Calculate latency
            latency_ms = (time.time() - start_time) * 1000
            
            return {
                "intent": intent,
                "confidence": confidence,
                "entities": [],  # In a real implementation, you would extract entities
                "latency_ms": latency_ms,
            }
        
        except Exception as e:
            logger.error(f"Error classifying intent: {str(e)}")
            return {"error": str(e)}
