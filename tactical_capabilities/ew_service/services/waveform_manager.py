"""
Waveform manager service for the EW service.

This service is responsible for managing waveform templates, including:
- Creating and storing waveform templates
- Retrieving waveform templates
- Generating waveforms from templates
- Validating waveforms
"""

import logging
import uuid
import os
import json
import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime

from services.waveform_generator import WaveformGenerator
from services.storage_manager import StorageManager
from core.config import settings

logger = logging.getLogger(__name__)

class WaveformManager:
    """Waveform manager service."""
    
    def __init__(
        self,
        waveform_generator: Optional[WaveformGenerator] = None,
        storage_manager: Optional[StorageManager] = None
    ):
        """
        Initialize the waveform manager.
        
        Args:
            waveform_generator: Waveform generator service
            storage_manager: Storage manager service
        """
        self.waveform_generator = waveform_generator or WaveformGenerator()
        self.storage_manager = storage_manager
        self.waveforms = {}  # In-memory cache of waveform templates
    
    async def create_waveform_template(
        self,
        name: str,
        description: str,
        waveform_type: str,
        parameters: Dict[str, Any],
        preview: bool = True,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Create a new waveform template.
        
        Args:
            name: Waveform template name
            description: Waveform template description
            waveform_type: Type of waveform (noise, tone, sweep, pulse, chirp, custom)
            parameters: Waveform parameters
            preview: Whether to generate a preview
            metadata: Optional metadata
            
        Returns:
            Created waveform template
        """
        # Validate parameters
        self._validate_waveform_parameters(waveform_type, parameters)
        
        # Create waveform ID
        waveform_id = str(uuid.uuid4())
        
        # Create waveform template
        now = datetime.utcnow()
        waveform_template = {
            "id": waveform_id,
            "name": name,
            "description": description,
            "waveform_type": waveform_type,
            "parameters": parameters,
            "file_url": None,
            "preview_url": None,
            "metadata": metadata or {},
            "created_at": now.isoformat(),
            "updated_at": now.isoformat()
        }
        
        # Generate waveform and preview if requested
        if preview:
            try:
                # Generate waveform
                samples, waveform_metadata = self._generate_waveform(waveform_type, parameters)
                
                # Store waveform if storage manager is available
                if self.storage_manager:
                    # Save waveform data
                    waveform_path = f"waveforms/{waveform_id}.npz"
                    with open(f"/tmp/{waveform_id}.npz", "wb") as f:
                        np.savez(f, samples=samples, **waveform_metadata)
                    
                    # Upload to storage
                    waveform_url = await self.storage_manager.upload_file(
                        f"/tmp/{waveform_id}.npz",
                        waveform_path,
                        "application/octet-stream"
                    )
                    
                    # Update template with file URL
                    waveform_template["file_url"] = waveform_url
                    
                    # Clean up temporary file
                    os.remove(f"/tmp/{waveform_id}.npz")
                    
                    # Generate preview
                    preview_path = f"waveforms/{waveform_id}_preview.png"
                    self._generate_waveform_preview(samples, f"/tmp/{waveform_id}_preview.png")
                    
                    # Upload preview to storage
                    preview_url = await self.storage_manager.upload_file(
                        f"/tmp/{waveform_id}_preview.png",
                        preview_path,
                        "image/png"
                    )
                    
                    # Update template with preview URL
                    waveform_template["preview_url"] = preview_url
                    
                    # Clean up temporary file
                    os.remove(f"/tmp/{waveform_id}_preview.png")
            
            except Exception as e:
                logger.error(f"Failed to generate waveform preview: {e}")
        
        # Store waveform template
        self.waveforms[waveform_id] = waveform_template
        
        # Store waveform template in storage if available
        if self.storage_manager:
            try:
                # Save template data
                template_path = f"waveforms/{waveform_id}_template.json"
                with open(f"/tmp/{waveform_id}_template.json", "w") as f:
                    json.dump(waveform_template, f)
                
                # Upload to storage
                await self.storage_manager.upload_file(
                    f"/tmp/{waveform_id}_template.json",
                    template_path,
                    "application/json"
                )
                
                # Clean up temporary file
                os.remove(f"/tmp/{waveform_id}_template.json")
            
            except Exception as e:
                logger.error(f"Failed to store waveform template: {e}")
        
        return waveform_template
    
    async def get_waveform_template(self, waveform_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a waveform template by ID.
        
        Args:
            waveform_id: Waveform template ID
            
        Returns:
            Waveform template or None if not found
        """
        # Check if waveform is in cache
        if waveform_id in self.waveforms:
            return self.waveforms[waveform_id]
        
        # Try to load from storage if available
        if self.storage_manager:
            try:
                # Download template
                template_path = f"waveforms/{waveform_id}_template.json"
                local_path = f"/tmp/{waveform_id}_template.json"
                
                if await self.storage_manager.download_file(template_path, local_path):
                    # Load template
                    with open(local_path, "r") as f:
                        waveform_template = json.load(f)
                    
                    # Add to cache
                    self.waveforms[waveform_id] = waveform_template
                    
                    # Clean up temporary file
                    os.remove(local_path)
                    
                    return waveform_template
            
            except Exception as e:
                logger.error(f"Failed to load waveform template: {e}")
        
        return None
    
    async def get_all_waveform_templates(self) -> List[Dict[str, Any]]:
        """
        Get all waveform templates.
        
        Returns:
            List of waveform templates
        """
        # Return cached waveforms
        return list(self.waveforms.values())
    
    async def update_waveform_template(
        self,
        waveform_id: str,
        update_data: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """
        Update a waveform template.
        
        Args:
            waveform_id: Waveform template ID
            update_data: Update data
            
        Returns:
            Updated waveform template or None if not found
        """
        # Get waveform template
        waveform_template = await self.get_waveform_template(waveform_id)
        
        if not waveform_template:
            return None
        
        # Update fields
        for key, value in update_data.items():
            if key in ["id", "created_at", "file_url", "preview_url"]:
                continue
            waveform_template[key] = value
        
        # Update timestamp
        waveform_template["updated_at"] = datetime.utcnow().isoformat()
        
        # Update cache
        self.waveforms[waveform_id] = waveform_template
        
        # Update in storage if available
        if self.storage_manager:
            try:
                # Save template data
                template_path = f"waveforms/{waveform_id}_template.json"
                with open(f"/tmp/{waveform_id}_template.json", "w") as f:
                    json.dump(waveform_template, f)
                
                # Upload to storage
                await self.storage_manager.upload_file(
                    f"/tmp/{waveform_id}_template.json",
                    template_path,
                    "application/json"
                )
                
                # Clean up temporary file
                os.remove(f"/tmp/{waveform_id}_template.json")
            
            except Exception as e:
                logger.error(f"Failed to update waveform template in storage: {e}")
        
        return waveform_template
    
    async def delete_waveform_template(self, waveform_id: str) -> bool:
        """
        Delete a waveform template.
        
        Args:
            waveform_id: Waveform template ID
            
        Returns:
            True if successful, False otherwise
        """
        # Check if waveform exists
        if waveform_id not in self.waveforms:
            return False
        
        # Get waveform template
        waveform_template = self.waveforms[waveform_id]
        
        # Remove from cache
        del self.waveforms[waveform_id]
        
        # Remove from storage if available
        if self.storage_manager:
            try:
                # Delete template
                template_path = f"waveforms/{waveform_id}_template.json"
                await self.storage_manager.delete_file(template_path)
                
                # Delete waveform file if exists
                if waveform_template.get("file_url"):
                    waveform_path = f"waveforms/{waveform_id}.npz"
                    await self.storage_manager.delete_file(waveform_path)
                
                # Delete preview if exists
                if waveform_template.get("preview_url"):
                    preview_path = f"waveforms/{waveform_id}_preview.png"
                    await self.storage_manager.delete_file(preview_path)
            
            except Exception as e:
                logger.error(f"Failed to delete waveform template from storage: {e}")
        
        return True
    
    async def generate_waveform(
        self,
        waveform_id: str,
        duration: float = 1.0,
        sample_rate: int = 44100,
        parameters_override: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a waveform from a template.
        
        Args:
            waveform_id: Waveform template ID
            duration: Duration in seconds
            sample_rate: Sample rate in Hz
            parameters_override: Optional parameters to override
            
        Returns:
            Tuple of (waveform samples, metadata)
            
        Raises:
            ValueError: If waveform template not found
        """
        # Get waveform template
        waveform_template = await self.get_waveform_template(waveform_id)
        
        if not waveform_template:
            raise ValueError(f"Waveform template {waveform_id} not found")
        
        # Merge parameters
        parameters = waveform_template["parameters"].copy()
        if parameters_override:
            parameters.update(parameters_override)
        
        # Override duration and sample rate
        parameters["duration"] = duration
        parameters["sample_rate"] = sample_rate
        
        # Generate waveform
        samples, metadata = self._generate_waveform(
            waveform_template["waveform_type"],
            parameters
        )
        
        return samples, metadata
    
    def _generate_waveform(
        self,
        waveform_type: str,
        parameters: Dict[str, Any]
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Generate a waveform.
        
        Args:
            waveform_type: Type of waveform
            parameters: Waveform parameters
            
        Returns:
            Tuple of (waveform samples, metadata)
            
        Raises:
            ValueError: If waveform type is unknown
        """
        # Extract common parameters
        sample_rate = parameters.get("sample_rate", 44100)
        duration = parameters.get("duration", 1.0)
        
        # Generate waveform based on type
        if waveform_type == "noise":
            noise_type = parameters.get("noise_type", "white")
            amplitude = parameters.get("amplitude", 1.0)
            seed = parameters.get("seed")
            
            return self.waveform_generator.generate_noise_waveform(
                waveform_type=noise_type,
                sample_rate=sample_rate,
                duration=duration,
                amplitude=amplitude,
                seed=seed
            )
        
        elif waveform_type == "tone":
            frequencies = parameters.get("frequencies", [1000.0])
            amplitudes = parameters.get("amplitudes")
            phases = parameters.get("phases")
            
            return self.waveform_generator.generate_tone_waveform(
                frequencies=frequencies,
                amplitudes=amplitudes,
                phases=phases,
                sample_rate=sample_rate,
                duration=duration
            )
        
        elif waveform_type == "sweep":
            start_frequency = parameters.get("start_frequency", 100.0)
            end_frequency = parameters.get("end_frequency", 10000.0)
            sweep_type = parameters.get("sweep_type", "linear")
            amplitude = parameters.get("amplitude", 1.0)
            
            return self.waveform_generator.generate_sweep_waveform(
                start_frequency=start_frequency,
                end_frequency=end_frequency,
                sweep_type=sweep_type,
                sample_rate=sample_rate,
                duration=duration,
                amplitude=amplitude
            )
        
        elif waveform_type == "pulse":
            pulse_width = parameters.get("pulse_width", 0.001)
            pulse_repetition_interval = parameters.get("pulse_repetition_interval", 0.01)
            frequency = parameters.get("frequency", 0.0)
            amplitude = parameters.get("amplitude", 1.0)
            pulse_shape = parameters.get("pulse_shape", "rectangular")
            
            return self.waveform_generator.generate_pulse_waveform(
                pulse_width=pulse_width,
                pulse_repetition_interval=pulse_repetition_interval,
                frequency=frequency,
                sample_rate=sample_rate,
                duration=duration,
                amplitude=amplitude,
                pulse_shape=pulse_shape
            )
        
        elif waveform_type == "chirp":
            start_frequency = parameters.get("start_frequency", 100.0)
            end_frequency = parameters.get("end_frequency", 10000.0)
            chirp_period = parameters.get("chirp_period", 0.1)
            amplitude = parameters.get("amplitude", 1.0)
            
            return self.waveform_generator.generate_chirp_waveform(
                start_frequency=start_frequency,
                end_frequency=end_frequency,
                chirp_period=chirp_period,
                sample_rate=sample_rate,
                duration=duration,
                amplitude=amplitude
            )
        
        elif waveform_type == "combined":
            # Combined waveform requires pre-generated waveforms
            raise ValueError("Combined waveform generation not supported directly")
        
        else:
            raise ValueError(f"Unknown waveform type: {waveform_type}")
    
    def _generate_waveform_preview(self, samples: np.ndarray, file_path: str):
        """
        Generate a preview image for a waveform.
        
        Args:
            samples: Waveform samples
            file_path: Path to save the preview
        """
        try:
            import matplotlib.pyplot as plt
            
            # Create figure
            plt.figure(figsize=(10, 4))
            
            # Plot waveform
            plt.plot(samples[:min(len(samples), 1000)])
            
            # Set labels
            plt.title("Waveform Preview")
            plt.xlabel("Sample")
            plt.ylabel("Amplitude")
            
            # Set limits
            plt.ylim(-1.1, 1.1)
            
            # Save figure
            plt.savefig(file_path, dpi=100, bbox_inches="tight")
            plt.close()
        
        except Exception as e:
            logger.error(f"Failed to generate waveform preview: {e}")
    
    def _validate_waveform_parameters(self, waveform_type: str, parameters: Dict[str, Any]):
        """
        Validate waveform parameters.
        
        Args:
            waveform_type: Type of waveform
            parameters: Waveform parameters
            
        Raises:
            ValueError: If parameters are invalid
        """
        # Validate common parameters
        if "sample_rate" in parameters:
            sample_rate = parameters["sample_rate"]
            if not isinstance(sample_rate, (int, float)) or sample_rate <= 0:
                raise ValueError("Sample rate must be a positive number")
        
        if "duration" in parameters:
            duration = parameters["duration"]
            if not isinstance(duration, (int, float)) or duration <= 0:
                raise ValueError("Duration must be a positive number")
        
        # Validate type-specific parameters
        if waveform_type == "noise":
            noise_type = parameters.get("noise_type", "white")
            if noise_type not in ["white", "pink", "brown", "blue"]:
                raise ValueError(f"Unknown noise type: {noise_type}")
        
        elif waveform_type == "tone":
            frequencies = parameters.get("frequencies", [1000.0])
            if not isinstance(frequencies, list):
                raise ValueError("Frequencies must be a list")
            
            for freq in frequencies:
                if not isinstance(freq, (int, float)) or freq < 0:
                    raise ValueError("Frequencies must be non-negative numbers")
            
            if "amplitudes" in parameters:
                amplitudes = parameters["amplitudes"]
                if not isinstance(amplitudes, list) or len(amplitudes) != len(frequencies):
                    raise ValueError("Amplitudes must be a list with the same length as frequencies")
                
                for amp in amplitudes:
                    if not isinstance(amp, (int, float)) or amp < 0:
                        raise ValueError("Amplitudes must be non-negative numbers")
            
            if "phases" in parameters:
                phases = parameters["phases"]
                if not isinstance(phases, list) or len(phases) != len(frequencies):
                    raise ValueError("Phases must be a list with the same length as frequencies")
                
                for phase in phases:
                    if not isinstance(phase, (int, float)):
                        raise ValueError("Phases must be numbers")
        
        elif waveform_type == "sweep":
            start_frequency = parameters.get("start_frequency", 100.0)
            end_frequency = parameters.get("end_frequency", 10000.0)
            sweep_type = parameters.get("sweep_type", "linear")
            
            if not isinstance(start_frequency, (int, float)) or start_frequency < 0:
                raise ValueError("Start frequency must be a non-negative number")
            
            if not isinstance(end_frequency, (int, float)) or end_frequency < 0:
                raise ValueError("End frequency must be a non-negative number")
            
            if sweep_type not in ["linear", "exponential", "logarithmic"]:
                raise ValueError(f"Unknown sweep type: {sweep_type}")
        
        elif waveform_type == "pulse":
            pulse_width = parameters.get("pulse_width", 0.001)
            pulse_repetition_interval = parameters.get("pulse_repetition_interval", 0.01)
            frequency = parameters.get("frequency", 0.0)
            pulse_shape = parameters.get("pulse_shape", "rectangular")
            
            if not isinstance(pulse_width, (int, float)) or pulse_width <= 0:
                raise ValueError("Pulse width must be a positive number")
            
            if not isinstance(pulse_repetition_interval, (int, float)) or pulse_repetition_interval <= 0:
                raise ValueError("Pulse repetition interval must be a positive number")
            
            if pulse_width >= pulse_repetition_interval:
                raise ValueError("Pulse width must be less than pulse repetition interval")
            
            if not isinstance(frequency, (int, float)) or frequency < 0:
                raise ValueError("Frequency must be a non-negative number")
            
            if pulse_shape not in ["rectangular", "gaussian", "triangular"]:
                raise ValueError(f"Unknown pulse shape: {pulse_shape}")
        
        elif waveform_type == "chirp":
            start_frequency = parameters.get("start_frequency", 100.0)
            end_frequency = parameters.get("end_frequency", 10000.0)
            chirp_period = parameters.get("chirp_period", 0.1)
            
            if not isinstance(start_frequency, (int, float)) or start_frequency < 0:
                raise ValueError("Start frequency must be a non-negative number")
            
            if not isinstance(end_frequency, (int, float)) or end_frequency < 0:
                raise ValueError("End frequency must be a non-negative number")
            
            if not isinstance(chirp_period, (int, float)) or chirp_period <= 0:
                raise ValueError("Chirp period must be a positive number")
        
        elif waveform_type not in ["combined", "custom"]:
            raise ValueError(f"Unknown waveform type: {waveform_type}")
