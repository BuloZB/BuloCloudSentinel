"""
Configuration utilities for the Anti-Jamming Service.

This module provides utilities for loading and validating configuration.
"""

import logging
import os
import json
import yaml
from typing import Dict, Any, Optional
from pathlib import Path

logger = logging.getLogger(__name__)

# Default configuration
DEFAULT_CONFIG = {
    "api": {
        "host": "0.0.0.0",
        "port": 8080,
        "ssl_cert": None,
        "ssl_key": None,
        "rate_limit": "60/minute"
    },
    "hardware": {
        "kraken_sdr": {
            "device_index": 0,
            "center_frequency": 1575420000,  # GPS L1 frequency (1575.42 MHz)
            "sample_rate": 2400000,  # 2.4 MSPS
            "gain": 30.0,  # dB
            "coherent_mode": True,
            "reference_channel": 0
        },
        "hackrf": {
            "serial_number": None,
            "center_frequency": 2450000000,  # 2.45 GHz
            "tx_frequency": 2450000000,  # 2.45 GHz
            "sample_rate": 10000000,  # 10 MSPS
            "rx_gain": 30.0,  # dB
            "tx_gain": 0.0  # dB
        },
        "lora_sx127x": {
            "port": "/dev/ttyUSB0",
            "frequency": 868000000,  # 868 MHz
            "spreading_factor": 7,  # SF7
            "bandwidth": 125000,  # 125 kHz
            "coding_rate": 5,  # 4/5
            "frequency_hopping": False,
            "hop_period": 0,
            "tx_power": 17  # dBm
        }
    },
    "processing": {
        "gnss_mitigation": {
            "enabled": True,
            "pulse_blanking": {
                "threshold": 3.0,
                "window_size": 1024
            },
            "notch_filter": {
                "threshold": 10.0,
                "fft_size": 1024,
                "overlap": 512
            },
            "adaptive_filter_bank": {
                "num_filters": 4,
                "filter_length": 32,
                "step_size": 0.01
            }
        },
        "doa_estimation": {
            "enabled": True,
            "num_sources": 1,
            "angle_resolution": 1.0,
            "sample_size": 1024,
            "music": {
                "enabled": True
            },
            "esprit": {
                "enabled": False
            }
        },
        "jamming_detection": {
            "enabled": True,
            "sample_size": 1024,
            "detection_interval": 1.0,
            "energy_detector": {
                "enabled": True,
                "threshold": 10.0,
                "window_size": 1024
            },
            "spectral_detector": {
                "enabled": True,
                "threshold": 10.0,
                "fft_size": 1024,
                "overlap": 512
            },
            "cyclostationary_detector": {
                "enabled": False,
                "threshold": 0.8,
                "window_size": 1024,
                "num_lags": 128
            }
        },
        "fhss": {
            "enabled": True,
            "num_channels": 10,
            "hop_interval": 1.0,
            "base_frequency": 868000000,
            "channel_spacing": 200000,
            "seed": None
        }
    },
    "security": {
        "vault": {
            "enabled": False,
            "url": "http://127.0.0.1:8200",
            "token": None,
            "path": "secret/anti_jamming_service"
        },
        "tls": {
            "enabled": True,
            "cert_file": "certs/server.crt",
            "key_file": "certs/server.key",
            "ca_file": None,
            "verify_client": False
        },
        "rate_limiting": {
            "enabled": True,
            "default_limit": "60/minute",
            "auth_limit": "10/minute",
            "whitelist_ips": [],
            "blacklist_ips": []
        }
    },
    "logging": {
        "level": "INFO",
        "file": "logs/anti_jamming_service.log",
        "max_size": 10485760,  # 10 MB
        "backup_count": 5,
        "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    }
}

# Global configuration
_config = None


def load_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load configuration from file.
    
    Args:
        config_path: Path to configuration file. If None, uses environment variable
                    ANTI_JAMMING_CONFIG or default path.
    
    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    global _config
    
    if _config is not None:
        return _config
    
    # Get config path from environment variable or use default
    if config_path is None:
        config_path = os.environ.get("ANTI_JAMMING_CONFIG", "config/config.yaml")
    
    config = DEFAULT_CONFIG.copy()
    
    # Load configuration from file if it exists
    config_file = Path(config_path)
    if config_file.exists():
        try:
            if config_file.suffix.lower() in [".yaml", ".yml"]:
                with open(config_file, "r") as f:
                    file_config = yaml.safe_load(f)
            elif config_file.suffix.lower() == ".json":
                with open(config_file, "r") as f:
                    file_config = json.load(f)
            else:
                logger.warning(f"Unsupported config file format: {config_file.suffix}")
                file_config = {}
            
            # Merge file config with default config
            _merge_configs(config, file_config)
            
            logger.info(f"Loaded configuration from {config_file}")
        except Exception as e:
            logger.error(f"Failed to load configuration from {config_file}: {str(e)}")
    else:
        logger.warning(f"Configuration file not found: {config_file}")
    
    # Override with environment variables
    _override_from_env(config)
    
    # Validate configuration
    _validate_config(config)
    
    _config = config
    return config


def get_config() -> Dict[str, Any]:
    """
    Get the current configuration.
    
    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    global _config
    
    if _config is None:
        return load_config()
    
    return _config


def _merge_configs(base_config: Dict[str, Any], override_config: Dict[str, Any]) -> None:
    """
    Merge override configuration into base configuration.
    
    Args:
        base_config: Base configuration to update.
        override_config: Override configuration to merge.
    """
    for key, value in override_config.items():
        if key in base_config and isinstance(base_config[key], dict) and isinstance(value, dict):
            _merge_configs(base_config[key], value)
        else:
            base_config[key] = value


def _override_from_env(config: Dict[str, Any], prefix: str = "ANTI_JAMMING") -> None:
    """
    Override configuration with environment variables.
    
    Args:
        config: Configuration to update.
        prefix: Environment variable prefix.
    """
    for key, value in os.environ.items():
        if key.startswith(f"{prefix}_"):
            # Remove prefix and split by underscore
            parts = key[len(prefix) + 1:].lower().split("_")
            
            # Navigate to the correct config section
            current = config
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
            
            # Set the value
            try:
                # Try to parse as JSON
                current[parts[-1]] = json.loads(value)
            except json.JSONDecodeError:
                # If not valid JSON, use as string
                current[parts[-1]] = value


def _validate_config(config: Dict[str, Any]) -> None:
    """
    Validate configuration.
    
    Args:
        config: Configuration to validate.
    
    Raises:
        ValueError: If configuration is invalid.
    """
    # Validate API configuration
    api_config = config.get("api", {})
    if not isinstance(api_config.get("port"), int) or api_config.get("port") < 1 or api_config.get("port") > 65535:
        raise ValueError("Invalid API port")
    
    # Validate hardware configuration
    hardware_config = config.get("hardware", {})
    
    # KrakenSDR
    kraken_config = hardware_config.get("kraken_sdr", {})
    if not isinstance(kraken_config.get("center_frequency"), int) or kraken_config.get("center_frequency") < 0:
        raise ValueError("Invalid KrakenSDR center frequency")
    
    # HackRF
    hackrf_config = hardware_config.get("hackrf", {})
    if not isinstance(hackrf_config.get("center_frequency"), int) or hackrf_config.get("center_frequency") < 0:
        raise ValueError("Invalid HackRF center frequency")
    
    # LoRa SX127x
    lora_config = hardware_config.get("lora_sx127x", {})
    if not isinstance(lora_config.get("frequency"), int) or lora_config.get("frequency") < 0:
        raise ValueError("Invalid LoRa frequency")
    
    # Validate processing configuration
    processing_config = config.get("processing", {})
    
    # GNSS mitigation
    gnss_config = processing_config.get("gnss_mitigation", {})
    if not isinstance(gnss_config.get("enabled"), bool):
        raise ValueError("Invalid GNSS mitigation enabled flag")
    
    # DoA estimation
    doa_config = processing_config.get("doa_estimation", {})
    if not isinstance(doa_config.get("enabled"), bool):
        raise ValueError("Invalid DoA estimation enabled flag")
    
    # Jamming detection
    jamming_config = processing_config.get("jamming_detection", {})
    if not isinstance(jamming_config.get("enabled"), bool):
        raise ValueError("Invalid jamming detection enabled flag")
    
    # FHSS
    fhss_config = processing_config.get("fhss", {})
    if not isinstance(fhss_config.get("enabled"), bool):
        raise ValueError("Invalid FHSS enabled flag")
    
    # Validate security configuration
    security_config = config.get("security", {})
    
    # Vault
    vault_config = security_config.get("vault", {})
    if not isinstance(vault_config.get("enabled"), bool):
        raise ValueError("Invalid Vault enabled flag")
    
    # TLS
    tls_config = security_config.get("tls", {})
    if not isinstance(tls_config.get("enabled"), bool):
        raise ValueError("Invalid TLS enabled flag")
    
    # Rate limiting
    rate_config = security_config.get("rate_limiting", {})
    if not isinstance(rate_config.get("enabled"), bool):
        raise ValueError("Invalid rate limiting enabled flag")
