"""
Main application for the Anti-Jamming Service.

This module provides the main application entry point for the Anti-Jamming Service.
"""

import asyncio
import logging
import os
import sys
import uvicorn
import ssl
from typing import Dict, Any, Optional
from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
import pika
import json

from anti_jamming_service import __version__
from anti_jamming_service.api.routes import router, RateLimitingMiddleware
from anti_jamming_service.hardware.kraken_sdr import KrakenSDR
from anti_jamming_service.hardware.hackrf import HackRF
from anti_jamming_service.hardware.lora_sx127x import LoRaSX127x
from anti_jamming_service.processing.gnss_mitigation import GNSSMitigationProcessor
from anti_jamming_service.processing.doa_estimation import DoAEstimator
from anti_jamming_service.processing.jamming_detection import JammingDetector
from anti_jamming_service.processing.fhss import FHSSProtocol
from anti_jamming_service.utils.config import load_config, get_config
from anti_jamming_service.utils.logging import setup_logging
from anti_jamming_service.utils.vault import get_vault_client

logger = logging.getLogger(__name__)


class AntiJammingService:
    """
    Anti-Jamming Service.
    
    This class provides the main application for the Anti-Jamming Service.
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the Anti-Jamming Service.
        
        Args:
            config_path: Path to configuration file.
        """
        # Load configuration
        self.config = load_config(config_path)
        
        # Set up logging
        setup_logging()
        
        # Initialize FastAPI app
        self.app = FastAPI(
            title="Anti-Jamming Service",
            description="API for the Anti-Jamming Service",
            version=__version__,
            docs_url="/docs",
            redoc_url="/redoc"
        )
        
        # Add middleware
        self._setup_middleware()
        
        # Add exception handlers
        self._setup_exception_handlers()
        
        # Add routes
        self.app.include_router(router, prefix="/api")
        
        # Initialize hardware and processors
        self.hardware = {}
        self.processors = {}
        
        # Initialize RabbitMQ connection
        self.rabbitmq_connection = None
        self.rabbitmq_channel = None
    
    def _setup_middleware(self):
        """Set up middleware."""
        # Add CORS middleware
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Add rate limiting middleware
        rate_limit = self.config.get("security", {}).get("rate_limiting", {}).get("default_limit", "60/minute")
        self.app.add_middleware(RateLimitingMiddleware, rate_limit=rate_limit)
        
        # Add startup and shutdown event handlers
        self.app.add_event_handler("startup", self.startup_event)
        self.app.add_event_handler("shutdown", self.shutdown_event)
    
    def _setup_exception_handlers(self):
        """Set up exception handlers."""
        # Add exception handlers
        @self.app.exception_handler(StarletteHTTPException)
        async def http_exception_handler(request, exc):
            return JSONResponse(
                status_code=exc.status_code,
                content={"error": str(exc.detail)}
            )
        
        @self.app.exception_handler(RequestValidationError)
        async def validation_exception_handler(request, exc):
            return JSONResponse(
                status_code=422,
                content={"error": "Validation error", "detail": str(exc)}
            )
        
        @self.app.exception_handler(Exception)
        async def general_exception_handler(request, exc):
            logger.exception("Unhandled exception")
            return JSONResponse(
                status_code=500,
                content={"error": "Internal server error", "detail": str(exc)}
            )
    
    async def startup_event(self):
        """Startup event handler."""
        logger.info("Starting Anti-Jamming Service")
        
        # Initialize hardware
        await self._initialize_hardware()
        
        # Initialize processors
        await self._initialize_processors()
        
        # Initialize RabbitMQ
        await self._initialize_rabbitmq()
        
        # Store hardware and processors in app state
        self.app.state.hardware = self.hardware
        self.app.state.processors = self.processors
        
        logger.info("Anti-Jamming Service started")
    
    async def shutdown_event(self):
        """Shutdown event handler."""
        logger.info("Shutting down Anti-Jamming Service")
        
        # Shutdown processors
        for processor in self.processors.values():
            if hasattr(processor, "shutdown"):
                await processor.shutdown()
        
        # Shutdown hardware
        for hardware in self.hardware.values():
            await hardware.shutdown()
        
        # Close RabbitMQ connection
        if self.rabbitmq_connection and self.rabbitmq_connection.is_open:
            self.rabbitmq_connection.close()
        
        logger.info("Anti-Jamming Service shut down")
    
    async def _initialize_hardware(self):
        """Initialize hardware components."""
        logger.info("Initializing hardware components")
        
        # Initialize KrakenSDR
        kraken_config = self.config.get("hardware", {}).get("kraken_sdr", {})
        if kraken_config.get("enabled", True):
            logger.info("Initializing KrakenSDR")
            kraken_sdr = KrakenSDR(
                device_index=kraken_config.get("device_index", 0),
                config=kraken_config
            )
            if await kraken_sdr.initialize():
                self.hardware["kraken_sdr"] = kraken_sdr
                logger.info("KrakenSDR initialized")
            else:
                logger.error("Failed to initialize KrakenSDR")
        
        # Initialize HackRF
        hackrf_config = self.config.get("hardware", {}).get("hackrf", {})
        if hackrf_config.get("enabled", True):
            logger.info("Initializing HackRF")
            hackrf = HackRF(
                serial_number=hackrf_config.get("serial_number"),
                config=hackrf_config
            )
            if await hackrf.initialize():
                self.hardware["hackrf"] = hackrf
                logger.info("HackRF initialized")
            else:
                logger.error("Failed to initialize HackRF")
        
        # Initialize LoRa SX127x
        lora_config = self.config.get("hardware", {}).get("lora_sx127x", {})
        if lora_config.get("enabled", True):
            logger.info("Initializing LoRa SX127x")
            lora = LoRaSX127x(
                port=lora_config.get("port", "/dev/ttyUSB0"),
                config=lora_config
            )
            if await lora.initialize():
                self.hardware["lora_sx127x"] = lora
                logger.info("LoRa SX127x initialized")
            else:
                logger.error("Failed to initialize LoRa SX127x")
    
    async def _initialize_processors(self):
        """Initialize signal processing components."""
        logger.info("Initializing signal processing components")
        
        # Initialize GNSS mitigation processor
        gnss_config = self.config.get("processing", {}).get("gnss_mitigation", {})
        if gnss_config.get("enabled", True) and "kraken_sdr" in self.hardware:
            logger.info("Initializing GNSS mitigation processor")
            gnss_processor = GNSSMitigationProcessor(config=gnss_config)
            self.processors["gnss_mitigation"] = gnss_processor
            logger.info("GNSS mitigation processor initialized")
        
        # Initialize DoA estimator
        doa_config = self.config.get("processing", {}).get("doa_estimation", {})
        if doa_config.get("enabled", True) and "kraken_sdr" in self.hardware:
            logger.info("Initializing DoA estimator")
            doa_estimator = DoAEstimator(
                sdr_interface=self.hardware["kraken_sdr"],
                config=doa_config
            )
            self.processors["doa_estimation"] = doa_estimator
            logger.info("DoA estimator initialized")
        
        # Initialize jamming detector
        jamming_config = self.config.get("processing", {}).get("jamming_detection", {})
        if jamming_config.get("enabled", True) and ("kraken_sdr" in self.hardware or "hackrf" in self.hardware):
            logger.info("Initializing jamming detector")
            # Use KrakenSDR if available, otherwise use HackRF
            sdr = self.hardware.get("kraken_sdr", self.hardware.get("hackrf"))
            jamming_detector = JammingDetector(
                sdr_interface=sdr,
                config=jamming_config
            )
            if await jamming_detector.initialize():
                self.processors["jamming_detection"] = jamming_detector
                logger.info("Jamming detector initialized")
            else:
                logger.error("Failed to initialize jamming detector")
        
        # Initialize FHSS protocol
        fhss_config = self.config.get("processing", {}).get("fhss", {})
        if fhss_config.get("enabled", True) and "lora_sx127x" in self.hardware:
            logger.info("Initializing FHSS protocol")
            fhss_protocol = FHSSProtocol(
                lora_interface=self.hardware["lora_sx127x"],
                config=fhss_config
            )
            if await fhss_protocol.initialize():
                self.processors["fhss"] = fhss_protocol
                logger.info("FHSS protocol initialized")
            else:
                logger.error("Failed to initialize FHSS protocol")
    
    async def _initialize_rabbitmq(self):
        """Initialize RabbitMQ connection."""
        rabbitmq_config = self.config.get("rabbitmq", {})
        if not rabbitmq_config.get("enabled", False):
            logger.info("RabbitMQ integration disabled")
            return
        
        try:
            # Connect to RabbitMQ
            logger.info("Connecting to RabbitMQ")
            
            # Get connection parameters
            host = rabbitmq_config.get("host", "localhost")
            port = rabbitmq_config.get("port", 5672)
            vhost = rabbitmq_config.get("vhost", "/")
            username = rabbitmq_config.get("username", "guest")
            password = rabbitmq_config.get("password", "guest")
            
            # Create connection
            credentials = pika.PlainCredentials(username, password)
            parameters = pika.ConnectionParameters(
                host=host,
                port=port,
                virtual_host=vhost,
                credentials=credentials
            )
            
            self.rabbitmq_connection = pika.BlockingConnection(parameters)
            self.rabbitmq_channel = self.rabbitmq_connection.channel()
            
            # Declare exchange
            exchange = rabbitmq_config.get("exchange", "anti_jamming")
            exchange_type = rabbitmq_config.get("exchange_type", "topic")
            self.rabbitmq_channel.exchange_declare(
                exchange=exchange,
                exchange_type=exchange_type,
                durable=True
            )
            
            logger.info(f"Connected to RabbitMQ at {host}:{port}")
            
            # Start jamming alert task
            if "jamming_detection" in self.processors:
                asyncio.create_task(self._jamming_alert_task())
        except Exception as e:
            logger.error(f"Failed to connect to RabbitMQ: {str(e)}")
    
    async def _jamming_alert_task(self):
        """Task for sending jamming alerts to RabbitMQ."""
        logger.info("Starting jamming alert task")
        
        rabbitmq_config = self.config.get("rabbitmq", {})
        exchange = rabbitmq_config.get("exchange", "anti_jamming")
        routing_key = rabbitmq_config.get("routing_key", "jamming.alert")
        
        jamming_detector = self.processors["jamming_detection"]
        last_alert_time = 0
        alert_interval = rabbitmq_config.get("alert_interval", 5.0)  # seconds
        
        while True:
            try:
                # Check if jamming is detected
                if jamming_detector.last_detection["is_jamming"]:
                    # Check if alert interval has passed
                    now = time.time()
                    if now - last_alert_time >= alert_interval:
                        # Send alert to RabbitMQ
                        alert = {
                            "is_jamming": True,
                            "jamming_type": jamming_detector.last_detection["jamming_type"].name,
                            "confidence": jamming_detector.last_detection["confidence"],
                            "snr_db": jamming_detector.last_detection["snr_db"],
                            "timestamp": jamming_detector.last_detection["timestamp"]
                        }
                        
                        # Add DoA information if available
                        if "doa_estimation" in self.processors:
                            doa = await self.processors["doa_estimation"].estimate()
                            alert["doa"] = {
                                "azimuth": doa["azimuth"],
                                "elevation": doa["elevation"],
                                "confidence": doa["confidence"]
                            }
                        
                        # Send alert
                        self.rabbitmq_channel.basic_publish(
                            exchange=exchange,
                            routing_key=routing_key,
                            body=json.dumps(alert),
                            properties=pika.BasicProperties(
                                delivery_mode=2,  # make message persistent
                                content_type="application/json"
                            )
                        )
                        
                        logger.info(f"Sent jamming alert: {alert}")
                        last_alert_time = now
            except Exception as e:
                logger.error(f"Error in jamming alert task: {str(e)}")
            
            # Sleep for a short time
            await asyncio.sleep(1.0)
    
    def run(self):
        """Run the Anti-Jamming Service."""
        # Get API configuration
        api_config = self.config.get("api", {})
        host = api_config.get("host", "0.0.0.0")
        port = api_config.get("port", 8080)
        
        # Check if TLS is enabled
        tls_config = self.config.get("security", {}).get("tls", {})
        ssl_enabled = tls_config.get("enabled", False)
        
        if ssl_enabled:
            # Get TLS configuration
            cert_file = tls_config.get("cert_file")
            key_file = tls_config.get("key_file")
            
            if not cert_file or not key_file:
                logger.error("TLS enabled but cert_file or key_file not provided")
                sys.exit(1)
            
            # Check if files exist
            if not os.path.exists(cert_file):
                logger.error(f"TLS certificate file not found: {cert_file}")
                sys.exit(1)
            
            if not os.path.exists(key_file):
                logger.error(f"TLS key file not found: {key_file}")
                sys.exit(1)
            
            # Create SSL context
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(cert_file, key_file)
            
            # Check if client verification is enabled
            if tls_config.get("verify_client", False):
                ssl_context.verify_mode = ssl.CERT_REQUIRED
                
                # Load CA certificate if provided
                ca_file = tls_config.get("ca_file")
                if ca_file:
                    if not os.path.exists(ca_file):
                        logger.error(f"CA certificate file not found: {ca_file}")
                        sys.exit(1)
                    
                    ssl_context.load_verify_locations(ca_file)
            
            # Run with TLS
            uvicorn.run(
                self.app,
                host=host,
                port=port,
                ssl_keyfile=key_file,
                ssl_certfile=cert_file,
                ssl_ca_certs=tls_config.get("ca_file"),
                ssl_cert_reqs=ssl.CERT_REQUIRED if tls_config.get("verify_client", False) else ssl.CERT_NONE
            )
        else:
            # Run without TLS
            uvicorn.run(self.app, host=host, port=port)


def main():
    """Main entry point."""
    # Get configuration path from command line
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    
    # Create and run service
    service = AntiJammingService(config_path)
    service.run()


if __name__ == "__main__":
    main()
