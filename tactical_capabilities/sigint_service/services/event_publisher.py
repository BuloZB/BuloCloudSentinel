"""
Event publisher service for the SIGINT service.

This service is responsible for publishing events to the message bus.
"""

import json
import logging
import pika
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

class EventPublisher:
    """Event publisher service."""
    
    def __init__(self, host: str, port: int, username: str, password: str, exchange: str):
        """
        Initialize the event publisher.
        
        Args:
            host: RabbitMQ host
            port: RabbitMQ port
            username: RabbitMQ username
            password: RabbitMQ password
            exchange: RabbitMQ exchange
        """
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.exchange = exchange
        self.connection = None
        self.channel = None
    
    async def connect(self):
        """Connect to the message bus."""
        logger.info(f"Connecting to RabbitMQ at {self.host}:{self.port}")
        
        try:
            # Create connection parameters
            credentials = pika.PlainCredentials(self.username, self.password)
            parameters = pika.ConnectionParameters(
                host=self.host,
                port=self.port,
                credentials=credentials,
                heartbeat=600,
                blocked_connection_timeout=300
            )
            
            # Connect to RabbitMQ
            self.connection = pika.BlockingConnection(parameters)
            self.channel = self.connection.channel()
            
            # Declare exchange
            self.channel.exchange_declare(
                exchange=self.exchange,
                exchange_type='topic',
                durable=True
            )
            
            logger.info("Connected to RabbitMQ")
        except Exception as e:
            logger.error(f"Failed to connect to RabbitMQ: {e}")
            # In a real implementation, we would retry with backoff
            raise
    
    async def disconnect(self):
        """Disconnect from the message bus."""
        logger.info("Disconnecting from RabbitMQ")
        
        if self.connection and self.connection.is_open:
            self.connection.close()
            logger.info("Disconnected from RabbitMQ")
    
    async def publish_event(self, routing_key: str, event: Dict[str, Any], correlation_id: Optional[str] = None):
        """
        Publish an event to the message bus.
        
        Args:
            routing_key: Routing key for the event
            event: Event data
            correlation_id: Optional correlation ID for the event
        """
        if not self.channel or not self.connection or self.connection.is_closed:
            logger.warning("Not connected to RabbitMQ, reconnecting")
            await self.connect()
        
        try:
            # Convert event to JSON
            event_json = json.dumps(event)
            
            # Create message properties
            properties = pika.BasicProperties(
                content_type='application/json',
                delivery_mode=2,  # Persistent
                correlation_id=correlation_id
            )
            
            # Publish message
            self.channel.basic_publish(
                exchange=self.exchange,
                routing_key=routing_key,
                body=event_json,
                properties=properties
            )
            
            logger.debug(f"Published event to {self.exchange}.{routing_key}: {event}")
        except Exception as e:
            logger.error(f"Failed to publish event: {e}")
            # In a real implementation, we would retry with backoff
            raise
    
    async def publish_detection_event(self, detection: Dict[str, Any], event_type: str = "detection.created"):
        """
        Publish a detection event.
        
        Args:
            detection: Detection data
            event_type: Event type (created, updated)
        """
        # Create routing key
        signal_type = detection.get("signal_type", "unknown").lower()
        routing_key = f"detection.{signal_type}.{event_type}"
        
        # Create event
        event = {
            "event_type": event_type,
            "detection": detection
        }
        
        # Publish event
        await self.publish_event(routing_key, event, correlation_id=str(detection.get("id")))
    
    async def publish_source_event(self, source: Dict[str, Any], event_type: str = "source.updated"):
        """
        Publish a source event.
        
        Args:
            source: Source data
            event_type: Event type (created, updated)
        """
        # Create routing key
        signal_type = source.get("signal_type", "unknown").lower()
        threat_level = source.get("threat_level", "none").lower()
        routing_key = f"source.{signal_type}.{threat_level}.{event_type}"
        
        # Create event
        event = {
            "event_type": event_type,
            "source": source
        }
        
        # Publish event
        await self.publish_event(routing_key, event, correlation_id=str(source.get("id")))
    
    async def publish_alert_event(self, alert: Dict[str, Any]):
        """
        Publish an alert event.
        
        Args:
            alert: Alert data
        """
        # Create routing key
        alert_type = alert.get("type", "unknown").lower()
        severity = alert.get("severity", "info").lower()
        routing_key = f"alert.{alert_type}.{severity}"
        
        # Create event
        event = {
            "event_type": "alert.created",
            "alert": alert
        }
        
        # Publish event
        await self.publish_event(routing_key, event, correlation_id=str(alert.get("id")))
    
    async def publish_recording_event(self, recording: Dict[str, Any], event_type: str = "recording.created"):
        """
        Publish a recording event.
        
        Args:
            recording: Recording data
            event_type: Event type (created, deleted)
        """
        # Create routing key
        format_type = recording.get("format", "unknown").lower()
        routing_key = f"recording.{format_type}.{event_type}"
        
        # Create event
        event = {
            "event_type": event_type,
            "recording": recording
        }
        
        # Publish event
        await self.publish_event(routing_key, event, correlation_id=str(recording.get("id")))
    
    async def publish_intelligence_product(self, product: Dict[str, Any]):
        """
        Publish an intelligence product.
        
        Args:
            product: Intelligence product data
        """
        # Create routing key
        classification = product.get("classification", "unclassified").lower()
        routing_key = f"intelligence.product.{classification}.created"
        
        # Create event
        event = {
            "event_type": "intelligence.product.created",
            "product": product
        }
        
        # Publish event
        await self.publish_event(routing_key, event, correlation_id=str(product.get("id")))
