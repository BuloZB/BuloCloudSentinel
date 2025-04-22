# SIGINT Service (Signal Intelligence)

## 1. Feature Name
Signal Intelligence (SIGINT) Service

## 2. Description & Benefits
The SIGINT Service provides advanced capabilities for collecting, processing, and analyzing signals intelligence data. It focuses on intercepting, identifying, and analyzing radio frequency (RF) signals, communications, and electronic emissions to support tactical decision-making and intelligence gathering.

### Key Features:
- **Signal Detection and Classification**: Identify and classify RF signals across various frequency bands
- **Communication Interception**: Capture and decode communication signals (COMINT)
- **Electronic Intelligence (ELINT)**: Analyze non-communication electronic signals
- **Direction Finding**: Determine the location of signal sources
- **Signal Analysis**: Extract metadata and content from intercepted signals
- **Threat Detection**: Identify potential threats based on signal characteristics
- **Secure API**: Role-based access control for all SIGINT capabilities
- **Event-driven Architecture**: Publish SIGINT events to the message bus for real-time updates

### Benefits:
- **Enhanced Situational Awareness**: Comprehensive understanding of the electromagnetic spectrum
- **Improved Threat Detection**: Early warning of potential threats
- **Tactical Advantage**: Knowledge of adversary communications and electronic systems
- **Intelligence Gathering**: Collection of valuable signals intelligence
- **Scalable Architecture**: Support for multiple signal collection platforms
- **Secure by Design**: Built with security best practices from the ground up

## 3. Integration with Existing Architecture

The SIGINT Service integrates with the following Bulo.Cloud Sentinel components:

- **Authentication and Authorization**: Uses the existing JWT-based authentication system and role-based access control
- **Database**: Stores SIGINT data in PostgreSQL using SQLAlchemy ORM
- **Message Bus**: Publishes SIGINT events to RabbitMQ for real-time updates
- **Object Storage**: Stores raw signal data and recordings in MinIO
- **Frontend**: Provides API endpoints for the React frontend to display SIGINT data

### Service Interactions:
- **ISR Service**: Shares signal source locations and correlates with visual intelligence
- **Electronic Warfare Service**: Provides signal data for jamming and countermeasures
- **Anti-Jamming Service**: Coordinates to identify and mitigate jamming threats
- **Drone Swarm System**: Coordinates signal collection platforms for optimal coverage
- **Mission Planning**: Receives signal collection tasks and mission parameters

### Message Flows:
1. Signal collectors publish raw signal data to the message bus
2. SIGINT Service processes signal data and publishes detections
3. SIGINT Service analyzes signals and publishes analysis results
4. SIGINT Service generates alerts for suspicious signals
5. SIGINT Service creates intelligence products from analyzed signals

## 4. API Endpoints

The SIGINT Service provides the following API endpoints:

### Signal Collectors
```
GET /api/v1/collectors - Get all signal collectors
POST /api/v1/collectors - Register a new signal collector
GET /api/v1/collectors/{collector_id} - Get a specific signal collector
PUT /api/v1/collectors/{collector_id} - Update a signal collector
DELETE /api/v1/collectors/{collector_id} - Unregister a signal collector
```

### Signal Detections
```
GET /api/v1/detections - Get all signal detections
POST /api/v1/detections - Create a new signal detection
GET /api/v1/detections/{detection_id} - Get a specific signal detection
```

### Signal Sources
```
GET /api/v1/sources - Get all signal sources
GET /api/v1/sources/{source_id} - Get a specific signal source
PUT /api/v1/sources/{source_id} - Update a signal source
```

### Signal Recordings
```
GET /api/v1/recordings - Get all signal recordings
POST /api/v1/recordings - Create a new signal recording
GET /api/v1/recordings/{recording_id} - Get a specific signal recording
DELETE /api/v1/recordings/{recording_id} - Delete a signal recording
```

### Signal Analysis
```
POST /api/v1/analysis/classify - Classify a signal
POST /api/v1/analysis/decode - Decode a signal
POST /api/v1/analysis/metadata - Extract metadata from a signal
POST /api/v1/analysis/direction - Perform direction finding
```

### SIGINT Alerts
```
GET /api/v1/alerts - Get all SIGINT alerts
POST /api/v1/alerts - Create a new SIGINT alert
GET /api/v1/alerts/{alert_id} - Get a specific SIGINT alert
PUT /api/v1/alerts/{alert_id} - Acknowledge an alert
```

### SIGINT Intelligence
```
POST /api/v1/intelligence/product - Generate a SIGINT intelligence product
GET /api/v1/intelligence/products - Get all SIGINT intelligence products
GET /api/v1/intelligence/products/{product_id} - Get a specific SIGINT intelligence product
```

## 5. Data Models

The SIGINT Service uses the following data models:

### SignalCollector
```python
class SignalCollector:
    id: UUID
    name: str
    type: CollectorType  # SDR, scanner, spectrum analyzer, etc.
    platform_id: Optional[UUID]
    status: str  # online, offline, error
    location: Optional[LocationSchema]
    orientation: Optional[OrientationSchema]
    frequency_range: FrequencyRangeSchema  # min_freq, max_freq in Hz
    capabilities: Optional[Dict[str, Any]]
    configuration: Optional[Dict[str, Any]]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### SignalDetection
```python
class SignalDetection:
    id: UUID
    collector_id: UUID
    timestamp: datetime
    frequency: float  # Hz
    bandwidth: float  # Hz
    signal_type: Optional[SignalType]  # AM, FM, GSM, etc.
    signal_strength: float  # dBm
    snr: float  # dB
    duration: float  # seconds
    location: Optional[LocationSchema]
    direction: Optional[DirectionSchema]  # azimuth, elevation
    recording_id: Optional[UUID]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
```

### SignalSource
```python
class SignalSource:
    id: UUID
    first_detected: datetime
    last_detected: datetime
    status: SourceStatus  # active, inactive, unknown
    signal_type: Optional[SignalType]
    frequency_range: FrequencyRangeSchema
    location: Optional[LocationSchema]
    location_accuracy: Optional[float]  # meters
    identification: Optional[str]
    threat_level: ThreatLevel  # none, low, medium, high, critical
    metadata: Optional[Dict[str, Any]]
    detections: List[SignalDetection]
    created_at: datetime
    updated_at: datetime
```

### SignalRecording
```python
class SignalRecording:
    id: UUID
    detection_id: UUID
    start_time: datetime
    duration: float  # seconds
    sample_rate: int  # Hz
    center_frequency: float  # Hz
    bandwidth: float  # Hz
    format: str  # IQ, WAV, etc.
    file_url: str
    file_size: int  # bytes
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
```

### SignalAnalysis
```python
class SignalAnalysis:
    id: UUID
    detection_id: UUID
    timestamp: datetime
    analysis_type: AnalysisType  # classification, decoding, metadata, direction
    confidence: float
    results: Dict[str, Any]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
```

### SigintAlert
```python
class SigintAlert:
    id: UUID
    type: AlertType  # new_signal, known_threat, jamming, etc.
    severity: AlertSeverity  # info, warning, critical
    message: str
    timestamp: datetime
    frequency: Optional[float]  # Hz
    location: Optional[LocationSchema]
    source_id: Optional[UUID]
    detection_id: Optional[UUID]
    acknowledged: bool
    acknowledged_by: Optional[str]
    acknowledged_at: Optional[datetime]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
```

### SigintIntelligenceProduct
```python
class SigintIntelligenceProduct:
    id: UUID
    title: str
    description: str
    timestamp: datetime
    classification: str  # unclassified, confidential, secret, etc.
    sources: List[UUID]  # List of source IDs
    detections: List[UUID]  # List of detection IDs
    analysis: Dict[str, Any]
    conclusions: List[str]
    recommendations: List[str]
    created_by: str
    created_at: datetime
```

## 6. Security Considerations

The SIGINT Service implements the following security measures:

### Authentication and Authorization
- **JWT-based authentication**: All API endpoints require a valid JWT token
- **Role-based access control**: Different permissions for different user roles
- **Permission-based access**: Fine-grained control over API endpoints
- **Classification levels**: Data access based on user clearance level

### Data Security
- **Input validation**: All API inputs are validated using Pydantic schemas
- **Parameterized queries**: SQLAlchemy ORM prevents SQL injection
- **Encrypted data**: Sensitive data is encrypted at rest and in transit
- **Secure defaults**: Conservative default settings for all components
- **Data classification**: Proper handling of classified information

### API Security
- **Rate limiting**: Prevents abuse of API endpoints
- **Audit logging**: All security-critical operations are logged
- **CORS protection**: Strict CORS policy to prevent cross-site attacks
- **Error handling**: Secure error handling to prevent information leakage

### Network Security
- **Kubernetes NetworkPolicy**: Restricts network traffic between services
- **TLS encryption**: All API endpoints use HTTPS
- **Secure message bus**: Authentication and authorization for RabbitMQ
- **Isolated networks**: Separation of collection and processing networks

### Threat Mitigations
- **OWASP Top 10**: Mitigations for all OWASP Top 10 vulnerabilities
- **Least privilege**: Services run with minimal permissions
- **Container security**: Secure container configuration and image scanning
- **Regular updates**: Dependencies are kept up to date to address vulnerabilities
- **Signal security**: Protection against malicious signals and data

## 7. Sample Code

### Signal Classification Example
```python
async def classify_signal(signal_data: np.ndarray, sample_rate: int) -> Dict[str, Any]:
    """
    Classify a signal based on its characteristics.
    
    Args:
        signal_data: Raw signal data (IQ samples)
        sample_rate: Sample rate in Hz
        
    Returns:
        Classification results
    """
    # Extract signal features
    features = extract_signal_features(signal_data, sample_rate)
    
    # Classify signal using machine learning model
    signal_type, confidence = ml_classifier.predict(features)
    
    # Determine modulation type
    modulation = determine_modulation(signal_data, sample_rate)
    
    # Estimate bandwidth
    bandwidth = estimate_bandwidth(signal_data, sample_rate)
    
    # Return classification results
    return {
        "signal_type": signal_type,
        "confidence": confidence,
        "modulation": modulation,
        "bandwidth": bandwidth,
        "features": features
    }
```

### Direction Finding Example
```python
async def perform_direction_finding(
    detections: List[Dict[str, Any]],
    collector_locations: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Perform direction finding to locate a signal source.
    
    Args:
        detections: List of signal detections from different collectors
        collector_locations: List of collector locations
        
    Returns:
        Direction finding results
    """
    # Extract bearings from detections
    bearings = []
    for detection, location in zip(detections, collector_locations):
        if detection.get("direction") and detection.get("direction").get("azimuth"):
            bearings.append({
                "lat": location.get("lat"),
                "lon": location.get("lon"),
                "azimuth": detection.get("direction").get("azimuth")
            })
    
    if len(bearings) < 2:
        return {
            "success": False,
            "error": "Insufficient bearings for triangulation",
            "bearings": bearings
        }
    
    # Perform triangulation
    source_location, accuracy = triangulate(bearings)
    
    # Return direction finding results
    return {
        "success": True,
        "source_location": source_location,
        "accuracy": accuracy,
        "method": "triangulation",
        "bearing_count": len(bearings)
    }
```

### Secure API Endpoint Example
```python
@router.post("/detections", response_model=SignalDetection, status_code=status.HTTP_201_CREATED)
async def create_signal_detection(
    detection: SignalDetectionCreate,
    current_user = Depends(has_permission("sigint:create")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new signal detection.
    
    Args:
        detection: Signal detection data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created signal detection
    """
    # Create signal detection model
    db_detection = SignalDetectionModel(
        collector_id=detection.collector_id,
        timestamp=detection.timestamp,
        frequency=detection.frequency,
        bandwidth=detection.bandwidth,
        signal_type=detection.signal_type,
        signal_strength=detection.signal_strength,
        snr=detection.snr,
        duration=detection.duration,
        location=detection.location.dict() if detection.location else None,
        direction=detection.direction.dict() if detection.direction else None,
        recording_id=detection.recording_id,
        metadata=detection.metadata
    )
    
    # Add to database
    db.add(db_detection)
    await db.commit()
    await db.refresh(db_detection)
    
    # Publish detection event
    await request.app.state.event_publisher.publish_detection_event(db_detection)
    
    # Check if detection matches known threats
    await check_for_threats(db_detection, request.app.state.threat_detector, db)
    
    return db_detection
```

## 8. Testing

The SIGINT Service includes the following tests:

### Unit Tests
- **Signal classification tests**: Test the signal classification algorithms
- **Direction finding tests**: Test the direction finding algorithms
- **Signal analysis tests**: Test the signal analysis functions
- **Event publishing tests**: Test the event publisher

### Integration Tests
- **API endpoint tests**: Test all API endpoints
- **Database integration tests**: Test database operations
- **Message bus integration tests**: Test event publishing and consuming
- **Authentication and authorization tests**: Test security mechanisms

### Example Test
```python
@pytest.mark.asyncio
async def test_classify_signal():
    """Test signal classification."""
    # Create test signal data (FM signal)
    sample_rate = 2.4e6  # 2.4 MHz
    duration = 1.0  # 1 second
    t = np.arange(0, duration, 1/sample_rate)
    carrier_freq = 100e3  # 100 kHz
    modulation_freq = 1e3  # 1 kHz
    modulation_index = 0.5
    
    # Generate FM signal
    modulation = np.sin(2 * np.pi * modulation_freq * t)
    phase = modulation_index * np.sin(2 * np.pi * modulation_freq * t)
    signal = np.exp(1j * (2 * np.pi * carrier_freq * t + phase))
    
    # Add noise
    noise = (np.random.randn(len(t)) + 1j * np.random.randn(len(t))) * 0.1
    signal_with_noise = signal + noise
    
    # Classify signal
    classification = await classify_signal(signal_with_noise, int(sample_rate))
    
    # Check classification results
    assert classification["signal_type"] == "FM"
    assert classification["confidence"] > 0.8
    assert classification["modulation"] == "FM"
    assert 0.9e3 < classification["bandwidth"] < 1.1e3
```

## 9. Containerization & Deployment

The SIGINT Service is containerized and deployed using the following:

### Dockerfile
```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    libfftw3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first to leverage Docker cache
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose port
EXPOSE 8000

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Kubernetes Deployment
The service is deployed to Kubernetes using Helm charts with the following resources:
- **Deployment**: Manages the SIGINT Service pods
- **Service**: Exposes the SIGINT Service API
- **ConfigMap**: Stores configuration parameters
- **Secret**: Stores sensitive information
- **NetworkPolicy**: Restricts network traffic

### Helm Chart
The Helm chart includes templates for all Kubernetes resources and a values.yaml file for configuration.

### NetworkPolicy
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: sigint-service-network-policy
  namespace: bulo-sentinel
spec:
  podSelector:
    matchLabels:
      app: sigint-service
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          app: api-gateway
    ports:
    - protocol: TCP
      port: 8000
  egress:
  - to:
    - podSelector:
        matchLabels:
          app: postgres
    ports:
    - protocol: TCP
      port: 5432
  - to:
    - podSelector:
        matchLabels:
          app: rabbitmq
    ports:
    - protocol: TCP
      port: 5672
  - to:
    - podSelector:
        matchLabels:
          app: minio
    ports:
    - protocol: TCP
      port: 9000
```
