# Electronic Warfare (EW) Service

## 1. Feature Name
Electronic Warfare (EW) Service

## 2. Description & Benefits
The Electronic Warfare (EW) Service provides advanced capabilities for electronic attack, electronic protection, and electronic support. It enables the Bulo.Cloud Sentinel platform to detect, analyze, and counter electronic threats, as well as to conduct electronic operations when necessary.

### Key Features:
- **Electronic Attack (EA)**: Generate and direct electromagnetic energy to disrupt or neutralize enemy electronic systems
- **Electronic Protection (EP)**: Protect friendly electronic systems from enemy electronic attacks
- **Electronic Support (ES)**: Detect, identify, and locate sources of electromagnetic energy for threat recognition
- **Signal Jamming**: Targeted jamming of specific frequencies or signals
- **Spectrum Monitoring**: Real-time monitoring of the electromagnetic spectrum
- **Threat Analysis**: Identify and analyze electronic threats
- **Countermeasure Deployment**: Automated deployment of electronic countermeasures
- **Secure API**: Role-based access control for all EW capabilities
- **Event-driven Architecture**: Publish EW events to the message bus for real-time updates

### Benefits:
- **Enhanced Operational Security**: Protection against electronic surveillance and attacks
- **Tactical Advantage**: Ability to disrupt enemy communications and sensors
- **Threat Awareness**: Early detection and identification of electronic threats
- **Automated Response**: Rapid deployment of countermeasures against electronic threats
- **Integrated Operations**: Seamless integration with ISR and SIGINT capabilities
- **Scalable Architecture**: Support for multiple EW platforms and systems
- **Secure by Design**: Built with security best practices from the ground up

## 3. Integration with Existing Architecture

The EW Service integrates with the following Bulo.Cloud Sentinel components:

- **Authentication and Authorization**: Uses the existing JWT-based authentication system and role-based access control
- **Database**: Stores EW data in PostgreSQL using SQLAlchemy ORM
- **Message Bus**: Publishes EW events to RabbitMQ for real-time updates
- **Object Storage**: Stores waveform templates and signal recordings in MinIO
- **Frontend**: Provides API endpoints for the React frontend to display EW data

### Service Interactions:
- **ISR Service**: Coordinates with ISR for target identification and tracking
- **SIGINT Service**: Receives signal intelligence for threat analysis and targeting
- **Anti-Jamming Service**: Coordinates with anti-jamming systems to avoid friendly fire
- **Drone Swarm System**: Coordinates EW platforms for optimal coverage
- **Mission Planning**: Receives EW mission parameters and objectives

### Message Flows:
1. SIGINT Service detects and identifies signals of interest
2. EW Service analyzes signals for threat assessment
3. EW Service deploys appropriate countermeasures
4. EW Service publishes status updates and effectiveness reports
5. Mission Planning receives feedback for mission adjustment

## 4. API Endpoints

The EW Service provides the following API endpoints:

### EW Platforms
```
GET /api/v1/platforms - Get all EW platforms
POST /api/v1/platforms - Register a new EW platform
GET /api/v1/platforms/{platform_id} - Get a specific EW platform
PUT /api/v1/platforms/{platform_id} - Update an EW platform
DELETE /api/v1/platforms/{platform_id} - Unregister an EW platform
```

### Electronic Attacks
```
GET /api/v1/attacks - Get all electronic attacks
POST /api/v1/attacks - Create a new electronic attack
GET /api/v1/attacks/{attack_id} - Get a specific electronic attack
PUT /api/v1/attacks/{attack_id} - Update an electronic attack
DELETE /api/v1/attacks/{attack_id} - Cancel an electronic attack
```

### Electronic Protection
```
GET /api/v1/protections - Get all electronic protection measures
POST /api/v1/protections - Create a new protection measure
GET /api/v1/protections/{protection_id} - Get a specific protection measure
PUT /api/v1/protections/{protection_id} - Update a protection measure
DELETE /api/v1/protections/{protection_id} - Remove a protection measure
```

### Electronic Support
```
GET /api/v1/support - Get all electronic support activities
POST /api/v1/support - Create a new support activity
GET /api/v1/support/{support_id} - Get a specific support activity
PUT /api/v1/support/{support_id} - Update a support activity
DELETE /api/v1/support/{support_id} - End a support activity
```

### Spectrum Monitoring
```
GET /api/v1/spectrum - Get spectrum monitoring data
POST /api/v1/spectrum/scan - Perform a spectrum scan
GET /api/v1/spectrum/bands - Get defined frequency bands
POST /api/v1/spectrum/bands - Define a new frequency band
```

### Threats
```
GET /api/v1/threats - Get all electronic threats
POST /api/v1/threats - Create a new threat
GET /api/v1/threats/{threat_id} - Get a specific threat
PUT /api/v1/threats/{threat_id} - Update a threat
DELETE /api/v1/threats/{threat_id} - Remove a threat
```

### Countermeasures
```
GET /api/v1/countermeasures - Get all countermeasures
POST /api/v1/countermeasures - Create a new countermeasure
GET /api/v1/countermeasures/{countermeasure_id} - Get a specific countermeasure
PUT /api/v1/countermeasures/{countermeasure_id} - Update a countermeasure
DELETE /api/v1/countermeasures/{countermeasure_id} - Remove a countermeasure
POST /api/v1/countermeasures/{countermeasure_id}/deploy - Deploy a countermeasure
```

### Waveform Templates
```
GET /api/v1/waveforms - Get all waveform templates
POST /api/v1/waveforms - Create a new waveform template
GET /api/v1/waveforms/{waveform_id} - Get a specific waveform template
PUT /api/v1/waveforms/{waveform_id} - Update a waveform template
DELETE /api/v1/waveforms/{waveform_id} - Remove a waveform template
```

## 5. Data Models

The EW Service uses the following data models:

### EwPlatform
```python
class EwPlatform:
    id: UUID
    name: str
    type: PlatformType  # drone, vehicle, fixed, etc.
    capabilities: List[EwCapability]  # EA, EP, ES
    frequency_range: FrequencyRangeSchema  # min_freq, max_freq in Hz
    power_output: float  # Watts
    status: str  # online, offline, error
    location: Optional[LocationSchema]
    orientation: Optional[OrientationSchema]
    configuration: Optional[Dict[str, Any]]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### ElectronicAttack
```python
class ElectronicAttack:
    id: UUID
    platform_id: UUID
    target_id: Optional[UUID]
    attack_type: AttackType  # jamming, spoofing, etc.
    frequency: float  # Hz
    bandwidth: float  # Hz
    power: float  # Watts
    waveform_id: Optional[UUID]
    start_time: datetime
    end_time: Optional[datetime]
    status: str  # scheduled, active, completed, failed
    effectiveness: Optional[float]  # 0.0 to 1.0
    location: Optional[LocationSchema]
    direction: Optional[DirectionSchema]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### ElectronicProtection
```python
class ElectronicProtection:
    id: UUID
    platform_id: UUID
    protection_type: ProtectionType  # frequency_hopping, directional_filtering, etc.
    frequency_range: FrequencyRangeSchema
    start_time: datetime
    end_time: Optional[datetime]
    status: str  # active, inactive
    effectiveness: Optional[float]  # 0.0 to 1.0
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### ElectronicSupport
```python
class ElectronicSupport:
    id: UUID
    platform_id: UUID
    support_type: SupportType  # signal_collection, direction_finding, etc.
    frequency_range: FrequencyRangeSchema
    start_time: datetime
    end_time: Optional[datetime]
    status: str  # active, inactive
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### SpectrumScan
```python
class SpectrumScan:
    id: UUID
    platform_id: UUID
    start_frequency: float  # Hz
    end_frequency: float  # Hz
    resolution: float  # Hz
    scan_time: datetime
    data_url: str  # URL to spectrum data
    peaks: List[Dict[str, Any]]  # List of detected signal peaks
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
```

### ElectronicThreat
```python
class ElectronicThreat:
    id: UUID
    threat_type: ThreatType  # jamming, surveillance, etc.
    frequency_range: FrequencyRangeSchema
    signal_strength: float  # dBm
    first_detected: datetime
    last_detected: datetime
    location: Optional[LocationSchema]
    direction: Optional[DirectionSchema]
    confidence: float  # 0.0 to 1.0
    severity: str  # low, medium, high, critical
    status: str  # active, inactive
    source_id: Optional[UUID]  # SIGINT source ID
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### Countermeasure
```python
class Countermeasure:
    id: UUID
    name: str
    countermeasure_type: CountermeasureType  # jamming, spoofing, etc.
    threat_types: List[ThreatType]  # Threat types this countermeasure is effective against
    frequency_range: FrequencyRangeSchema
    power_required: float  # Watts
    effectiveness: float  # 0.0 to 1.0
    description: str
    configuration_template: Dict[str, Any]
    waveform_id: Optional[UUID]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### CountermeasureDeployment
```python
class CountermeasureDeployment:
    id: UUID
    countermeasure_id: UUID
    platform_id: UUID
    threat_id: Optional[UUID]
    start_time: datetime
    end_time: Optional[datetime]
    status: str  # scheduled, active, completed, failed
    effectiveness: Optional[float]  # 0.0 to 1.0
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

### WaveformTemplate
```python
class WaveformTemplate:
    id: UUID
    name: str
    description: str
    waveform_type: WaveformType  # noise, tone, chirp, etc.
    parameters: Dict[str, Any]
    file_url: Optional[str]
    preview_url: Optional[str]
    metadata: Optional[Dict[str, Any]]
    created_at: datetime
    updated_at: datetime
```

## 6. Security Considerations

The EW Service implements the following security measures:

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
- **Isolated networks**: Separation of operational and management networks

### Threat Mitigations
- **OWASP Top 10**: Mitigations for all OWASP Top 10 vulnerabilities
- **Least privilege**: Services run with minimal permissions
- **Container security**: Secure container configuration and image scanning
- **Regular updates**: Dependencies are kept up to date to address vulnerabilities
- **Operational security**: Protection against unauthorized EW operations

## 7. Sample Code

### Electronic Attack Example
```python
async def create_jamming_attack(
    platform_id: UUID,
    frequency: float,
    bandwidth: float,
    power: float,
    duration: int,
    direction: Optional[Dict[str, float]] = None
) -> Dict[str, Any]:
    """
    Create a new jamming attack.
    
    Args:
        platform_id: EW platform ID
        frequency: Center frequency in Hz
        bandwidth: Bandwidth in Hz
        power: Power in Watts
        duration: Duration in seconds
        direction: Optional direction (azimuth, elevation)
        
    Returns:
        Created electronic attack
    """
    # Get platform
    platform = await get_ew_platform(platform_id)
    
    if not platform:
        raise ValueError(f"Platform {platform_id} not found")
    
    # Check if platform has EA capability
    if "EA" not in platform.get("capabilities", []):
        raise ValueError(f"Platform {platform_id} does not have Electronic Attack capability")
    
    # Check if frequency is within platform's range
    freq_range = platform.get("frequency_range", {})
    min_freq = freq_range.get("min_freq", 0)
    max_freq = freq_range.get("max_freq", 0)
    
    if frequency < min_freq or frequency > max_freq:
        raise ValueError(f"Frequency {frequency} Hz is outside platform's range ({min_freq}-{max_freq} Hz)")
    
    # Check if power is within platform's capability
    if power > platform.get("power_output", 0):
        raise ValueError(f"Power {power} W exceeds platform's maximum power output {platform.get('power_output')} W")
    
    # Create attack
    now = datetime.utcnow()
    attack = {
        "id": str(uuid.uuid4()),
        "platform_id": str(platform_id),
        "attack_type": "jamming",
        "frequency": frequency,
        "bandwidth": bandwidth,
        "power": power,
        "start_time": now.isoformat(),
        "end_time": (now + timedelta(seconds=duration)).isoformat(),
        "status": "scheduled",
        "location": platform.get("location"),
        "direction": direction,
        "metadata": {
            "created_by": "system",
            "scheduled_duration": duration
        },
        "created_at": now.isoformat(),
        "updated_at": now.isoformat()
    }
    
    # Save attack to database
    await save_electronic_attack(attack)
    
    # Schedule attack execution
    await schedule_attack_execution(attack)
    
    return attack
```

### Countermeasure Deployment Example
```python
async def deploy_countermeasure(
    countermeasure_id: UUID,
    platform_id: UUID,
    threat_id: Optional[UUID] = None,
    configuration: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Deploy a countermeasure against a threat.
    
    Args:
        countermeasure_id: Countermeasure ID
        platform_id: EW platform ID
        threat_id: Optional threat ID
        configuration: Optional configuration overrides
        
    Returns:
        Countermeasure deployment
    """
    # Get countermeasure
    countermeasure = await get_countermeasure(countermeasure_id)
    
    if not countermeasure:
        raise ValueError(f"Countermeasure {countermeasure_id} not found")
    
    # Get platform
    platform = await get_ew_platform(platform_id)
    
    if not platform:
        raise ValueError(f"Platform {platform_id} not found")
    
    # Check if platform has required capability
    required_capability = "EA" if countermeasure.get("countermeasure_type") in ["jamming", "spoofing"] else "EP"
    
    if required_capability not in platform.get("capabilities", []):
        raise ValueError(f"Platform {platform_id} does not have required capability {required_capability}")
    
    # Get threat if provided
    threat = None
    if threat_id:
        threat = await get_electronic_threat(threat_id)
        
        if not threat:
            raise ValueError(f"Threat {threat_id} not found")
    
    # Merge configuration with template
    merged_config = countermeasure.get("configuration_template", {}).copy()
    if configuration:
        merged_config.update(configuration)
    
    # Create deployment
    now = datetime.utcnow()
    deployment = {
        "id": str(uuid.uuid4()),
        "countermeasure_id": str(countermeasure_id),
        "platform_id": str(platform_id),
        "threat_id": str(threat_id) if threat_id else None,
        "start_time": now.isoformat(),
        "status": "scheduled",
        "configuration": merged_config,
        "metadata": {
            "created_by": "system",
            "threat_type": threat.get("threat_type") if threat else None,
            "threat_severity": threat.get("severity") if threat else None
        },
        "created_at": now.isoformat(),
        "updated_at": now.isoformat()
    }
    
    # Save deployment to database
    await save_countermeasure_deployment(deployment)
    
    # Schedule deployment execution
    await schedule_countermeasure_execution(deployment)
    
    return deployment
```

### Secure API Endpoint Example
```python
@router.post("/attacks", response_model=ElectronicAttack, status_code=status.HTTP_201_CREATED)
async def create_electronic_attack(
    attack: ElectronicAttackCreate,
    current_user = Depends(has_permission("ew:create_attack")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new electronic attack.
    
    Args:
        attack: Electronic attack data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created electronic attack
    """
    # Check if user has required clearance
    if attack.attack_type in ["jamming", "spoofing"] and not has_clearance(current_user, "secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get platform
    platform = await get_ew_platform_by_id(attack.platform_id, db)
    
    if not platform:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Platform {attack.platform_id} not found"
        )
    
    # Check if platform has EA capability
    if "EA" not in platform.capabilities:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Platform {attack.platform_id} does not have Electronic Attack capability"
        )
    
    # Create attack model
    db_attack = ElectronicAttackModel(
        platform_id=attack.platform_id,
        target_id=attack.target_id,
        attack_type=attack.attack_type,
        frequency=attack.frequency,
        bandwidth=attack.bandwidth,
        power=attack.power,
        waveform_id=attack.waveform_id,
        start_time=attack.start_time,
        end_time=attack.end_time,
        status="scheduled",
        location=attack.location.dict() if attack.location else None,
        direction=attack.direction.dict() if attack.direction else None,
        metadata=attack.metadata
    )
    
    # Add to database
    db.add(db_attack)
    await db.commit()
    await db.refresh(db_attack)
    
    # Log the operation
    await log_security_event(
        event_type="electronic_attack_created",
        user_id=current_user.id,
        resource_id=str(db_attack.id),
        resource_type="electronic_attack",
        details={
            "attack_type": attack.attack_type,
            "platform_id": str(attack.platform_id),
            "frequency": attack.frequency,
            "power": attack.power
        }
    )
    
    # Schedule attack execution
    await schedule_attack_execution(db_attack)
    
    return db_attack
```

## 8. Testing

The EW Service includes the following tests:

### Unit Tests
- **Electronic attack tests**: Test the electronic attack functionality
- **Electronic protection tests**: Test the electronic protection functionality
- **Electronic support tests**: Test the electronic support functionality
- **Countermeasure tests**: Test the countermeasure deployment functionality
- **Waveform generation tests**: Test the waveform generation functionality

### Integration Tests
- **API endpoint tests**: Test all API endpoints
- **Database integration tests**: Test database operations
- **Message bus integration tests**: Test event publishing and consuming
- **Authentication and authorization tests**: Test security mechanisms

### Example Test
```python
@pytest.mark.asyncio
async def test_create_jamming_attack():
    """Test creating a jamming attack."""
    # Create test platform
    platform = {
        "id": str(uuid.uuid4()),
        "name": "Test Platform",
        "type": "drone",
        "capabilities": ["EA", "EP", "ES"],
        "frequency_range": {"min_freq": 100e6, "max_freq": 6e9},
        "power_output": 50.0,
        "status": "online",
        "location": {"lat": 40.0, "lon": -74.0, "alt": 100.0},
        "orientation": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    }
    
    # Mock platform retrieval
    with patch("services.ew_platform_manager.get_ew_platform", return_value=platform):
        # Mock attack saving
        with patch("services.ew_platform_manager.save_electronic_attack") as mock_save:
            # Mock attack scheduling
            with patch("services.ew_platform_manager.schedule_attack_execution") as mock_schedule:
                # Create jamming attack
                attack = await create_jamming_attack(
                    platform_id=uuid.UUID(platform["id"]),
                    frequency=2.4e9,
                    bandwidth=20e6,
                    power=10.0,
                    duration=60,
                    direction={"azimuth": 45.0, "elevation": 0.0}
                )
                
                # Check attack properties
                assert attack["platform_id"] == platform["id"]
                assert attack["attack_type"] == "jamming"
                assert attack["frequency"] == 2.4e9
                assert attack["bandwidth"] == 20e6
                assert attack["power"] == 10.0
                assert attack["status"] == "scheduled"
                assert attack["location"] == platform["location"]
                assert attack["direction"] == {"azimuth": 45.0, "elevation": 0.0}
                
                # Check that attack was saved
                mock_save.assert_called_once()
                
                # Check that attack was scheduled
                mock_schedule.assert_called_once()
```

## 9. Containerization & Deployment

The EW Service is containerized and deployed using the following:

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
- **Deployment**: Manages the EW Service pods
- **Service**: Exposes the EW Service API
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
  name: ew-service-network-policy
  namespace: bulo-sentinel
spec:
  podSelector:
    matchLabels:
      app: ew-service
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
  - from:
    - podSelector:
        matchLabels:
          app: sigint-service
    ports:
    - protocol: TCP
      port: 8000
  - from:
    - podSelector:
        matchLabels:
          app: isr-service
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
