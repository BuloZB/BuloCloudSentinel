# Services

This section documents the services that implement the business logic of the Drone Show microservice.

## Choreography Service

::: drone_show_service.services.choreography_service
    handler: python
    selection:
      members:
        - ChoreographyService

## Simulation Service

::: drone_show_service.services.simulation_service
    handler: python
    selection:
      members:
        - SimulationService

## Execution Service

::: drone_show_service.services.execution_service
    handler: python
    selection:
      members:
        - ExecutionService

## LED Control Service

::: drone_show_service.services.led_control_service
    handler: python
    selection:
      members:
        - LEDControlService

## Synchronization Service

::: drone_show_service.services.synchronization_service
    handler: python
    selection:
      members:
        - SynchronizationService

## Telemetry Service

::: drone_show_service.services.telemetry_service
    handler: python
    selection:
      members:
        - TelemetryService

## Logging Service

::: drone_show_service.services.logging_service
    handler: python
    selection:
      members:
        - LoggingService

## MinIO Service

::: drone_show_service.services.minio_service
    handler: python
    selection:
      members:
        - MinioService

## Sentinel Integration Service

::: drone_show_service.services.sentinel_integration
    handler: python
    selection:
      members:
        - SentinelIntegrationService
