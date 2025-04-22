# Models

This section documents the data models used in the Drone Show microservice.

## Choreography Models

::: drone_show_service.models.choreography
    handler: python
    selection:
      members:
        - Choreography
        - ChoreographyMetadata
        - ChoreographyType
        - ChoreographyStatus
        - ChoreographyCreate
        - ChoreographyUpdate
        - ChoreographyResponse
        - DroneTrajectory
        - Waypoint
        - Position
        - LEDState
        - LEDColor
        - LEDEffect
        - Formation
        - FormationType
        - FormationPoint

## Simulation Models

::: drone_show_service.models.choreography
    handler: python
    selection:
      members:
        - SimulationSettings
        - SimulationFrame
        - SimulationResponse

## Execution Models

::: drone_show_service.models.choreography
    handler: python
    selection:
      members:
        - ExecutionSettings
        - ExecutionStatus
        - ExecutionResponse
        - DroneStatus

## Database Models

::: drone_show_service.models.database
    handler: python
    selection:
      members:
        - Base
        - ChoreographyDB
        - SimulationDB
        - ExecutionDB
        - ExecutionLogDB
