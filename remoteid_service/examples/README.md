# Remote ID & Regulatory Compliance Service Examples

This directory contains example files for the Remote ID & Regulatory Compliance Service.

## Flight Plan Example

The `flight_plan.yaml` file contains an example flight plan for the Golden Gate Park in San Francisco. This file can be used with the flight plan CLI to create a new flight plan:

```bash
python -m remoteid_service.cli.flightplan create --file examples/flight_plan.yaml
```

## EASA SORA Submission Example

The `easa_sora_submission.yaml` file contains an example EASA SORA submission for an operation in Paris, France. This file can be used with the flight plan CLI to submit a flight plan to the EASA SORA system:

```bash
# First, create a flight plan
python -m remoteid_service.cli.flightplan create --file examples/flight_plan.yaml

# Then, submit the flight plan using the EASA SORA submission
python -m remoteid_service.cli.flightplan submit --id <flight_plan_id> --type easa_sora --file examples/easa_sora_submission.yaml
```

## FAA LAANC Submission Example

The `faa_laanc_submission.yaml` file contains an example FAA LAANC submission for an operation in San Francisco, USA. This file can be used with the flight plan CLI to submit a flight plan to the FAA LAANC system:

```bash
# First, create a flight plan
python -m remoteid_service.cli.flightplan create --file examples/flight_plan.yaml

# Then, submit the flight plan using the FAA LAANC submission
python -m remoteid_service.cli.flightplan submit --id <flight_plan_id> --type faa_laanc --file examples/faa_laanc_submission.yaml
```

## Remote ID Simulation Example

You can use the simulator CLI to simulate Remote ID broadcasting for multiple drones:

```bash
# Simulate 5 drones with full movement patterns
python -m remoteid_service.cli.simulator --mode full --drones 5 --interval 1

# Simulate 10 drones with random movement patterns
python -m remoteid_service.cli.simulator --mode random --drones 10 --interval 1 --duration 300
```

## NOTAM Integration Example

You can use the NOTAM CLI to import and search for NOTAMs:

```bash
# Import NOTAMs from the FAA for the US East region
python -m remoteid_service.cli.notam import --source faa --region us-east

# Search for active airspace NOTAMs
python -m remoteid_service.cli.notam search --source faa --type airspace --status active

# Check a flight plan for NOTAM conflicts
python -m remoteid_service.cli.notam check --flight-plan-id <flight_plan_id>
```
