@echo off
REM Start the Drone Show microservice using docker-compose

REM Build the Docker images
echo Building Docker images...
docker-compose build

REM Start the services
echo Starting services...
docker-compose up -d

REM Wait for the database to be ready
echo Waiting for the database to be ready...
timeout /t 10 /nobreak > nul

REM Initialize the database
echo Initializing the database...
docker-compose exec api python -m drone_show_service.init_db

REM Generate sample data
echo Generating sample data...
docker-compose exec api python -m drone_show_service.generate_sample_data

REM Show the logs
echo Services started. Showing logs...
docker-compose logs -f
