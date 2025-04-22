#!/bin/bash

# Start the Drone Show microservice using docker-compose

# Build the Docker images
echo "Building Docker images..."
docker-compose build

# Start the services
echo "Starting services..."
docker-compose up -d

# Wait for the database to be ready
echo "Waiting for the database to be ready..."
sleep 10

# Initialize the database
echo "Initializing the database..."
docker-compose exec api python -m drone_show_service.init_db

# Generate sample data
echo "Generating sample data..."
docker-compose exec api python -m drone_show_service.generate_sample_data

# Show the logs
echo "Services started. Showing logs..."
docker-compose logs -f
