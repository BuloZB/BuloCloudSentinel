#!/bin/bash

# Set environment variables
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/backend

# Run the tests
python -m unittest discover -s backend/tests
