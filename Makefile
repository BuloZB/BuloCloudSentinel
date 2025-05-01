# Makefile for Bulo.Cloud Sentinel

# Default values
ML_BACKEND ?= torch
DEVICE ?= AUTO

# Docker image name
IMAGE_NAME = bulosentinel

# Build Docker image
docker-build:
	docker build -t $(IMAGE_NAME) -f docker/bulosentinel.Dockerfile .

# Run Docker container
docker-run:
	docker run -it --rm \
		-e ML_BACKEND=$(ML_BACKEND) \
		-e DEVICE=$(DEVICE) \
		-p 8000:8000 \
		$(IMAGE_NAME)

# Run Docker container with CUDA support
docker-run-cuda:
	docker run -it --rm \
		--gpus all \
		-e ML_BACKEND=$(ML_BACKEND) \
		-e DEVICE=CUDA \
		-p 8000:8000 \
		$(IMAGE_NAME)

# Run tinygrad demo
tinygrad-demo:
	python examples/tinygrad_demo.py --backend $(ML_BACKEND) --device $(DEVICE)

# Run benchmark
benchmark:
	python examples/benchmark_backends.py --backends $(ML_BACKEND) --device $(DEVICE)

# Run benchmark for all backends
benchmark-all:
	python examples/benchmark_backends.py --backends tinygrad,torch,tflite --device $(DEVICE)

# Run tests
test:
	pytest tests/test_tinygrad_backend.py -v

# Run all tests
test-all:
	pytest tests/ -v

# Clean up
clean:
	rm -rf __pycache__
	rm -rf */__pycache__
	rm -rf */*/__pycache__
	rm -rf */*/*/__pycache__
	rm -rf .pytest_cache
	rm -rf models/*.npz
	rm -rf models/*.pt
	rm -rf models/*.pth
	rm -rf models/*.tflite
	rm -rf models/*.onnx
	rm -rf models/*.safetensors

# Help
help:
	@echo "Available targets:"
	@echo "  docker-build      - Build Docker image"
	@echo "  docker-run        - Run Docker container"
	@echo "  docker-run-cuda   - Run Docker container with CUDA support"
	@echo "  tinygrad-demo     - Run tinygrad demo"
	@echo "  benchmark         - Run benchmark for specified backend"
	@echo "  benchmark-all     - Run benchmark for all backends"
	@echo "  test              - Run tinygrad tests"
	@echo "  test-all          - Run all tests"
	@echo "  clean             - Clean up"
	@echo ""
	@echo "Environment variables:"
	@echo "  ML_BACKEND        - ML backend to use (tinygrad, torch, tflite)"
	@echo "  DEVICE            - Device to use (AUTO, CPU, CUDA, OCL)"

.PHONY: docker-build docker-run docker-run-cuda tinygrad-demo benchmark benchmark-all test test-all clean help
