#!/usr/bin/env python3
"""
Enhanced inference demo for Bulo.Cloud Sentinel.

This script demonstrates the enhanced inference capabilities of Bulo.Cloud Sentinel,
including batch inference, streaming inference, and distributed inference.
"""

import os
import sys
import argparse
import logging
import time
import asyncio
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from ai.inference import (
    InferenceEngine,
    BatchInferenceEngine,
    AsyncBatchInferenceEngine,
    StreamingInferenceEngine,
    DistributedInferenceEngine,
    WorkerServer
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run_basic_inference(model_path: str, backend: str, device: str):
    """
    Run basic inference.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
    """
    logger.info(f"Running basic inference with {backend} backend on {device}")
    
    # Initialize engine
    engine = InferenceEngine(backend=backend, model_path=model_path, device=device)
    
    # Create dummy input
    input_shape = engine.get_input_shapes()["input"]
    dummy_input = np.random.random(input_shape).astype(np.float32)
    
    # Run inference
    start_time = time.time()
    outputs = engine.predict({"input": dummy_input})
    inference_time = (time.time() - start_time) * 1000  # Convert to ms
    
    # Print results
    logger.info(f"Inference time: {inference_time:.2f} ms")
    logger.info(f"Output shapes: {[array.shape for array in outputs.values()]}")


def run_batch_inference(model_path: str, backend: str, device: str, batch_size: int, num_inputs: int):
    """
    Run batch inference.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        batch_size: Batch size
        num_inputs: Number of inputs to process
    """
    logger.info(f"Running batch inference with {backend} backend on {device}")
    
    # Initialize engine
    engine = BatchInferenceEngine(
        model_path=model_path,
        backend=backend,
        device=device,
        batch_size=batch_size
    )
    
    # Start engine
    engine.start()
    
    # Create dummy inputs
    input_shape = InferenceEngine(backend=backend, model_path=model_path, device=device).get_input_shapes()["input"]
    
    # Define callback
    def callback(input_id, outputs):
        logger.debug(f"Received output for input {input_id}")
    
    # Add inputs
    start_time = time.time()
    
    for i in range(num_inputs):
        dummy_input = np.random.random(input_shape).astype(np.float32)
        engine.add_input(f"input_{i}", {"input": dummy_input}, callback)
    
    # Wait for all outputs
    while engine.get_stats()["total_inputs"] > engine.get_stats()["total_batches"] * engine.batch_size:
        time.sleep(0.1)
    
    total_time = (time.time() - start_time) * 1000  # Convert to ms
    
    # Print results
    logger.info(f"Total time: {total_time:.2f} ms")
    logger.info(f"Average inference time: {engine.get_stats()['avg_inference_time']:.2f} ms")
    logger.info(f"Average batch size: {engine.get_stats()['avg_batch_size']:.2f}")
    
    # Stop engine
    engine.stop()


async def run_async_batch_inference(model_path: str, backend: str, device: str, batch_size: int, num_inputs: int):
    """
    Run asynchronous batch inference.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        batch_size: Batch size
        num_inputs: Number of inputs to process
    """
    logger.info(f"Running asynchronous batch inference with {backend} backend on {device}")
    
    # Initialize engine
    engine = AsyncBatchInferenceEngine(
        model_path=model_path,
        backend=backend,
        device=device,
        batch_size=batch_size
    )
    
    # Start engine
    await engine.start()
    
    # Create dummy inputs
    input_shape = InferenceEngine(backend=backend, model_path=model_path, device=device).get_input_shapes()["input"]
    
    # Define callback
    async def callback(input_id, outputs):
        logger.debug(f"Received output for input {input_id}")
    
    # Add inputs
    start_time = time.time()
    
    for i in range(num_inputs):
        dummy_input = np.random.random(input_shape).astype(np.float32)
        await engine.add_input(f"input_{i}", {"input": dummy_input}, callback)
    
    # Wait for all outputs
    while engine.stats["total_inputs"] > engine.stats["total_batches"] * engine.batch_size:
        await asyncio.sleep(0.1)
    
    total_time = (time.time() - start_time) * 1000  # Convert to ms
    
    # Print results
    logger.info(f"Total time: {total_time:.2f} ms")
    logger.info(f"Average inference time: {engine.stats['avg_inference_time']:.2f} ms")
    logger.info(f"Average batch size: {engine.stats['avg_batch_size']:.2f}")
    
    # Stop engine
    await engine.stop()


async def run_streaming_inference(model_path: str, backend: str, device: str, batch_size: int, duration: float):
    """
    Run streaming inference.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        batch_size: Batch size
        duration: Duration of the streaming inference in seconds
    """
    logger.info(f"Running streaming inference with {backend} backend on {device}")
    
    # Initialize engine
    engine = StreamingInferenceEngine(
        model_path=model_path,
        backend=backend,
        device=device,
        batch_size=batch_size
    )
    
    # Start engine
    await engine.start()
    
    # Create dummy inputs
    input_shape = InferenceEngine(backend=backend, model_path=model_path, device=device).get_input_shapes()["input"]
    
    # Define input generator
    def input_generator():
        return {"input": np.random.random(input_shape).astype(np.float32)}
    
    # Define output handler
    def output_handler(outputs):
        logger.debug(f"Received output with shapes: {[array.shape for array in outputs.values()]}")
    
    # Start stream
    await engine.start_stream("test_stream", input_generator, output_handler, interval=0.1)
    
    # Wait for duration
    logger.info(f"Streaming for {duration} seconds...")
    await asyncio.sleep(duration)
    
    # Stop stream
    await engine.stop_stream("test_stream")
    
    # Print results
    stats = engine.get_stats()
    logger.info(f"Total inputs: {stats['streams']['test_stream']['total_inputs']}")
    logger.info(f"Total outputs: {stats['streams']['test_stream']['total_outputs']}")
    logger.info(f"Average inference time: {stats['avg_inference_time']:.2f} ms")
    
    # Stop engine
    await engine.stop()


async def run_distributed_inference(
    model_path: str,
    backend: str,
    device: str,
    batch_size: int,
    num_inputs: int,
    worker_urls: List[str]
):
    """
    Run distributed inference.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        batch_size: Batch size
        num_inputs: Number of inputs to process
        worker_urls: List of worker URLs
    """
    logger.info(f"Running distributed inference with {backend} backend on {device}")
    
    # Initialize engine
    engine = DistributedInferenceEngine(
        model_path=model_path,
        backend=backend,
        device=device,
        batch_size=batch_size,
        worker_urls=worker_urls
    )
    
    # Start engine
    await engine.start()
    
    # Create dummy inputs
    input_shape = InferenceEngine(backend=backend, model_path=model_path, device=device).get_input_shapes()["input"]
    
    # Define callback
    async def callback(input_id, outputs):
        logger.debug(f"Received output for input {input_id}")
    
    # Add inputs
    start_time = time.time()
    
    for i in range(num_inputs):
        dummy_input = np.random.random(input_shape).astype(np.float32)
        await engine.add_input(f"input_{i}", {"input": dummy_input}, callback)
    
    # Wait for all outputs
    while engine.stats["total_inputs"] > engine.stats["total_outputs"]:
        await asyncio.sleep(0.1)
    
    total_time = (time.time() - start_time) * 1000  # Convert to ms
    
    # Print results
    logger.info(f"Total time: {total_time:.2f} ms")
    logger.info(f"Local inputs: {engine.stats['local_inputs']}")
    logger.info(f"Remote inputs: {engine.stats['remote_inputs']}")
    
    # Stop engine
    await engine.stop()


async def run_worker_server(model_path: str, backend: str, device: str, batch_size: int, host: str, port: int):
    """
    Run worker server.
    
    Args:
        model_path: Path to the model file
        backend: Backend to use ("tinygrad", "torch", "tflite")
        device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
        batch_size: Batch size
        host: Host to bind to
        port: Port to bind to
    """
    logger.info(f"Running worker server with {backend} backend on {device}")
    
    # Initialize server
    server = WorkerServer(
        model_path=model_path,
        backend=backend,
        device=device,
        batch_size=batch_size,
        host=host,
        port=port
    )
    
    # Start server
    await server.start()
    
    # Wait for Ctrl+C
    try:
        logger.info(f"Worker server running on {host}:{port}. Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping worker server...")
    finally:
        # Stop server
        await server.stop()


async def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Enhanced inference demo for Bulo.Cloud Sentinel")
    parser.add_argument("--model", required=True, help="Path to the model file")
    parser.add_argument("--backend", choices=["tinygrad", "torch", "tflite"], default="torch",
                        help="Backend to use (default: torch)")
    parser.add_argument("--device", choices=["CPU", "CUDA", "OCL", "AUTO"], default="AUTO",
                        help="Device to run inference on (default: AUTO)")
    parser.add_argument("--mode", choices=["basic", "batch", "async", "stream", "distributed", "worker"],
                        default="basic", help="Inference mode (default: basic)")
    parser.add_argument("--batch-size", type=int, default=4, help="Batch size (default: 4)")
    parser.add_argument("--num-inputs", type=int, default=10, help="Number of inputs to process (default: 10)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Duration of streaming inference in seconds (default: 5.0)")
    parser.add_argument("--worker-urls", nargs="+", default=[],
                        help="List of worker URLs for distributed inference")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind worker server to (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8070, help="Port to bind worker server to (default: 8070)")
    args = parser.parse_args()
    
    # Check if model exists
    if not os.path.exists(args.model):
        logger.error(f"Model not found: {args.model}")
        return
    
    # Run inference based on mode
    if args.mode == "basic":
        run_basic_inference(args.model, args.backend, args.device)
    elif args.mode == "batch":
        run_batch_inference(args.model, args.backend, args.device, args.batch_size, args.num_inputs)
    elif args.mode == "async":
        await run_async_batch_inference(args.model, args.backend, args.device, args.batch_size, args.num_inputs)
    elif args.mode == "stream":
        await run_streaming_inference(args.model, args.backend, args.device, args.batch_size, args.duration)
    elif args.mode == "distributed":
        await run_distributed_inference(
            args.model, args.backend, args.device, args.batch_size, args.num_inputs, args.worker_urls
        )
    elif args.mode == "worker":
        await run_worker_server(args.model, args.backend, args.device, args.batch_size, args.host, args.port)


if __name__ == "__main__":
    asyncio.run(main())
