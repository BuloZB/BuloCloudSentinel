# Enhanced Inference Engine for Bulo.Cloud Sentinel

This document describes the enhanced inference capabilities of Bulo.Cloud Sentinel, including batch inference, streaming inference, distributed inference, and model management.

## Overview

The enhanced inference engine provides advanced capabilities for running inference with different ML backends. It supports:

- **Batch Inference**: Process multiple inputs in batches for improved throughput
- **Streaming Inference**: Process a continuous stream of inputs in real-time
- **Distributed Inference**: Distribute inference workloads across multiple nodes
- **Model Management**: Convert, visualize, and analyze models
- **Model Optimization**: Optimize models for improved performance
- **Model Quantization**: Quantize models for reduced size and faster inference

## Batch Inference

Batch inference allows you to process multiple inputs in batches, which can significantly improve throughput. The batch inference engine supports both synchronous and asynchronous operation.

### Synchronous Batch Inference

```python
from ai.inference import BatchInferenceEngine
import numpy as np

# Initialize the engine
engine = BatchInferenceEngine(
    model_path="models/mobilenet_v2.npz",
    backend="tinygrad",
    device="AUTO",
    batch_size=4
)

# Start the engine
engine.start()

# Define callback function
def callback(input_id, outputs):
    print(f"Received output for input {input_id}")

# Add inputs to the queue
for i in range(10):
    input_data = np.random.random((1, 3, 224, 224)).astype(np.float32)
    engine.add_input(f"input_{i}", {"input": input_data}, callback)

# Wait for all outputs
while engine.get_stats()["total_inputs"] > engine.get_stats()["total_batches"] * engine.batch_size:
    time.sleep(0.1)

# Print statistics
print(f"Average inference time: {engine.get_stats()['avg_inference_time']:.2f} ms")
print(f"Average batch size: {engine.get_stats()['avg_batch_size']:.2f}")

# Stop the engine
engine.stop()
```

### Asynchronous Batch Inference

```python
import asyncio
from ai.inference import AsyncBatchInferenceEngine
import numpy as np

async def run_inference():
    # Initialize the engine
    engine = AsyncBatchInferenceEngine(
        model_path="models/mobilenet_v2.npz",
        backend="tinygrad",
        device="AUTO",
        batch_size=4
    )
    
    # Start the engine
    await engine.start()
    
    # Define callback function
    async def callback(input_id, outputs):
        print(f"Received output for input {input_id}")
    
    # Add inputs to the queue
    for i in range(10):
        input_data = np.random.random((1, 3, 224, 224)).astype(np.float32)
        await engine.add_input(f"input_{i}", {"input": input_data}, callback)
    
    # Wait for all outputs
    while engine.stats["total_inputs"] > engine.stats["total_batches"] * engine.batch_size:
        await asyncio.sleep(0.1)
    
    # Print statistics
    print(f"Average inference time: {engine.stats['avg_inference_time']:.2f} ms")
    print(f"Average batch size: {engine.stats['avg_batch_size']:.2f}")
    
    # Stop the engine
    await engine.stop()

# Run the async function
asyncio.run(run_inference())
```

## Streaming Inference

Streaming inference allows you to process a continuous stream of inputs in real-time. The streaming inference engine supports both synchronous and asynchronous operation.

```python
import asyncio
from ai.inference import StreamingInferenceEngine
import numpy as np

async def run_streaming_inference():
    # Initialize the engine
    engine = StreamingInferenceEngine(
        model_path="models/mobilenet_v2.npz",
        backend="tinygrad",
        device="AUTO",
        batch_size=4
    )
    
    # Start the engine
    await engine.start()
    
    # Define input generator
    def input_generator():
        return {"input": np.random.random((1, 3, 224, 224)).astype(np.float32)}
    
    # Define output handler
    def output_handler(outputs):
        print(f"Received output with shapes: {[array.shape for array in outputs.values()]}")
    
    # Start stream
    await engine.start_stream("test_stream", input_generator, output_handler, interval=0.1)
    
    # Wait for a while
    print("Streaming for 5 seconds...")
    await asyncio.sleep(5)
    
    # Stop stream
    await engine.stop_stream("test_stream")
    
    # Print statistics
    stats = engine.get_stats()
    print(f"Total inputs: {stats['streams']['test_stream']['total_inputs']}")
    print(f"Total outputs: {stats['streams']['test_stream']['total_outputs']}")
    print(f"Average inference time: {stats['avg_inference_time']:.2f} ms")
    
    # Stop the engine
    await engine.stop()

# Run the async function
asyncio.run(run_streaming_inference())
```

## Distributed Inference

Distributed inference allows you to distribute inference workloads across multiple nodes. The distributed inference engine supports both synchronous and asynchronous operation.

### Worker Server

First, start one or more worker servers:

```python
import asyncio
from ai.inference import WorkerServer

async def run_worker_server():
    # Initialize the server
    server = WorkerServer(
        model_path="models/mobilenet_v2.npz",
        backend="tinygrad",
        device="AUTO",
        batch_size=4,
        host="0.0.0.0",
        port=8070
    )
    
    # Start the server
    await server.start()
    
    # Wait for Ctrl+C
    try:
        print("Worker server running. Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("Stopping worker server...")
    finally:
        # Stop the server
        await server.stop()

# Run the async function
asyncio.run(run_worker_server())
```

### Distributed Inference Engine

Then, use the distributed inference engine to distribute workloads across the worker servers:

```python
import asyncio
from ai.inference import DistributedInferenceEngine
import numpy as np

async def run_distributed_inference():
    # Initialize the engine
    engine = DistributedInferenceEngine(
        model_path="models/mobilenet_v2.npz",
        backend="tinygrad",
        device="AUTO",
        batch_size=4,
        worker_urls=["http://worker1:8070", "http://worker2:8070"]
    )
    
    # Start the engine
    await engine.start()
    
    # Define callback function
    async def callback(input_id, outputs):
        print(f"Received output for input {input_id}")
    
    # Add inputs to the queue
    for i in range(10):
        input_data = np.random.random((1, 3, 224, 224)).astype(np.float32)
        await engine.add_input(f"input_{i}", {"input": input_data}, callback)
    
    # Wait for all outputs
    while engine.stats["total_inputs"] > engine.stats["total_outputs"]:
        await asyncio.sleep(0.1)
    
    # Print statistics
    print(f"Local inputs: {engine.stats['local_inputs']}")
    print(f"Remote inputs: {engine.stats['remote_inputs']}")
    
    # Stop the engine
    await engine.stop()

# Run the async function
asyncio.run(run_distributed_inference())
```

## Model Management

The model management tools allow you to convert, visualize, and analyze models. The following tools are available:

### Model Conversion

```python
from ai.inference import convert_torch_to_tinygrad, convert_onnx_to_tinygrad, convert_tflite_to_tinygrad

# Convert PyTorch model to TinyGrad
convert_torch_to_tinygrad("models/mobilenet_v2.pt", "models/mobilenet_v2.npz")

# Convert ONNX model to TinyGrad
convert_onnx_to_tinygrad("models/mobilenet_v2.onnx", "models/mobilenet_v2.npz")

# Convert TFLite model to TinyGrad
convert_tflite_to_tinygrad("models/mobilenet_v2.tflite", "models/mobilenet_v2.npz")
```

### Model Optimization

```python
from ai.inference import optimize_model

# Optimize ONNX model
optimize_model("models/mobilenet_v2.onnx", "models/mobilenet_v2_optimized.onnx", optimization_level=2)

# Optimize TFLite model
optimize_model("models/mobilenet_v2.tflite", "models/mobilenet_v2_optimized.tflite", optimization_level=2)
```

### Model Quantization

```python
from ai.inference import quantize_model

# Quantize ONNX model to INT8
quantize_model("models/mobilenet_v2.onnx", "models/mobilenet_v2_int8.onnx", quantization_type="int8")

# Quantize TFLite model to INT8
quantize_model("models/mobilenet_v2.tflite", "models/mobilenet_v2_int8.tflite", quantization_type="int8")
```

### Model Visualization

```python
from ai.inference import visualize_model_architecture, visualize_model_weights

# Visualize model architecture
visualize_model_architecture("models/mobilenet_v2.pt", "models/mobilenet_v2_architecture.png")

# Visualize model weights
visualize_model_weights("models/mobilenet_v2.npz", "models/mobilenet_v2_weights.png")
```

## Web Interfaces

The enhanced inference engine provides web interfaces for model conversion and management:

### Model Converter

The model converter web interface allows you to convert models between different formats:

```bash
python -m ai.inference.web_converter
```

Then, open your browser and navigate to `http://localhost:7860`.

### Model Manager

The model manager web interface allows you to manage models, including uploading, downloading, visualization, and analysis:

```bash
python -m ai.inference.model_manager
```

Then, open your browser and navigate to `http://localhost:7861`.

## Command-Line Tools

The enhanced inference engine provides command-line tools for model conversion and visualization:

### Model Conversion

```bash
python -m ai.inference.convert models/mobilenet_v2.pt models/mobilenet_v2.npz --format tinygrad
```

### Model Visualization

```bash
python -m ai.inference.visualize models/mobilenet_v2.pt --type architecture --output models/mobilenet_v2_architecture.png
```

## Enhanced Inference Demo

The enhanced inference demo shows how to use the enhanced inference capabilities:

```bash
# Basic inference
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode basic

# Batch inference
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode batch --batch-size 4 --num-inputs 10

# Asynchronous batch inference
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode async --batch-size 4 --num-inputs 10

# Streaming inference
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode stream --batch-size 4 --duration 5.0

# Distributed inference
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode distributed --batch-size 4 --num-inputs 10 --worker-urls http://worker1:8070 http://worker2:8070

# Worker server
python examples/enhanced_inference_demo.py --model models/mobilenet_v2.npz --mode worker --batch-size 4 --host 0.0.0.0 --port 8070
```

## Performance Considerations

When using the enhanced inference engine, consider the following performance considerations:

- **Batch Size**: Larger batch sizes generally improve throughput but may increase latency.
- **Device Selection**: Choose the appropriate device for your hardware (CPU, CUDA, OCL).
- **Model Optimization**: Optimize models for improved performance.
- **Model Quantization**: Quantize models for reduced size and faster inference.
- **Distributed Inference**: Distribute inference workloads across multiple nodes for improved throughput.

## Troubleshooting

### Batch Inference

- **Queue Full**: If the input queue is full, increase the `max_queue_size` parameter.
- **Slow Inference**: If inference is slow, try reducing the batch size or using a more powerful device.
- **Memory Issues**: If you encounter memory issues, try reducing the batch size or using a smaller model.

### Streaming Inference

- **High Latency**: If latency is high, try reducing the interval between inference runs.
- **Queue Full**: If the input queue is full, increase the `max_queue_size` parameter.
- **Memory Issues**: If you encounter memory issues, try using a smaller model or reducing the batch size.

### Distributed Inference

- **Worker Unavailable**: If a worker is unavailable, check the worker server logs and ensure it's running.
- **Network Issues**: If you encounter network issues, check the network connectivity between the client and worker servers.
- **Load Balancing**: If load balancing is uneven, try adjusting the worker selection algorithm.

## Further Reading

- [TinyGrad Documentation](https://github.com/tinygrad/tinygrad)
- [PyTorch Documentation](https://pytorch.org/docs/stable/index.html)
- [TensorFlow Lite Documentation](https://www.tensorflow.org/lite/guide)
- [ONNX Documentation](https://onnx.ai/onnx/index.html)
- [ONNX Runtime Documentation](https://onnxruntime.ai/docs/)
