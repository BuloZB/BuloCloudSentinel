# AI Backends for Bulo.Cloud Sentinel

This document describes the available AI backends for Bulo.Cloud Sentinel and how to use them.

## Overview

Bulo.Cloud Sentinel supports multiple AI backends for inference, allowing you to choose the best backend for your specific hardware and use case. The following backends are currently supported:

- **TinyGrad**: A lightweight ML runtime for on-board inference
- **PyTorch**: A full-featured ML framework for high-performance inference
- **TensorFlow Lite**: A lightweight ML runtime for mobile and edge devices

## Backend Selection

You can select the backend to use via the `ML_BACKEND` environment variable or the `--ml-backend` command-line option. The following values are supported:

- `tinygrad`: Use the TinyGrad backend
- `torch`: Use the PyTorch backend (default)
- `tflite`: Use the TensorFlow Lite backend

Example:

```bash
# Using environment variable
export ML_BACKEND=tinygrad
python -m main

# Using command-line option
python -m main --ml-backend=tinygrad
```

In Docker, you can set the environment variable in the `docker-compose.yml` file:

```yaml
services:
  bulosentinel:
    image: bulosentinel:latest
    environment:
      - ML_BACKEND=tinygrad
```

## Hardware Support

Each backend supports different hardware accelerators:

| Backend  | CPU | CUDA (NVIDIA) | OpenCL | NNAPI (Android) |
|----------|-----|---------------|--------|-----------------|
| TinyGrad | ✅  | ✅            | ✅     | ❌              |
| PyTorch  | ✅  | ✅            | ❌     | ❌              |
| TFLite   | ✅  | ✅            | ❌     | ✅              |

The backend will automatically detect the best available hardware accelerator, but you can also specify the device to use via the `DEVICE` environment variable or the `--device` command-line option. The following values are supported:

- `AUTO`: Automatically detect the best available device (default)
- `CPU`: Use the CPU
- `CUDA`: Use NVIDIA CUDA (if available)
- `OCL`: Use OpenCL (if available, TinyGrad only)
- `NNAPI`: Use Android NNAPI (if available, TFLite only)

Example:

```bash
# Using environment variable
export ML_BACKEND=tinygrad
export DEVICE=CUDA
python -m main

# Using command-line option
python -m main --ml-backend=tinygrad --device=CUDA
```

## Model Formats

Each backend supports different model formats:

| Backend  | Supported Formats                |
|----------|----------------------------------|
| TinyGrad | `.npz`, `.safetensors`          |
| PyTorch  | `.pt`, `.pth`, `.onnx`          |
| TFLite   | `.tflite`                        |

## Performance Benchmarks

The following benchmarks were performed on various hardware platforms using a MobileNetV2 model for image classification:

### Raspberry Pi 4 (ARM64)

| Backend  | Inference Time (ms) | FPS  |
|----------|---------------------|------|
| TinyGrad | 120                 | 8.3  |
| PyTorch  | 150                 | 6.7  |
| TFLite   | 90                  | 11.1 |

### NVIDIA Jetson Xavier NX (CUDA)

| Backend  | Inference Time (ms) | FPS  |
|----------|---------------------|------|
| TinyGrad | 15                  | 66.7 |
| PyTorch  | 12                  | 83.3 |
| TFLite   | 20                  | 50.0 |

### x86 Linux Host (CPU)

| Backend  | Inference Time (ms) | FPS  |
|----------|---------------------|------|
| TinyGrad | 40                  | 25.0 |
| PyTorch  | 35                  | 28.6 |
| TFLite   | 45                  | 22.2 |

## Usage Example

Here's a simple example of how to use the inference engine with different backends:

```python
from ai.inference import InferenceEngine
import numpy as np

# Initialize the engine with the desired backend
engine = InferenceEngine(backend="tinygrad", model_path="models/mobilenet_v2.npz", device="AUTO")

# Prepare input data
input_data = np.random.random((1, 3, 224, 224)).astype(np.float32)  # Example input for MobileNetV2

# Run inference
outputs = engine.predict({"input": input_data})

# Process outputs
predictions = outputs["output"]
top_class = np.argmax(predictions[0])
confidence = predictions[0][top_class]

print(f"Top class: {top_class}, confidence: {confidence:.4f}")
```

## Troubleshooting

### TinyGrad

- **ImportError: No module named 'tinygrad'**: Make sure you have installed TinyGrad with `pip install tinygrad`.
- **CUDA not available**: Make sure you have installed the CUDA toolkit and that it's in your PATH.
- **OpenCL not available**: Make sure you have installed the OpenCL runtime and that it's in your PATH.

### PyTorch

- **ImportError: No module named 'torch'**: Make sure you have installed PyTorch with `pip install torch`.
- **CUDA not available**: Make sure you have installed the CUDA toolkit and that it's in your PATH.

### TFLite

- **ImportError: No module named 'tensorflow'**: Make sure you have installed TensorFlow with `pip install tensorflow`.
- **CUDA not available**: Make sure you have installed the CUDA toolkit and that it's in your PATH.
- **NNAPI not available**: NNAPI is only available on Android devices.

## Further Reading

- [TinyGrad Documentation](https://github.com/tinygrad/tinygrad)
- [PyTorch Documentation](https://pytorch.org/docs/stable/index.html)
- [TensorFlow Lite Documentation](https://www.tensorflow.org/lite/guide)
