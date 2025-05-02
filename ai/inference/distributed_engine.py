"""
Distributed inference engine for Bulo.Cloud Sentinel.

This module provides a distributed inference engine for running inference across
multiple nodes with different ML backends.
"""

import logging
import time
import asyncio
import os
import sys
import ssl
import hmac
from typing import Dict, Any, Optional, List, Tuple, Callable
import numpy as np
from pathlib import Path
import aiohttp

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from ai.inference.batch_engine import AsyncBatchInferenceEngine
from ai.inference.security import is_safe_file, validate_model_format

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DistributedInferenceEngine:
    """
    Distributed inference engine for Bulo.Cloud Sentinel.

    This class provides a distributed inference engine for running inference across
    multiple nodes with different ML backends. It supports load balancing, fault tolerance,
    and dynamic scaling.
    """

    def __init__(
        self,
        model_path: str,
        backend: str = "torch",
        device: str = "AUTO",
        batch_size: int = 4,
        max_queue_size: int = 100,
        timeout: float = 1.0,
        worker_urls: Optional[List[str]] = None
    ):
        """
        Initialize the distributed inference engine.

        Args:
            model_path: Path to the model file
            backend: Backend to use ("tinygrad", "torch", "tflite")
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
            batch_size: Maximum batch size for inference
            max_queue_size: Maximum size of the input queue
            timeout: Timeout for waiting for inputs in seconds
            worker_urls: List of worker URLs
        """
        self.model_path = model_path
        self.backend = backend
        self.device = device
        self.batch_size = batch_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.worker_urls = worker_urls or []

        # Initialize local engine
        self.local_engine = AsyncBatchInferenceEngine(
            model_path=model_path,
            backend=backend,
            device=device,
            batch_size=batch_size,
            max_queue_size=max_queue_size,
            timeout=timeout
        )

        # Initialize worker clients
        self.worker_clients = {}

        # Initialize input queue
        self.input_queue = asyncio.Queue(maxsize=max_queue_size)

        # Initialize output queue
        self.output_queue = asyncio.Queue()

        # Initialize worker task
        self.worker_task = None
        self.running = False

        # Initialize stats
        self.stats = {
            "total_inputs": 0,
            "total_outputs": 0,
            "local_inputs": 0,
            "remote_inputs": 0,
            "worker_stats": {}
        }

    async def start(self):
        """Start the distributed inference engine."""
        if self.running:
            logger.warning("Distributed inference engine is already running")
            return

        # Set running flag
        self.running = True

        # Start local engine
        await self.local_engine.start()

        # Initialize worker clients
        for url in self.worker_urls:
            await self._add_worker(url)

        # Start worker task
        self.worker_task = asyncio.create_task(self._worker_loop())

        logger.info(f"Distributed inference engine started with {len(self.worker_urls) + 1} workers")

    async def stop(self):
        """Stop the distributed inference engine."""
        if not self.running:
            logger.warning("Distributed inference engine is not running")
            return

        # Clear running flag
        self.running = False

        # Wait for worker task to finish
        if self.worker_task:
            try:
                await asyncio.wait_for(self.worker_task, timeout=self.timeout)
            except asyncio.TimeoutError:
                logger.warning("Timeout waiting for worker task to finish")

        # Stop local engine
        await self.local_engine.stop()

        # Close worker clients
        for client in self.worker_clients.values():
            await client.close()

        # Clear worker clients
        self.worker_clients = {}

        # Clear queues
        while not self.input_queue.empty():
            try:
                self.input_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        while not self.output_queue.empty():
            try:
                self.output_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        logger.info("Distributed inference engine stopped")

    async def add_worker(self, url: str) -> bool:
        """
        Add a worker to the distributed inference engine.

        Args:
            url: Worker URL

        Returns:
            True if the worker was added, False otherwise
        """
        if not self.running:
            logger.warning("Distributed inference engine is not running")
            return False

        return await self._add_worker(url)

    async def remove_worker(self, url: str) -> bool:
        """
        Remove a worker from the distributed inference engine.

        Args:
            url: Worker URL

        Returns:
            True if the worker was removed, False otherwise
        """
        if not self.running:
            logger.warning("Distributed inference engine is not running")
            return False

        if url not in self.worker_clients:
            logger.warning(f"Worker {url} not found")
            return False

        # Close worker client
        await self.worker_clients[url].close()

        # Remove worker client
        del self.worker_clients[url]

        # Remove worker URL
        self.worker_urls.remove(url)

        # Remove worker stats
        if url in self.stats["worker_stats"]:
            del self.stats["worker_stats"][url]

        logger.info(f"Worker {url} removed")

        return True

    async def add_input(self, input_id: str, inputs: Dict[str, np.ndarray], callback: Optional[Callable] = None) -> bool:
        """
        Add an input to the queue.

        Args:
            input_id: Unique ID for the input
            inputs: Dictionary mapping input names to numpy arrays
            callback: Callback function to call with the results

        Returns:
            True if the input was added, False otherwise
        """
        if not self.running:
            logger.warning("Distributed inference engine is not running")
            return False

        try:
            # Add input to queue
            await self.input_queue.put((input_id, inputs, callback))

            # Update stats
            self.stats["total_inputs"] += 1

            return True
        except asyncio.QueueFull:
            logger.warning("Input queue is full")
            return False

    async def get_output(self, timeout: Optional[float] = None) -> Optional[Tuple[str, Dict[str, np.ndarray]]]:
        """
        Get an output from the queue.

        Args:
            timeout: Timeout for waiting for an output in seconds

        Returns:
            Tuple of (input_id, outputs) or None if no output is available
        """
        try:
            if timeout:
                return await asyncio.wait_for(self.output_queue.get(), timeout=timeout)
            else:
                return await self.output_queue.get()
        except (asyncio.QueueEmpty, asyncio.TimeoutError):
            return None

    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the distributed inference engine.

        Returns:
            Dictionary with statistics
        """
        # Get local engine stats
        local_stats = self.local_engine.get_stats()

        # Update stats
        self.stats["local_stats"] = local_stats

        return self.stats

    async def _add_worker(self, url: str) -> bool:
        """
        Add a worker to the distributed inference engine.

        Args:
            url: Worker URL

        Returns:
            True if the worker was added, False otherwise
        """
        if url in self.worker_clients:
            logger.warning(f"Worker {url} already exists")
            return False

        try:
            # Create worker client
            client = WorkerClient(url)

            # Check if worker is available
            if not await client.check_health():
                logger.warning(f"Worker {url} is not available")
                await client.close()
                return False

            # Add worker client
            self.worker_clients[url] = client

            # Add worker URL if not already in list
            if url not in self.worker_urls:
                self.worker_urls.append(url)

            # Initialize worker stats
            self.stats["worker_stats"][url] = {
                "total_inputs": 0,
                "total_outputs": 0,
                "last_input_time": None,
                "last_output_time": None
            }

            logger.info(f"Worker {url} added")

            return True

        except Exception as e:
            logger.error(f"Error adding worker {url}: {str(e)}")
            return False

    async def _worker_loop(self):
        """Worker loop for processing inputs."""
        logger.info("Worker task started")

        while self.running:
            try:
                # Get input from queue
                try:
                    input_id, inputs, callback = await asyncio.wait_for(
                        self.input_queue.get(), timeout=self.timeout
                    )
                except asyncio.TimeoutError:
                    # No inputs available
                    continue

                # Select worker
                worker = await self._select_worker()

                if worker is None:
                    # Use local engine
                    await self.local_engine.add_input(
                        input_id, inputs, lambda id, outputs: self._handle_output(id, outputs, callback)
                    )

                    # Update stats
                    self.stats["local_inputs"] += 1
                else:
                    # Use remote worker
                    await worker.run_inference(
                        input_id, inputs, lambda id, outputs: self._handle_output(id, outputs, callback)
                    )

                    # Update stats
                    self.stats["remote_inputs"] += 1
                    self.stats["worker_stats"][worker.url]["total_inputs"] += 1
                    self.stats["worker_stats"][worker.url]["last_input_time"] = time.time()

            except Exception as e:
                logger.error(f"Error in worker loop: {str(e)}")

    async def _select_worker(self) -> Optional["WorkerClient"]:
        """
        Select a worker for inference.

        Returns:
            Worker client or None to use local engine
        """
        # Check if there are any workers
        if not self.worker_clients:
            return None

        # Simple round-robin selection
        # In a real implementation, you would use a more sophisticated selection algorithm
        # based on worker load, latency, etc.

        # Get worker URLs
        worker_urls = list(self.worker_clients.keys())

        # Try each worker
        for url in worker_urls:
            worker = self.worker_clients[url]

            # Check if worker is available
            if await worker.check_health():
                return worker

        # No available workers
        return None

    async def _handle_output(self, input_id: str, outputs: Dict[str, np.ndarray], callback: Optional[Callable]):
        """
        Handle an output from a worker.

        Args:
            input_id: Input ID
            outputs: Dictionary mapping output names to numpy arrays
            callback: Callback function to call with the results
        """
        try:
            # Add output to queue
            await self.output_queue.put((input_id, outputs))

            # Update stats
            self.stats["total_outputs"] += 1

            # Call callback if provided
            if callback:
                if asyncio.iscoroutinefunction(callback):
                    await callback(input_id, outputs)
                else:
                    callback(input_id, outputs)

        except Exception as e:
            logger.error(f"Error handling output: {str(e)}")


class WorkerClient:
    """
    Worker client for the distributed inference engine.

    This class provides a client for communicating with a worker node.
    """

    def __init__(
        self,
        url: str,
        api_key: Optional[str] = None,
        verify_ssl: bool = True,
        timeout: float = 5.0
    ):
        """
        Initialize the worker client.

        Args:
            url: Worker URL
            api_key: API key for authentication
            verify_ssl: Whether to verify SSL certificates
            timeout: Timeout for requests in seconds
        """
        self.url = url
        self.api_key = api_key
        self.verify_ssl = verify_ssl
        self.timeout = timeout

        # Create SSL context
        ssl_context = None
        if not verify_ssl:
            ssl_context = ssl.create_default_context()
            ssl_context.check_hostname = False
            ssl_context.verify_mode = ssl.CERT_NONE

        # Create session
        self.session = aiohttp.ClientSession(
            headers={"X-API-Key": api_key} if api_key else None,
            connector=aiohttp.TCPConnector(ssl=ssl_context)
        )

        # Initialize callbacks
        self.callbacks = {}

    async def close(self):
        """Close the worker client."""
        await self.session.close()

    async def check_health(self) -> bool:
        """
        Check if the worker is healthy.

        Returns:
            True if the worker is healthy, False otherwise
        """
        try:
            async with self.session.get(
                f"{self.url}/health",
                timeout=self.timeout,
                raise_for_status=False
            ) as response:
                if response.status == 200:
                    return True
                else:
                    logger.warning(f"Worker {self.url} returned status {response.status}")
                    return False
        except Exception as e:
            logger.warning(f"Error checking health of worker {self.url}: {str(e)}")
            return False

    async def run_inference(
        self,
        input_id: str,
        inputs: Dict[str, np.ndarray],
        callback: Optional[Callable] = None
    ) -> bool:
        """
        Run inference on the worker.

        Args:
            input_id: Unique ID for the input
            inputs: Dictionary mapping input names to numpy arrays
            callback: Callback function to call with the results

        Returns:
            True if the inference was started, False otherwise
        """
        try:
            # Validate input ID
            if not input_id or not isinstance(input_id, str) or len(input_id) > 100:
                logger.error(f"Invalid input ID: {input_id}")
                return False

            # Validate inputs
            if not inputs or not isinstance(inputs, dict) or len(inputs) > 10:
                logger.error("Invalid inputs")
                return False

            # Store callback
            if callback:
                self.callbacks[input_id] = callback

            # Convert inputs to JSON-serializable format
            serialized_inputs = {}
            try:
                for name, array in inputs.items():
                    # Validate input name
                    if not name or not isinstance(name, str) or len(name) > 100:
                        logger.error(f"Invalid input name: {name}")
                        return False

                    # Validate array
                    if not isinstance(array, np.ndarray):
                        logger.error(f"Invalid array for input {name}")
                        return False

                    # Check array shape
                    if len(array.shape) > 5:
                        logger.error(f"Array shape too complex for input {name}")
                        return False

                    # Check array size
                    total_size = np.prod(array.shape)
                    if total_size > 10000000:  # Limit to 10M elements
                        logger.error(f"Array too large for input {name}")
                        return False

                    # Serialize array
                    serialized_inputs[name] = {
                        "shape": array.shape,
                        "dtype": str(array.dtype),
                        "data": array.tobytes().hex()
                    }
            except Exception as e:
                logger.error(f"Error serializing inputs: {str(e)}")
                return False

            # Send request
            async with self.session.post(
                f"{self.url}/inference",
                json={
                    "input_id": input_id,
                    "inputs": serialized_inputs
                },
                timeout=self.timeout,
                raise_for_status=False
            ) as response:
                if response.status == 200:
                    # Inference started successfully
                    return True
                else:
                    # Try to parse error message
                    try:
                        error_data = await response.json()
                        error_message = error_data.get("error", "Unknown error")
                        logger.warning(f"Worker {self.url} returned status {response.status}: {error_message}")
                    except Exception:
                        logger.warning(f"Worker {self.url} returned status {response.status}")
                    return False

        except Exception as e:
            logger.error(f"Error running inference on worker {self.url}: {str(e)}")
            return False

    async def handle_result(self, input_id: str, outputs: Dict[str, Any]):
        """
        Handle a result from the worker.

        Args:
            input_id: Input ID
            outputs: Dictionary mapping output names to output data
        """
        try:
            # Convert outputs to numpy arrays
            deserialized_outputs = {}
            for name, output in outputs.items():
                shape = tuple(output["shape"])
                dtype = np.dtype(output["dtype"])
                data = bytes.fromhex(output["data"])
                array = np.frombuffer(data, dtype=dtype).reshape(shape)
                deserialized_outputs[name] = array

            # Call callback if provided
            if input_id in self.callbacks:
                callback = self.callbacks[input_id]
                del self.callbacks[input_id]

                if asyncio.iscoroutinefunction(callback):
                    await callback(input_id, deserialized_outputs)
                else:
                    callback(input_id, deserialized_outputs)

        except Exception as e:
            logger.error(f"Error handling result from worker {self.url}: {str(e)}")


class WorkerServer:
    """
    Worker server for the distributed inference engine.

    This class provides a server for handling inference requests from the distributed
    inference engine.
    """

    def __init__(
        self,
        model_path: str,
        backend: str = "torch",
        device: str = "AUTO",
        batch_size: int = 4,
        max_queue_size: int = 100,
        timeout: float = 1.0,
        host: str = "0.0.0.0",
        port: int = 8070,
        api_key: Optional[str] = None,
        use_ssl: bool = False,
        ssl_cert: Optional[str] = None,
        ssl_key: Optional[str] = None
    ):
        """
        Initialize the worker server.

        Args:
            model_path: Path to the model file
            backend: Backend to use ("tinygrad", "torch", "tflite")
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
            batch_size: Maximum batch size for inference
            max_queue_size: Maximum size of the input queue
            timeout: Timeout for waiting for inputs in seconds
            host: Host to bind to
            port: Port to bind to
            api_key: API key for authentication
            use_ssl: Whether to use SSL
            ssl_cert: Path to SSL certificate
            ssl_key: Path to SSL key
        """
        self.model_path = model_path
        self.backend = backend
        self.device = device
        self.batch_size = batch_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        self.host = host
        self.port = port
        self.api_key = api_key
        self.use_ssl = use_ssl
        self.ssl_cert = ssl_cert
        self.ssl_key = ssl_key

        # Initialize engine
        self.engine = AsyncBatchInferenceEngine(
            model_path=model_path,
            backend=backend,
            device=device,
            batch_size=batch_size,
            max_queue_size=max_queue_size,
            timeout=timeout
        )

        # Initialize web server
        self.app = None
        self.runner = None
        self.site = None

        # Initialize stats
        self.stats = {
            "total_requests": 0,
            "total_inferences": 0,
            "start_time": None
        }

    async def start(self):
        """Start the worker server."""
        # Check if model file is valid
        if not os.path.exists(self.model_path):
            logger.error(f"Model file not found: {self.model_path}")
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        # Check if model file is safe
        if not is_safe_file(self.model_path):
            logger.error(f"Invalid model file: {self.model_path}")
            raise ValueError(f"Invalid model file: {self.model_path}")

        # Check if model format is supported
        if not validate_model_format(self.model_path):
            logger.error(f"Unsupported model format: {Path(self.model_path).suffix.lower()}")
            raise ValueError(f"Unsupported model format: {Path(self.model_path).suffix.lower()}")

        # Start engine
        await self.engine.start()

        # Create web server
        from aiohttp import web

        # Create middleware for authentication
        @web.middleware
        async def auth_middleware(request, handler):
            # Skip authentication for health endpoint
            if request.path == "/health":
                return await handler(request)

            # Check if API key is required
            if self.api_key:
                # Get API key from header
                api_key = request.headers.get("X-API-Key")

                # Check if API key is valid
                if not api_key or not hmac.compare_digest(api_key, self.api_key):
                    return web.json_response({"error": "Unauthorized"}, status=401)

            # Call handler
            return await handler(request)

        # Create middleware for request validation
        @web.middleware
        async def validation_middleware(request, handler):
            # Check content type for POST requests
            if request.method == "POST" and request.path == "/inference":
                content_type = request.headers.get("Content-Type", "")
                if not content_type.startswith("application/json"):
                    return web.json_response({"error": "Invalid content type"}, status=400)

                # Check content length
                content_length = request.content_length
                if content_length is not None and content_length > 10 * 1024 * 1024:  # 10MB
                    return web.json_response({"error": "Request too large"}, status=413)

            # Call handler
            return await handler(request)

        # Create middleware for rate limiting
        @web.middleware
        async def rate_limit_middleware(request, handler):
            # Simple rate limiting based on client IP
            client_ip = request.remote

            # Get current time
            current_time = time.time()

            # Check if client is rate limited
            if hasattr(self, "_rate_limit") and client_ip in self._rate_limit:
                last_request_time, request_count = self._rate_limit[client_ip]

                # Reset request count if more than 60 seconds have passed
                if current_time - last_request_time > 60:
                    self._rate_limit[client_ip] = (current_time, 1)
                # Rate limit if more than 60 requests in 60 seconds
                elif request_count > 60:
                    return web.json_response({"error": "Rate limit exceeded"}, status=429)
                # Increment request count
                else:
                    self._rate_limit[client_ip] = (last_request_time, request_count + 1)
            else:
                # Initialize rate limit for client
                if not hasattr(self, "_rate_limit"):
                    self._rate_limit = {}
                self._rate_limit[client_ip] = (current_time, 1)

            # Call handler
            return await handler(request)

        # Create application with middleware
        self.app = web.Application(middlewares=[
            auth_middleware,
            validation_middleware,
            rate_limit_middleware
        ])

        # Add routes
        self.app.add_routes([
            web.get("/health", self.health_handler),
            web.post("/inference", self.inference_handler),
            web.get("/stats", self.stats_handler)
        ])

        # Start web server
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()

        # Create SSL context if needed
        ssl_context = None
        if self.use_ssl and self.ssl_cert and self.ssl_key:
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(self.ssl_cert, self.ssl_key)

        # Create site
        if ssl_context:
            self.site = web.TCPSite(self.runner, self.host, self.port, ssl_context=ssl_context)
            protocol = "https"
        else:
            self.site = web.TCPSite(self.runner, self.host, self.port)
            protocol = "http"

        # Start site
        await self.site.start()

        # Update stats
        self.stats["start_time"] = time.time()

        logger.info(f"Worker server started on {protocol}://{self.host}:{self.port}")

    async def stop(self):
        """Stop the worker server."""
        # Stop web server
        if self.site:
            await self.site.stop()

        if self.runner:
            await self.runner.cleanup()

        # Stop engine
        await self.engine.stop()

        logger.info("Worker server stopped")

    async def health_handler(self, request):
        """
        Handle health check requests.

        Args:
            request: Web request

        Returns:
            Web response
        """
        from aiohttp import web

        return web.json_response({
            "status": "healthy",
            "backend": self.backend,
            "device": self.device
        })

    async def inference_handler(self, request):
        """
        Handle inference requests.

        Args:
            request: Web request

        Returns:
            Web response
        """
        from aiohttp import web

        try:
            # Parse request
            try:
                data = await request.json()
            except Exception:
                return web.json_response({"error": "Invalid JSON"}, status=400)

            # Get input ID
            input_id = data.get("input_id")
            if not input_id:
                return web.json_response({"error": "Missing input_id"}, status=400)

            # Validate input ID
            if not isinstance(input_id, str) or len(input_id) > 100:
                return web.json_response({"error": "Invalid input_id"}, status=400)

            # Get inputs
            serialized_inputs = data.get("inputs")
            if not serialized_inputs:
                return web.json_response({"error": "Missing inputs"}, status=400)

            # Validate inputs
            if not isinstance(serialized_inputs, dict) or len(serialized_inputs) > 10:
                return web.json_response({"error": "Invalid inputs"}, status=400)

            # Deserialize inputs
            inputs = {}
            try:
                for name, input_data in serialized_inputs.items():
                    # Validate input name
                    if not isinstance(name, str) or len(name) > 100:
                        return web.json_response({"error": f"Invalid input name: {name}"}, status=400)

                    # Validate input data
                    if not isinstance(input_data, dict):
                        return web.json_response({"error": f"Invalid input data for {name}"}, status=400)

                    # Validate shape
                    shape = input_data.get("shape")
                    if not shape or not isinstance(shape, list):
                        return web.json_response({"error": f"Invalid shape for {name}"}, status=400)

                    # Validate dtype
                    dtype_str = input_data.get("dtype")
                    if not dtype_str or not isinstance(dtype_str, str):
                        return web.json_response({"error": f"Invalid dtype for {name}"}, status=400)

                    # Validate data
                    data_hex = input_data.get("data")
                    if not data_hex or not isinstance(data_hex, str):
                        return web.json_response({"error": f"Invalid data for {name}"}, status=400)

                    # Convert shape to tuple
                    shape = tuple(shape)

                    # Check shape size
                    if len(shape) > 5 or any(dim <= 0 for dim in shape):
                        return web.json_response({"error": f"Invalid shape dimensions for {name}"}, status=400)

                    # Check total size
                    total_size = 1
                    for dim in shape:
                        total_size *= dim
                    if total_size > 10000000:  # Limit to 10M elements
                        return web.json_response({"error": f"Input {name} is too large"}, status=400)

                    # Parse dtype
                    try:
                        dtype = np.dtype(dtype_str)
                    except Exception:
                        return web.json_response({"error": f"Invalid dtype for {name}: {dtype_str}"}, status=400)

                    # Parse data
                    try:
                        data = bytes.fromhex(data_hex)
                    except Exception:
                        return web.json_response({"error": f"Invalid data for {name}"}, status=400)

                    # Check data size
                    expected_size = total_size * dtype.itemsize
                    if len(data) != expected_size:
                        return web.json_response({"error": f"Data size mismatch for {name}"}, status=400)

                    # Create array
                    try:
                        array = np.frombuffer(data, dtype=dtype).reshape(shape)
                        inputs[name] = array
                    except Exception as e:
                        return web.json_response({"error": f"Error creating array for {name}: {str(e)}"}, status=400)
            except Exception as e:
                return web.json_response({"error": f"Error deserializing inputs: {str(e)}"}, status=400)

            # Update stats
            self.stats["total_requests"] += 1

            # Run inference
            await self.engine.add_input(input_id, inputs, self._handle_result)

            return web.json_response({"status": "accepted"})

        except Exception as e:
            # Log detailed error for administrators
            logger.error(f"Error handling inference request: {str(e)}", exc_info=True)
            # Return generic error for users
            return web.json_response({"error": "An error occurred while processing the request"}, status=500)

    async def stats_handler(self, request):
        """
        Handle stats requests.

        Args:
            request: Web request

        Returns:
            Web response
        """
        from aiohttp import web

        # Get engine stats
        engine_stats = self.engine.get_stats()

        # Combine stats
        combined_stats = {
            **self.stats,
            "engine_stats": engine_stats
        }

        return web.json_response(combined_stats)

    async def _handle_result(self, input_id: str, outputs: Dict[str, np.ndarray]):
        """
        Handle a result from the engine.

        Args:
            input_id: Input ID
            outputs: Dictionary mapping output names to numpy arrays
        """
        try:
            # Update stats
            self.stats["total_inferences"] += 1

            # Serialize outputs
            serialized_outputs = {}
            for name, array in outputs.items():
                serialized_outputs[name] = {
                    "shape": array.shape,
                    "dtype": str(array.dtype),
                    "data": array.tobytes().hex()
                }

            # Send result to client
            # In a real implementation, you would send the result back to the client
            # using a callback URL or a message queue

            logger.debug(f"Inference result for {input_id}: {len(outputs)} outputs")

        except Exception as e:
            logger.error(f"Error handling result: {str(e)}")
