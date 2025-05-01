"""
Batch inference engine for Bulo.Cloud Sentinel.

This module provides a batch inference engine for running inference on multiple inputs
with different ML backends.
"""

import logging
import time
import asyncio
import threading
import queue
from typing import Dict, Any, Optional, List, Tuple, Callable, Union
import numpy as np
from pathlib import Path

from .engine import InferenceEngine

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BatchInferenceEngine:
    """
    Batch inference engine for Bulo.Cloud Sentinel.
    
    This class provides a batch inference engine for running inference on multiple inputs
    with different ML backends. It supports batching, streaming, and asynchronous inference.
    """
    
    def __init__(
        self,
        model_path: str,
        backend: str = "torch",
        device: str = "AUTO",
        batch_size: int = 4,
        max_queue_size: int = 100,
        timeout: float = 1.0
    ):
        """
        Initialize the batch inference engine.
        
        Args:
            model_path: Path to the model file
            backend: Backend to use ("tinygrad", "torch", "tflite")
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
            batch_size: Maximum batch size for inference
            max_queue_size: Maximum size of the input queue
            timeout: Timeout for waiting for inputs in seconds
        """
        self.model_path = model_path
        self.backend = backend
        self.device = device
        self.batch_size = batch_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        
        # Initialize engine
        self.engine = InferenceEngine(backend=backend, model_path=model_path, device=device)
        
        # Initialize queues
        self.input_queue = queue.Queue(maxsize=max_queue_size)
        self.output_queue = queue.Queue()
        
        # Initialize worker thread
        self.worker_thread = None
        self.running = False
        
        # Initialize stats
        self.stats = {
            "total_inputs": 0,
            "total_batches": 0,
            "total_inference_time": 0.0,
            "avg_batch_size": 0.0,
            "avg_inference_time": 0.0,
            "max_inference_time": 0.0,
            "min_inference_time": float("inf")
        }
    
    def start(self):
        """Start the batch inference engine."""
        if self.running:
            logger.warning("Batch inference engine is already running")
            return
        
        # Set running flag
        self.running = True
        
        # Start worker thread
        self.worker_thread = threading.Thread(target=self._worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        logger.info(f"Batch inference engine started with {self.backend} backend on {self.device}")
    
    def stop(self):
        """Stop the batch inference engine."""
        if not self.running:
            logger.warning("Batch inference engine is not running")
            return
        
        # Clear running flag
        self.running = False
        
        # Wait for worker thread to finish
        if self.worker_thread:
            self.worker_thread.join(timeout=self.timeout)
        
        # Clear queues
        while not self.input_queue.empty():
            try:
                self.input_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.output_queue.empty():
            try:
                self.output_queue.get_nowait()
            except queue.Empty:
                break
        
        logger.info("Batch inference engine stopped")
    
    def add_input(self, input_id: str, inputs: Dict[str, np.ndarray], callback: Optional[Callable] = None) -> bool:
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
            logger.warning("Batch inference engine is not running")
            return False
        
        try:
            # Add input to queue
            self.input_queue.put((input_id, inputs, callback), block=False)
            return True
        except queue.Full:
            logger.warning("Input queue is full")
            return False
    
    def get_output(self, block: bool = True, timeout: Optional[float] = None) -> Optional[Tuple[str, Dict[str, np.ndarray]]]:
        """
        Get an output from the queue.
        
        Args:
            block: Whether to block until an output is available
            timeout: Timeout for waiting for an output in seconds
            
        Returns:
            Tuple of (input_id, outputs) or None if no output is available
        """
        try:
            return self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the batch inference engine.
        
        Returns:
            Dictionary with statistics
        """
        return self.stats
    
    def _worker_loop(self):
        """Worker loop for processing inputs."""
        logger.info("Worker thread started")
        
        while self.running:
            try:
                # Collect inputs for a batch
                batch_inputs = []
                batch_callbacks = {}
                
                # Get first input
                try:
                    input_id, inputs, callback = self.input_queue.get(block=True, timeout=self.timeout)
                    batch_inputs.append((input_id, inputs))
                    if callback:
                        batch_callbacks[input_id] = callback
                except queue.Empty:
                    # No inputs available
                    continue
                
                # Get more inputs if available
                while len(batch_inputs) < self.batch_size:
                    try:
                        input_id, inputs, callback = self.input_queue.get(block=False)
                        batch_inputs.append((input_id, inputs))
                        if callback:
                            batch_callbacks[input_id] = callback
                    except queue.Empty:
                        # No more inputs available
                        break
                
                # Process batch
                if batch_inputs:
                    self._process_batch(batch_inputs, batch_callbacks)
            
            except Exception as e:
                logger.error(f"Error in worker loop: {str(e)}")
    
    def _process_batch(self, batch_inputs: List[Tuple[str, Dict[str, np.ndarray]]], batch_callbacks: Dict[str, Callable]):
        """
        Process a batch of inputs.
        
        Args:
            batch_inputs: List of (input_id, inputs) tuples
            batch_callbacks: Dictionary mapping input IDs to callback functions
        """
        try:
            # Check if batch is empty
            if not batch_inputs:
                return
            
            # Update stats
            self.stats["total_inputs"] += len(batch_inputs)
            self.stats["total_batches"] += 1
            self.stats["avg_batch_size"] = self.stats["total_inputs"] / self.stats["total_batches"]
            
            # Check if batch has only one input
            if len(batch_inputs) == 1:
                # Process single input
                input_id, inputs = batch_inputs[0]
                
                # Run inference
                start_time = time.time()
                outputs = self.engine.predict(inputs)
                inference_time = (time.time() - start_time) * 1000  # Convert to ms
                
                # Update stats
                self.stats["total_inference_time"] += inference_time
                self.stats["avg_inference_time"] = self.stats["total_inference_time"] / self.stats["total_batches"]
                self.stats["max_inference_time"] = max(self.stats["max_inference_time"], inference_time)
                self.stats["min_inference_time"] = min(self.stats["min_inference_time"], inference_time)
                
                # Add output to queue
                self.output_queue.put((input_id, outputs))
                
                # Call callback if provided
                if input_id in batch_callbacks:
                    batch_callbacks[input_id](input_id, outputs)
                
                return
            
            # Process batch
            # This is a simplified implementation that processes inputs sequentially
            # In a real implementation, you would batch the inputs and run inference once
            
            start_time = time.time()
            
            for input_id, inputs in batch_inputs:
                # Run inference
                outputs = self.engine.predict(inputs)
                
                # Add output to queue
                self.output_queue.put((input_id, outputs))
                
                # Call callback if provided
                if input_id in batch_callbacks:
                    batch_callbacks[input_id](input_id, outputs)
            
            inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            # Update stats
            self.stats["total_inference_time"] += inference_time
            self.stats["avg_inference_time"] = self.stats["total_inference_time"] / self.stats["total_batches"]
            self.stats["max_inference_time"] = max(self.stats["max_inference_time"], inference_time)
            self.stats["min_inference_time"] = min(self.stats["min_inference_time"], inference_time)
        
        except Exception as e:
            logger.error(f"Error processing batch: {str(e)}")


class AsyncBatchInferenceEngine:
    """
    Asynchronous batch inference engine for Bulo.Cloud Sentinel.
    
    This class provides an asynchronous batch inference engine for running inference
    on multiple inputs with different ML backends. It supports batching, streaming,
    and asynchronous inference.
    """
    
    def __init__(
        self,
        model_path: str,
        backend: str = "torch",
        device: str = "AUTO",
        batch_size: int = 4,
        max_queue_size: int = 100,
        timeout: float = 1.0
    ):
        """
        Initialize the asynchronous batch inference engine.
        
        Args:
            model_path: Path to the model file
            backend: Backend to use ("tinygrad", "torch", "tflite")
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
            batch_size: Maximum batch size for inference
            max_queue_size: Maximum size of the input queue
            timeout: Timeout for waiting for inputs in seconds
        """
        self.model_path = model_path
        self.backend = backend
        self.device = device
        self.batch_size = batch_size
        self.max_queue_size = max_queue_size
        self.timeout = timeout
        
        # Initialize engine
        self.engine = InferenceEngine(backend=backend, model_path=model_path, device=device)
        
        # Initialize queues
        self.input_queue = asyncio.Queue(maxsize=max_queue_size)
        self.output_queue = asyncio.Queue()
        
        # Initialize worker task
        self.worker_task = None
        self.running = False
        
        # Initialize stats
        self.stats = {
            "total_inputs": 0,
            "total_batches": 0,
            "total_inference_time": 0.0,
            "avg_batch_size": 0.0,
            "avg_inference_time": 0.0,
            "max_inference_time": 0.0,
            "min_inference_time": float("inf")
        }
    
    async def start(self):
        """Start the asynchronous batch inference engine."""
        if self.running:
            logger.warning("Asynchronous batch inference engine is already running")
            return
        
        # Set running flag
        self.running = True
        
        # Start worker task
        self.worker_task = asyncio.create_task(self._worker_loop())
        
        logger.info(f"Asynchronous batch inference engine started with {self.backend} backend on {self.device}")
    
    async def stop(self):
        """Stop the asynchronous batch inference engine."""
        if not self.running:
            logger.warning("Asynchronous batch inference engine is not running")
            return
        
        # Clear running flag
        self.running = False
        
        # Wait for worker task to finish
        if self.worker_task:
            try:
                await asyncio.wait_for(self.worker_task, timeout=self.timeout)
            except asyncio.TimeoutError:
                logger.warning("Timeout waiting for worker task to finish")
        
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
        
        logger.info("Asynchronous batch inference engine stopped")
    
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
            logger.warning("Asynchronous batch inference engine is not running")
            return False
        
        try:
            # Add input to queue
            await self.input_queue.put((input_id, inputs, callback))
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
        Get statistics about the asynchronous batch inference engine.
        
        Returns:
            Dictionary with statistics
        """
        return self.stats
    
    async def _worker_loop(self):
        """Worker loop for processing inputs."""
        logger.info("Worker task started")
        
        while self.running:
            try:
                # Collect inputs for a batch
                batch_inputs = []
                batch_callbacks = {}
                
                # Get first input
                try:
                    input_id, inputs, callback = await asyncio.wait_for(
                        self.input_queue.get(), timeout=self.timeout
                    )
                    batch_inputs.append((input_id, inputs))
                    if callback:
                        batch_callbacks[input_id] = callback
                except asyncio.TimeoutError:
                    # No inputs available
                    continue
                
                # Get more inputs if available
                while len(batch_inputs) < self.batch_size:
                    try:
                        input_id, inputs, callback = self.input_queue.get_nowait()
                        batch_inputs.append((input_id, inputs))
                        if callback:
                            batch_callbacks[input_id] = callback
                    except asyncio.QueueEmpty:
                        # No more inputs available
                        break
                
                # Process batch
                if batch_inputs:
                    await self._process_batch(batch_inputs, batch_callbacks)
            
            except Exception as e:
                logger.error(f"Error in worker loop: {str(e)}")
    
    async def _process_batch(self, batch_inputs: List[Tuple[str, Dict[str, np.ndarray]]], batch_callbacks: Dict[str, Callable]):
        """
        Process a batch of inputs.
        
        Args:
            batch_inputs: List of (input_id, inputs) tuples
            batch_callbacks: Dictionary mapping input IDs to callback functions
        """
        try:
            # Check if batch is empty
            if not batch_inputs:
                return
            
            # Update stats
            self.stats["total_inputs"] += len(batch_inputs)
            self.stats["total_batches"] += 1
            self.stats["avg_batch_size"] = self.stats["total_inputs"] / self.stats["total_batches"]
            
            # Check if batch has only one input
            if len(batch_inputs) == 1:
                # Process single input
                input_id, inputs = batch_inputs[0]
                
                # Run inference
                start_time = time.time()
                outputs = self.engine.predict(inputs)
                inference_time = (time.time() - start_time) * 1000  # Convert to ms
                
                # Update stats
                self.stats["total_inference_time"] += inference_time
                self.stats["avg_inference_time"] = self.stats["total_inference_time"] / self.stats["total_batches"]
                self.stats["max_inference_time"] = max(self.stats["max_inference_time"], inference_time)
                self.stats["min_inference_time"] = min(self.stats["min_inference_time"], inference_time)
                
                # Add output to queue
                await self.output_queue.put((input_id, outputs))
                
                # Call callback if provided
                if input_id in batch_callbacks:
                    if asyncio.iscoroutinefunction(batch_callbacks[input_id]):
                        await batch_callbacks[input_id](input_id, outputs)
                    else:
                        batch_callbacks[input_id](input_id, outputs)
                
                return
            
            # Process batch
            # This is a simplified implementation that processes inputs sequentially
            # In a real implementation, you would batch the inputs and run inference once
            
            start_time = time.time()
            
            for input_id, inputs in batch_inputs:
                # Run inference
                outputs = self.engine.predict(inputs)
                
                # Add output to queue
                await self.output_queue.put((input_id, outputs))
                
                # Call callback if provided
                if input_id in batch_callbacks:
                    if asyncio.iscoroutinefunction(batch_callbacks[input_id]):
                        await batch_callbacks[input_id](input_id, outputs)
                    else:
                        batch_callbacks[input_id](input_id, outputs)
            
            inference_time = (time.time() - start_time) * 1000  # Convert to ms
            
            # Update stats
            self.stats["total_inference_time"] += inference_time
            self.stats["avg_inference_time"] = self.stats["total_inference_time"] / self.stats["total_batches"]
            self.stats["max_inference_time"] = max(self.stats["max_inference_time"], inference_time)
            self.stats["min_inference_time"] = min(self.stats["min_inference_time"], inference_time)
        
        except Exception as e:
            logger.error(f"Error processing batch: {str(e)}")


class StreamingInferenceEngine:
    """
    Streaming inference engine for Bulo.Cloud Sentinel.
    
    This class provides a streaming inference engine for running inference on a stream
    of inputs with different ML backends. It supports batching and asynchronous inference.
    """
    
    def __init__(
        self,
        model_path: str,
        backend: str = "torch",
        device: str = "AUTO",
        batch_size: int = 4,
        max_queue_size: int = 100,
        timeout: float = 1.0
    ):
        """
        Initialize the streaming inference engine.
        
        Args:
            model_path: Path to the model file
            backend: Backend to use ("tinygrad", "torch", "tflite")
            device: Device to run inference on ("CPU", "CUDA", "OCL", "AUTO")
            batch_size: Maximum batch size for inference
            max_queue_size: Maximum size of the input queue
            timeout: Timeout for waiting for inputs in seconds
        """
        # Initialize batch inference engine
        self.batch_engine = AsyncBatchInferenceEngine(
            model_path=model_path,
            backend=backend,
            device=device,
            batch_size=batch_size,
            max_queue_size=max_queue_size,
            timeout=timeout
        )
        
        # Initialize stream handlers
        self.stream_handlers = {}
    
    async def start(self):
        """Start the streaming inference engine."""
        await self.batch_engine.start()
    
    async def stop(self):
        """Stop the streaming inference engine."""
        await self.batch_engine.stop()
    
    async def start_stream(
        self,
        stream_id: str,
        input_generator: Callable[[], Dict[str, np.ndarray]],
        output_handler: Callable[[Dict[str, np.ndarray]], None],
        interval: float = 0.1
    ) -> bool:
        """
        Start a stream.
        
        Args:
            stream_id: Unique ID for the stream
            input_generator: Function that generates inputs for inference
            output_handler: Function that handles inference outputs
            interval: Interval between inference runs in seconds
            
        Returns:
            True if the stream was started, False otherwise
        """
        if stream_id in self.stream_handlers:
            logger.warning(f"Stream {stream_id} already exists")
            return False
        
        # Create stream handler
        handler = StreamHandler(
            stream_id=stream_id,
            batch_engine=self.batch_engine,
            input_generator=input_generator,
            output_handler=output_handler,
            interval=interval
        )
        
        # Start stream handler
        await handler.start()
        
        # Store stream handler
        self.stream_handlers[stream_id] = handler
        
        return True
    
    async def stop_stream(self, stream_id: str) -> bool:
        """
        Stop a stream.
        
        Args:
            stream_id: Unique ID for the stream
            
        Returns:
            True if the stream was stopped, False otherwise
        """
        if stream_id not in self.stream_handlers:
            logger.warning(f"Stream {stream_id} not found")
            return False
        
        # Get stream handler
        handler = self.stream_handlers[stream_id]
        
        # Stop stream handler
        await handler.stop()
        
        # Remove stream handler
        del self.stream_handlers[stream_id]
        
        return True
    
    def get_stream_ids(self) -> List[str]:
        """
        Get the IDs of active streams.
        
        Returns:
            List of stream IDs
        """
        return list(self.stream_handlers.keys())
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the streaming inference engine.
        
        Returns:
            Dictionary with statistics
        """
        stats = self.batch_engine.get_stats()
        
        # Add stream stats
        stream_stats = {}
        for stream_id, handler in self.stream_handlers.items():
            stream_stats[stream_id] = handler.get_stats()
        
        stats["streams"] = stream_stats
        
        return stats


class StreamHandler:
    """
    Stream handler for the streaming inference engine.
    
    This class handles a stream of inputs and outputs for the streaming inference engine.
    """
    
    def __init__(
        self,
        stream_id: str,
        batch_engine: AsyncBatchInferenceEngine,
        input_generator: Callable[[], Dict[str, np.ndarray]],
        output_handler: Callable[[Dict[str, np.ndarray]], None],
        interval: float = 0.1
    ):
        """
        Initialize the stream handler.
        
        Args:
            stream_id: Unique ID for the stream
            batch_engine: Batch inference engine
            input_generator: Function that generates inputs for inference
            output_handler: Function that handles inference outputs
            interval: Interval between inference runs in seconds
        """
        self.stream_id = stream_id
        self.batch_engine = batch_engine
        self.input_generator = input_generator
        self.output_handler = output_handler
        self.interval = interval
        
        # Initialize worker task
        self.worker_task = None
        self.running = False
        
        # Initialize stats
        self.stats = {
            "total_inputs": 0,
            "total_outputs": 0,
            "start_time": None,
            "last_input_time": None,
            "last_output_time": None
        }
    
    async def start(self):
        """Start the stream handler."""
        if self.running:
            logger.warning(f"Stream handler {self.stream_id} is already running")
            return
        
        # Set running flag
        self.running = True
        
        # Start worker task
        self.worker_task = asyncio.create_task(self._worker_loop())
        
        # Update stats
        self.stats["start_time"] = time.time()
        
        logger.info(f"Stream handler {self.stream_id} started")
    
    async def stop(self):
        """Stop the stream handler."""
        if not self.running:
            logger.warning(f"Stream handler {self.stream_id} is not running")
            return
        
        # Clear running flag
        self.running = False
        
        # Wait for worker task to finish
        if self.worker_task:
            try:
                await asyncio.wait_for(self.worker_task, timeout=1.0)
            except asyncio.TimeoutError:
                logger.warning(f"Timeout waiting for stream handler {self.stream_id} to finish")
        
        logger.info(f"Stream handler {self.stream_id} stopped")
    
    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the stream handler.
        
        Returns:
            Dictionary with statistics
        """
        return self.stats
    
    async def _worker_loop(self):
        """Worker loop for processing inputs and outputs."""
        logger.info(f"Stream handler {self.stream_id} worker task started")
        
        while self.running:
            try:
                # Generate input
                input_data = self.input_generator()
                
                if input_data is None:
                    # No input available
                    await asyncio.sleep(self.interval)
                    continue
                
                # Update stats
                self.stats["total_inputs"] += 1
                self.stats["last_input_time"] = time.time()
                
                # Add input to batch engine
                input_id = f"{self.stream_id}_{self.stats['total_inputs']}"
                await self.batch_engine.add_input(input_id, input_data, self._handle_output)
                
                # Wait for next interval
                await asyncio.sleep(self.interval)
            
            except Exception as e:
                logger.error(f"Error in stream handler {self.stream_id} worker loop: {str(e)}")
                await asyncio.sleep(self.interval)
    
    async def _handle_output(self, input_id: str, outputs: Dict[str, np.ndarray]):
        """
        Handle an output from the batch engine.
        
        Args:
            input_id: Input ID
            outputs: Dictionary mapping output names to numpy arrays
        """
        try:
            # Update stats
            self.stats["total_outputs"] += 1
            self.stats["last_output_time"] = time.time()
            
            # Call output handler
            if asyncio.iscoroutinefunction(self.output_handler):
                await self.output_handler(outputs)
            else:
                self.output_handler(outputs)
        
        except Exception as e:
            logger.error(f"Error handling output in stream handler {self.stream_id}: {str(e)}")
