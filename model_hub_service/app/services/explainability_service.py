"""
Explainability service for the Model Hub.

This module provides a service for explaining model predictions.
"""

import os
import logging
import json
import tempfile
import base64
from typing import Dict, List, Any, Optional, Tuple, Union
from datetime import datetime
import io

import numpy as np

# Setup logging
logger = logging.getLogger(__name__)

class ExplainabilityService:
    """Service for explaining model predictions."""
    
    def __init__(self):
        """Initialize the explainability service."""
        # Check if explainability libraries are available
        self.shap_available = self._check_shap()
        self.lime_available = self._check_lime()
        self.captum_available = self._check_captum()
        
        logger.info(f"Explainability service initialized (SHAP: {self.shap_available}, LIME: {self.lime_available}, Captum: {self.captum_available})")
    
    def _check_shap(self) -> bool:
        """
        Check if SHAP is available.
        
        Returns:
            True if SHAP is available, False otherwise
        """
        try:
            import shap
            return True
        except ImportError:
            logger.warning("SHAP not available")
            return False
    
    def _check_lime(self) -> bool:
        """
        Check if LIME is available.
        
        Returns:
            True if LIME is available, False otherwise
        """
        try:
            import lime
            return True
        except ImportError:
            logger.warning("LIME not available")
            return False
    
    def _check_captum(self) -> bool:
        """
        Check if Captum is available.
        
        Returns:
            True if Captum is available, False otherwise
        """
        try:
            import captum
            return True
        except ImportError:
            logger.warning("Captum not available")
            return False
    
    async def explain_prediction(
        self,
        model_id: str,
        inputs: Any,
        method: str = "shap",
        target_class: Optional[int] = None,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model prediction.
        
        Args:
            model_id: ID of the model
            inputs: Input data
            method: Explainability method (shap, lime, captum)
            target_class: Target class for classification models
            num_samples: Number of samples for explainability methods
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        try:
            if method.lower() == "shap":
                return await self._explain_with_shap(
                    model_id=model_id,
                    inputs=inputs,
                    target_class=target_class,
                    num_samples=num_samples,
                    format=format,
                )
            elif method.lower() == "lime":
                return await self._explain_with_lime(
                    model_id=model_id,
                    inputs=inputs,
                    target_class=target_class,
                    num_samples=num_samples,
                    format=format,
                )
            elif method.lower() == "captum":
                return await self._explain_with_captum(
                    model_id=model_id,
                    inputs=inputs,
                    target_class=target_class,
                    num_samples=num_samples,
                    format=format,
                )
            else:
                logger.error(f"Unsupported explainability method: {method}")
                raise ValueError(f"Unsupported explainability method: {method}")
        except Exception as e:
            logger.error(f"Error explaining prediction: {e}")
            raise
    
    async def _explain_with_shap(
        self,
        model_id: str,
        inputs: Any,
        target_class: Optional[int] = None,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model prediction using SHAP.
        
        Args:
            model_id: ID of the model
            inputs: Input data
            target_class: Target class for classification models
            num_samples: Number of samples for SHAP
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        if not self.shap_available:
            logger.error("SHAP not available")
            raise ValueError("SHAP not available")
        
        try:
            import shap
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Create explainer
            explainer = shap.Explainer(model)
            
            # Calculate SHAP values
            shap_values = explainer(inputs, max_evals=num_samples)
            
            # Create visualization
            plt.figure(figsize=(10, 6))
            
            if target_class is not None:
                # Plot SHAP values for specific class
                shap.plots.waterfall(shap_values[0, :, target_class], show=False)
            else:
                # Plot SHAP values for all classes
                shap.plots.beeswarm(shap_values, show=False)
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining with SHAP: {e}")
            raise
    
    async def _explain_with_lime(
        self,
        model_id: str,
        inputs: Any,
        target_class: Optional[int] = None,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model prediction using LIME.
        
        Args:
            model_id: ID of the model
            inputs: Input data
            target_class: Target class for classification models
            num_samples: Number of samples for LIME
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        if not self.lime_available:
            logger.error("LIME not available")
            raise ValueError("LIME not available")
        
        try:
            import lime
            import lime.lime_tabular
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Get model metadata
            model_metadata = await self._get_model_metadata(model_id)
            
            # Create explainer
            if model_metadata.get("task") == "image":
                # Image explainer
                from lime.lime_image import LimeImageExplainer
                explainer = LimeImageExplainer()
                
                # Explain prediction
                explanation = explainer.explain_instance(
                    inputs[0],
                    model.predict,
                    top_labels=5,
                    hide_color=0,
                    num_samples=num_samples,
                )
                
                # Create visualization
                plt.figure(figsize=(10, 6))
                
                if target_class is not None:
                    # Plot explanation for specific class
                    temp, mask = explanation.get_image_and_mask(
                        target_class,
                        positive_only=False,
                        num_features=10,
                        hide_rest=False,
                    )
                    plt.imshow(lime.lime_image.mark_boundaries(temp, mask))
                else:
                    # Plot explanation for top class
                    top_class = explanation.top_labels[0]
                    temp, mask = explanation.get_image_and_mask(
                        top_class,
                        positive_only=False,
                        num_features=10,
                        hide_rest=False,
                    )
                    plt.imshow(lime.lime_image.mark_boundaries(temp, mask))
            else:
                # Tabular explainer
                explainer = lime.lime_tabular.LimeTabularExplainer(
                    np.array(inputs),
                    feature_names=model_metadata.get("feature_names", []),
                    class_names=model_metadata.get("class_names", []),
                    mode="classification" if model_metadata.get("task") == "classification" else "regression",
                )
                
                # Explain prediction
                explanation = explainer.explain_instance(
                    inputs[0],
                    model.predict,
                    num_features=10,
                    top_labels=5 if target_class is None else 1,
                )
                
                # Create visualization
                plt.figure(figsize=(10, 6))
                
                if target_class is not None:
                    # Plot explanation for specific class
                    explanation.as_pyplot_figure(label=target_class)
                else:
                    # Plot explanation for top class
                    explanation.as_pyplot_figure()
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining with LIME: {e}")
            raise
    
    async def _explain_with_captum(
        self,
        model_id: str,
        inputs: Any,
        target_class: Optional[int] = None,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model prediction using Captum.
        
        Args:
            model_id: ID of the model
            inputs: Input data
            target_class: Target class for classification models
            num_samples: Number of samples for Captum
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        if not self.captum_available:
            logger.error("Captum not available")
            raise ValueError("Captum not available")
        
        try:
            import torch
            from captum.attr import IntegratedGradients, GradientShap, DeepLift, Occlusion
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Convert inputs to tensor
            if not isinstance(inputs, torch.Tensor):
                inputs_tensor = torch.tensor(inputs)
            else:
                inputs_tensor = inputs
            
            # Create explainer
            explainer = IntegratedGradients(model)
            
            # Calculate attributions
            if target_class is not None:
                # Calculate attributions for specific class
                attributions = explainer.attribute(
                    inputs_tensor,
                    target=target_class,
                    n_steps=num_samples,
                )
            else:
                # Calculate attributions for predicted class
                outputs = model(inputs_tensor)
                predicted_class = torch.argmax(outputs, dim=1)[0].item()
                attributions = explainer.attribute(
                    inputs_tensor,
                    target=predicted_class,
                    n_steps=num_samples,
                )
            
            # Create visualization
            plt.figure(figsize=(10, 6))
            
            # Plot attributions
            plt.bar(range(attributions.shape[1]), attributions[0].detach().numpy())
            plt.xlabel("Feature")
            plt.ylabel("Attribution")
            plt.title("Feature Attributions")
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining with Captum: {e}")
            raise
    
    async def _load_model(self, model_id: str) -> Any:
        """
        Load a model.
        
        Args:
            model_id: ID of the model
            
        Returns:
            Loaded model
        """
        # In a real implementation, this would load the model from the model registry
        # For now, we'll simulate loading a model
        
        # Simulate model loading
        class DummyModel:
            def __init__(self):
                pass
            
            def predict(self, inputs):
                # Simulate prediction
                return np.random.rand(len(inputs), 10)
            
            def __call__(self, inputs):
                # Simulate prediction
                return torch.rand(len(inputs), 10)
        
        return DummyModel()
    
    async def _get_model_metadata(self, model_id: str) -> Dict[str, Any]:
        """
        Get model metadata.
        
        Args:
            model_id: ID of the model
            
        Returns:
            Model metadata
        """
        # In a real implementation, this would get the model metadata from the model registry
        # For now, we'll simulate model metadata
        
        # Simulate model metadata
        metadata = {
            "id": model_id,
            "name": f"model_{model_id}",
            "version": "1.0.0",
            "task": "classification",
            "feature_names": [f"feature_{i}" for i in range(10)],
            "class_names": [f"class_{i}" for i in range(10)],
        }
        
        return metadata
    
    async def explain_model(
        self,
        model_id: str,
        method: str = "shap",
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model globally.
        
        Args:
            model_id: ID of the model
            method: Explainability method (shap, lime, captum)
            num_samples: Number of samples for explainability methods
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        try:
            if method.lower() == "shap":
                return await self._explain_model_with_shap(
                    model_id=model_id,
                    num_samples=num_samples,
                    format=format,
                )
            elif method.lower() == "lime":
                return await self._explain_model_with_lime(
                    model_id=model_id,
                    num_samples=num_samples,
                    format=format,
                )
            elif method.lower() == "captum":
                return await self._explain_model_with_captum(
                    model_id=model_id,
                    num_samples=num_samples,
                    format=format,
                )
            else:
                logger.error(f"Unsupported explainability method: {method}")
                raise ValueError(f"Unsupported explainability method: {method}")
        except Exception as e:
            logger.error(f"Error explaining model: {e}")
            raise
    
    async def _explain_model_with_shap(
        self,
        model_id: str,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model globally using SHAP.
        
        Args:
            model_id: ID of the model
            num_samples: Number of samples for SHAP
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        if not self.shap_available:
            logger.error("SHAP not available")
            raise ValueError("SHAP not available")
        
        try:
            import shap
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Create explainer
            explainer = shap.Explainer(model)
            
            # Generate sample data
            # In a real implementation, this would use real data
            sample_data = np.random.rand(num_samples, 10)
            
            # Calculate SHAP values
            shap_values = explainer(sample_data)
            
            # Create visualization
            plt.figure(figsize=(10, 6))
            
            # Plot SHAP summary
            shap.summary_plot(shap_values, sample_data, show=False)
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining model with SHAP: {e}")
            raise
    
    async def _explain_model_with_lime(
        self,
        model_id: str,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model globally using LIME.
        
        Args:
            model_id: ID of the model
            num_samples: Number of samples for LIME
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        # LIME doesn't have a built-in global explanation method
        # We'll simulate one by explaining multiple instances and aggregating the results
        
        if not self.lime_available:
            logger.error("LIME not available")
            raise ValueError("LIME not available")
        
        try:
            import lime
            import lime.lime_tabular
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Get model metadata
            model_metadata = await self._get_model_metadata(model_id)
            
            # Generate sample data
            # In a real implementation, this would use real data
            sample_data = np.random.rand(num_samples, 10)
            
            # Create explainer
            explainer = lime.lime_tabular.LimeTabularExplainer(
                sample_data,
                feature_names=model_metadata.get("feature_names", []),
                class_names=model_metadata.get("class_names", []),
                mode="classification" if model_metadata.get("task") == "classification" else "regression",
            )
            
            # Explain multiple instances
            feature_importances = np.zeros(len(model_metadata.get("feature_names", [])))
            
            for i in range(min(10, num_samples)):
                # Explain instance
                explanation = explainer.explain_instance(
                    sample_data[i],
                    model.predict,
                    num_features=len(model_metadata.get("feature_names", [])),
                )
                
                # Get feature importances
                for feature, importance in explanation.local_exp[0]:
                    feature_importances[feature] += abs(importance)
            
            # Normalize feature importances
            feature_importances /= min(10, num_samples)
            
            # Create visualization
            plt.figure(figsize=(10, 6))
            
            # Plot feature importances
            plt.bar(model_metadata.get("feature_names", []), feature_importances)
            plt.xlabel("Feature")
            plt.ylabel("Importance")
            plt.title("Global Feature Importances")
            plt.xticks(rotation=45, ha="right")
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining model with LIME: {e}")
            raise
    
    async def _explain_model_with_captum(
        self,
        model_id: str,
        num_samples: int = 100,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Explain a model globally using Captum.
        
        Args:
            model_id: ID of the model
            num_samples: Number of samples for Captum
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (explanation data, content type)
        """
        if not self.captum_available:
            logger.error("Captum not available")
            raise ValueError("Captum not available")
        
        try:
            import torch
            from captum.attr import IntegratedGradients
            import matplotlib.pyplot as plt
            
            # Load model
            model = await self._load_model(model_id)
            
            # Get model metadata
            model_metadata = await self._get_model_metadata(model_id)
            
            # Generate sample data
            # In a real implementation, this would use real data
            sample_data = torch.rand(num_samples, 10)
            
            # Create explainer
            explainer = IntegratedGradients(model)
            
            # Calculate attributions for multiple instances
            attributions = []
            
            for i in range(min(10, num_samples)):
                # Calculate attributions
                attribution = explainer.attribute(
                    sample_data[i:i+1],
                    n_steps=10,
                )
                
                attributions.append(attribution.detach().numpy())
            
            # Aggregate attributions
            mean_attributions = np.mean(np.abs(np.vstack(attributions)), axis=0)
            
            # Create visualization
            plt.figure(figsize=(10, 6))
            
            # Plot feature importances
            plt.bar(model_metadata.get("feature_names", []), mean_attributions[0])
            plt.xlabel("Feature")
            plt.ylabel("Attribution")
            plt.title("Global Feature Attributions")
            plt.xticks(rotation=45, ha="right")
            
            # Save visualization to bytes
            buf = io.BytesIO()
            plt.savefig(buf, format=format, bbox_inches="tight")
            plt.close()
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error explaining model with Captum: {e}")
            raise
