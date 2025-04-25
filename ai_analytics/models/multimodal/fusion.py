"""
Fusion models for multimodal detection.

This module provides models for fusing data from multiple modalities
(visual, thermal, etc.) for improved object detection.
"""

import logging
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Union

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FeatureFusion(nn.Module):
    """
    Feature-level fusion model for multimodal detection.
    
    This model fuses features from multiple modalities at the feature level,
    before making detection decisions.
    """
    
    def __init__(
        self,
        input_dims: Dict[str, int],
        fusion_dim: int = 256,
        dropout: float = 0.3
    ):
        """
        Initialize the feature fusion model.
        
        Args:
            input_dims: Dictionary mapping modality names to feature dimensions
            fusion_dim: Dimension of the fused features
            dropout: Dropout probability
        """
        super().__init__()
        
        self.input_dims = input_dims
        self.fusion_dim = fusion_dim
        
        # Create projection layers for each modality
        self.projections = nn.ModuleDict()
        for modality, dim in input_dims.items():
            self.projections[modality] = nn.Sequential(
                nn.Linear(dim, fusion_dim),
                nn.BatchNorm1d(fusion_dim),
                nn.ReLU(),
                nn.Dropout(dropout)
            )
        
        # Attention mechanism for adaptive fusion
        self.attention = nn.Sequential(
            nn.Linear(fusion_dim, fusion_dim // 2),
            nn.ReLU(),
            nn.Linear(fusion_dim // 2, 1),
            nn.Sigmoid()
        )
    
    def forward(self, inputs: Dict[str, torch.Tensor]) -> torch.Tensor:
        """
        Forward pass of the feature fusion model.
        
        Args:
            inputs: Dictionary mapping modality names to feature tensors
            
        Returns:
            Fused feature tensor
        """
        # Project features from each modality
        projected_features = {}
        for modality, features in inputs.items():
            if modality in self.projections:
                projected_features[modality] = self.projections[modality](features)
        
        # If only one modality is available, return its features
        if len(projected_features) == 1:
            return next(iter(projected_features.values()))
        
        # Compute attention weights for each modality
        attention_weights = {}
        for modality, features in projected_features.items():
            attention_weights[modality] = self.attention(features)
        
        # Normalize attention weights
        total_weight = sum(attention_weights.values())
        normalized_weights = {m: w / total_weight for m, w in attention_weights.items()}
        
        # Weighted sum of features
        fused_features = sum(features * normalized_weights[modality] 
                            for modality, features in projected_features.items())
        
        return fused_features


class DecisionFusion(nn.Module):
    """
    Decision-level fusion model for multimodal detection.
    
    This model makes detection decisions for each modality separately,
    then fuses the decisions.
    """
    
    def __init__(
        self,
        num_classes: int,
        fusion_method: str = "weighted_average",
        weights: Optional[Dict[str, float]] = None
    ):
        """
        Initialize the decision fusion model.
        
        Args:
            num_classes: Number of object classes
            fusion_method: Method for fusing decisions ("weighted_average", "max", "bayesian")
            weights: Dictionary mapping modality names to weights (for weighted_average)
        """
        super().__init__()
        
        self.num_classes = num_classes
        self.fusion_method = fusion_method
        self.weights = weights or {}
    
    def forward(self, decisions: Dict[str, torch.Tensor]) -> torch.Tensor:
        """
        Forward pass of the decision fusion model.
        
        Args:
            decisions: Dictionary mapping modality names to decision tensors
                      (class probabilities or logits)
            
        Returns:
            Fused decision tensor
        """
        # Ensure all decisions are probabilities (apply softmax if needed)
        probs = {}
        for modality, decision in decisions.items():
            if decision.dim() == 2 and decision.size(1) == self.num_classes:
                # If logits, convert to probabilities
                probs[modality] = F.softmax(decision, dim=1)
            else:
                probs[modality] = decision
        
        # If only one modality is available, return its decisions
        if len(probs) == 1:
            return next(iter(probs.values()))
        
        # Fusion based on selected method
        if self.fusion_method == "weighted_average":
            # Use provided weights or equal weights
            weights = {}
            for modality in probs.keys():
                weights[modality] = self.weights.get(modality, 1.0)
            
            # Normalize weights
            total_weight = sum(weights.values())
            normalized_weights = {m: w / total_weight for m, w in weights.items()}
            
            # Weighted average of probabilities
            fused_probs = sum(prob * normalized_weights[modality] 
                             for modality, prob in probs.items())
        
        elif self.fusion_method == "max":
            # Take maximum probability for each class
            fused_probs = torch.stack(list(probs.values())).max(dim=0)[0]
        
        elif self.fusion_method == "bayesian":
            # Bayesian fusion (product of probabilities, normalized)
            fused_probs = torch.ones_like(next(iter(probs.values())))
            for prob in probs.values():
                fused_probs *= prob
            
            # Normalize to ensure valid probability distribution
            fused_probs /= fused_probs.sum(dim=1, keepdim=True)
        
        else:
            raise ValueError(f"Unknown fusion method: {self.fusion_method}")
        
        return fused_probs


class HybridFusion(nn.Module):
    """
    Hybrid fusion model for multimodal detection.
    
    This model combines feature-level and decision-level fusion
    for improved detection performance.
    """
    
    def __init__(
        self,
        input_dims: Dict[str, int],
        num_classes: int,
        fusion_dim: int = 256,
        dropout: float = 0.3,
        decision_fusion_method: str = "weighted_average"
    ):
        """
        Initialize the hybrid fusion model.
        
        Args:
            input_dims: Dictionary mapping modality names to feature dimensions
            num_classes: Number of object classes
            fusion_dim: Dimension of the fused features
            dropout: Dropout probability
            decision_fusion_method: Method for fusing decisions
        """
        super().__init__()
        
        # Feature-level fusion
        self.feature_fusion = FeatureFusion(
            input_dims=input_dims,
            fusion_dim=fusion_dim,
            dropout=dropout
        )
        
        # Classifiers for each modality
        self.classifiers = nn.ModuleDict()
        for modality in input_dims.keys():
            self.classifiers[modality] = nn.Sequential(
                nn.Linear(fusion_dim, fusion_dim // 2),
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(fusion_dim // 2, num_classes)
            )
        
        # Classifier for fused features
        self.fusion_classifier = nn.Sequential(
            nn.Linear(fusion_dim, fusion_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(fusion_dim // 2, num_classes)
        )
        
        # Decision-level fusion
        self.decision_fusion = DecisionFusion(
            num_classes=num_classes,
            fusion_method=decision_fusion_method
        )
    
    def forward(
        self,
        inputs: Dict[str, torch.Tensor],
        return_individual: bool = False
    ) -> Union[torch.Tensor, Tuple[torch.Tensor, Dict[str, torch.Tensor]]]:
        """
        Forward pass of the hybrid fusion model.
        
        Args:
            inputs: Dictionary mapping modality names to feature tensors
            return_individual: Whether to return individual modality decisions
            
        Returns:
            Fused decision tensor, or tuple of (fused_decision, individual_decisions)
        """
        # Project features from each modality
        projected_features = {}
        for modality, features in inputs.items():
            if modality in self.classifiers:
                projected_features[modality] = self.feature_fusion.projections[modality](features)
        
        # Fuse features
        fused_features = self.feature_fusion(inputs)
        
        # Get decisions for each modality
        individual_decisions = {}
        for modality, features in projected_features.items():
            individual_decisions[modality] = self.classifiers[modality](features)
        
        # Get decision for fused features
        fusion_decision = self.fusion_classifier(fused_features)
        
        # Add fusion decision to individual decisions
        all_decisions = {**individual_decisions, "fusion": fusion_decision}
        
        # Fuse decisions
        final_decision = self.decision_fusion(all_decisions)
        
        if return_individual:
            return final_decision, individual_decisions
        else:
            return final_decision
