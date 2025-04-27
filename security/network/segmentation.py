"""
Network segmentation for Bulo.Cloud Sentinel.

This module provides utilities for implementing network segmentation
to restrict communication between services.
"""

import logging
import ipaddress
from typing import Dict, List, Optional, Union, Set, Tuple
import json

# Configure logging
logger = logging.getLogger(__name__)


class NetworkPolicy:
    """
    Network policy for service communication.
    
    This class defines a network policy that specifies which services
    can communicate with each other.
    """
    
    def __init__(
        self,
        name: str,
        description: Optional[str] = None
    ):
        """
        Initialize network policy.
        
        Args:
            name: Policy name
            description: Policy description
        """
        self.name = name
        self.description = description
        self.rules: List[Dict] = []
        
        logger.info(f"Initialized network policy: {name}")
    
    def add_rule(
        self,
        source: Union[str, List[str]],
        destination: Union[str, List[str]],
        ports: Optional[Union[int, List[int]]] = None,
        protocols: Optional[Union[str, List[str]]] = None,
        action: str = "allow",
        description: Optional[str] = None
    ):
        """
        Add a rule to the policy.
        
        Args:
            source: Source service(s) or IP(s)
            destination: Destination service(s) or IP(s)
            ports: Port(s) to allow/deny
            protocols: Protocol(s) to allow/deny
            action: Action to take (allow/deny)
            description: Rule description
        """
        # Normalize inputs
        if isinstance(source, str):
            source = [source]
        
        if isinstance(destination, str):
            destination = [destination]
        
        if isinstance(ports, int):
            ports = [ports]
        
        if isinstance(protocols, str):
            protocols = [protocols]
        
        # Create rule
        rule = {
            "source": source,
            "destination": destination,
            "ports": ports,
            "protocols": protocols,
            "action": action.lower(),
            "description": description
        }
        
        # Add rule to policy
        self.rules.append(rule)
        
        logger.info(f"Added rule to policy {self.name}: {source} -> {destination}")
    
    def remove_rule(self, index: int):
        """
        Remove a rule from the policy.
        
        Args:
            index: Rule index
        """
        if 0 <= index < len(self.rules):
            rule = self.rules.pop(index)
            logger.info(f"Removed rule from policy {self.name}: {rule}")
        else:
            logger.warning(f"Invalid rule index: {index}")
    
    def is_allowed(
        self,
        source: str,
        destination: str,
        port: Optional[int] = None,
        protocol: Optional[str] = None
    ) -> bool:
        """
        Check if communication is allowed.
        
        Args:
            source: Source service or IP
            destination: Destination service or IP
            port: Port number
            protocol: Protocol
            
        Returns:
            True if allowed, False otherwise
        """
        # Default to deny if no rules match
        default_action = False
        
        # Check each rule
        for rule in self.rules:
            # Check if source matches
            source_match = False
            for src in rule["source"]:
                if src == "*" or src == source:
                    source_match = True
                    break
                
                # Check if source is an IP and matches a CIDR
                try:
                    if "/" in src and ipaddress.ip_address(source) in ipaddress.ip_network(src):
                        source_match = True
                        break
                except ValueError:
                    pass
            
            if not source_match:
                continue
            
            # Check if destination matches
            dest_match = False
            for dst in rule["destination"]:
                if dst == "*" or dst == destination:
                    dest_match = True
                    break
                
                # Check if destination is an IP and matches a CIDR
                try:
                    if "/" in dst and ipaddress.ip_address(destination) in ipaddress.ip_network(dst):
                        dest_match = True
                        break
                except ValueError:
                    pass
            
            if not dest_match:
                continue
            
            # Check if port matches
            if port is not None and rule["ports"] is not None:
                if port not in rule["ports"]:
                    continue
            
            # Check if protocol matches
            if protocol is not None and rule["protocols"] is not None:
                if protocol.lower() not in [p.lower() for p in rule["protocols"]]:
                    continue
            
            # Rule matches, return action
            return rule["action"] == "allow"
        
        # No rules matched
        return default_action
    
    def to_dict(self) -> Dict:
        """
        Convert policy to dictionary.
        
        Returns:
            Policy as dictionary
        """
        return {
            "name": self.name,
            "description": self.description,
            "rules": self.rules
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> "NetworkPolicy":
        """
        Create policy from dictionary.
        
        Args:
            data: Policy data
            
        Returns:
            Network policy
        """
        policy = cls(
            name=data["name"],
            description=data.get("description")
        )
        
        for rule in data.get("rules", []):
            policy.add_rule(
                source=rule["source"],
                destination=rule["destination"],
                ports=rule.get("ports"),
                protocols=rule.get("protocols"),
                action=rule.get("action", "allow"),
                description=rule.get("description")
            )
        
        return policy
    
    def to_json(self) -> str:
        """
        Convert policy to JSON.
        
        Returns:
            Policy as JSON string
        """
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_json(cls, json_str: str) -> "NetworkPolicy":
        """
        Create policy from JSON.
        
        Args:
            json_str: Policy as JSON string
            
        Returns:
            Network policy
        """
        data = json.loads(json_str)
        return cls.from_dict(data)


class NetworkPolicyManager:
    """
    Manager for network policies.
    
    This class manages network policies for service communication.
    """
    
    def __init__(self):
        """Initialize network policy manager."""
        self.policies: Dict[str, NetworkPolicy] = {}
        
        logger.info("Initialized network policy manager")
    
    def add_policy(self, policy: NetworkPolicy):
        """
        Add a policy.
        
        Args:
            policy: Network policy
        """
        self.policies[policy.name] = policy
        logger.info(f"Added policy: {policy.name}")
    
    def remove_policy(self, name: str):
        """
        Remove a policy.
        
        Args:
            name: Policy name
        """
        if name in self.policies:
            del self.policies[name]
            logger.info(f"Removed policy: {name}")
        else:
            logger.warning(f"Policy not found: {name}")
    
    def get_policy(self, name: str) -> Optional[NetworkPolicy]:
        """
        Get a policy.
        
        Args:
            name: Policy name
            
        Returns:
            Network policy or None if not found
        """
        return self.policies.get(name)
    
    def is_allowed(
        self,
        source: str,
        destination: str,
        port: Optional[int] = None,
        protocol: Optional[str] = None
    ) -> bool:
        """
        Check if communication is allowed by any policy.
        
        Args:
            source: Source service or IP
            destination: Destination service or IP
            port: Port number
            protocol: Protocol
            
        Returns:
            True if allowed by any policy, False otherwise
        """
        # Check each policy
        for policy in self.policies.values():
            if policy.is_allowed(source, destination, port, protocol):
                return True
        
        # No policy allowed the communication
        return False
    
    def to_dict(self) -> Dict:
        """
        Convert manager to dictionary.
        
        Returns:
            Manager as dictionary
        """
        return {
            "policies": {name: policy.to_dict() for name, policy in self.policies.items()}
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> "NetworkPolicyManager":
        """
        Create manager from dictionary.
        
        Args:
            data: Manager data
            
        Returns:
            Network policy manager
        """
        manager = cls()
        
        for name, policy_data in data.get("policies", {}).items():
            policy = NetworkPolicy.from_dict(policy_data)
            manager.add_policy(policy)
        
        return manager
    
    def to_json(self) -> str:
        """
        Convert manager to JSON.
        
        Returns:
            Manager as JSON string
        """
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_json(cls, json_str: str) -> "NetworkPolicyManager":
        """
        Create manager from JSON.
        
        Args:
            json_str: Manager as JSON string
            
        Returns:
            Network policy manager
        """
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    def save_to_file(self, file_path: str):
        """
        Save manager to file.
        
        Args:
            file_path: File path
        """
        with open(file_path, "w") as f:
            f.write(self.to_json())
        
        logger.info(f"Saved network policies to {file_path}")
    
    @classmethod
    def load_from_file(cls, file_path: str) -> "NetworkPolicyManager":
        """
        Load manager from file.
        
        Args:
            file_path: File path
            
        Returns:
            Network policy manager
        """
        with open(file_path, "r") as f:
            json_str = f.read()
        
        manager = cls.from_json(json_str)
        logger.info(f"Loaded network policies from {file_path}")
        
        return manager


# Create default network policy manager
network_policy_manager = NetworkPolicyManager()


def create_default_policies():
    """Create default network policies."""
    # Default policy for internal services
    internal_policy = NetworkPolicy(
        name="internal",
        description="Policy for internal service communication"
    )
    
    # Allow internal services to communicate with each other
    internal_policy.add_rule(
        source=["api", "db", "cache", "auth", "storage"],
        destination=["api", "db", "cache", "auth", "storage"],
        action="allow",
        description="Allow internal services to communicate with each other"
    )
    
    # Add policy to manager
    network_policy_manager.add_policy(internal_policy)
    
    # Default policy for external services
    external_policy = NetworkPolicy(
        name="external",
        description="Policy for external service communication"
    )
    
    # Allow external services to communicate with API only
    external_policy.add_rule(
        source=["*"],
        destination=["api"],
        ports=[80, 443],
        protocols=["http", "https"],
        action="allow",
        description="Allow external services to communicate with API"
    )
    
    # Add policy to manager
    network_policy_manager.add_policy(external_policy)
    
    logger.info("Created default network policies")


def is_allowed(
    source: str,
    destination: str,
    port: Optional[int] = None,
    protocol: Optional[str] = None
) -> bool:
    """
    Check if communication is allowed by any policy.
    
    Args:
        source: Source service or IP
        destination: Destination service or IP
        port: Port number
        protocol: Protocol
        
    Returns:
        True if allowed by any policy, False otherwise
    """
    return network_policy_manager.is_allowed(source, destination, port, protocol)


def add_policy(policy: NetworkPolicy):
    """
    Add a policy.
    
    Args:
        policy: Network policy
    """
    network_policy_manager.add_policy(policy)


def remove_policy(name: str):
    """
    Remove a policy.
    
    Args:
        name: Policy name
    """
    network_policy_manager.remove_policy(name)


def get_policy(name: str) -> Optional[NetworkPolicy]:
    """
    Get a policy.
    
    Args:
        name: Policy name
        
    Returns:
        Network policy or None if not found
    """
    return network_policy_manager.get_policy(name)


def save_policies(file_path: str):
    """
    Save policies to file.
    
    Args:
        file_path: File path
    """
    network_policy_manager.save_to_file(file_path)


def load_policies(file_path: str):
    """
    Load policies from file.
    
    Args:
        file_path: File path
    """
    global network_policy_manager
    network_policy_manager = NetworkPolicyManager.load_from_file(file_path)
