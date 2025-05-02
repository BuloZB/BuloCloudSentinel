"""
Authentication UI for Bulo.Cloud Sentinel.

This module provides a web interface for user authentication, including login,
logout, and password management.
"""

import os
import sys
import logging
import time
import json
from typing import Dict, Any, Optional, List, Tuple
import gradio as gr

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import local modules
from ai.inference.auth import AuthManager
from ai.inference.config import ConfigManager
from ai.inference.monitoring import structured_logger, metrics_collector, alert_manager

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create configuration manager
config = ConfigManager()

# Create authentication manager
auth_manager = AuthManager(
    secret_key=config.get("auth.jwt_secret"),
    token_expiration=config.get("auth.token_expiration", 3600),
    refresh_token_expiration=config.get("auth.refresh_token_expiration", 86400 * 7)
)


class AuthUI:
    """
    Authentication UI for Bulo.Cloud Sentinel.
    
    This class provides a web interface for user authentication.
    """
    
    def __init__(self):
        """Initialize the authentication UI."""
        self.auth_manager = auth_manager
        self.config = config
        
        # Initialize session data
        self.sessions = {}
    
    def login(self, username: str, password: str) -> Dict[str, Any]:
        """
        Login a user.
        
        Args:
            username: Username
            password: Password
            
        Returns:
            Login result
        """
        try:
            # Check if username and password are provided
            if not username or not password:
                structured_logger.warning(
                    "Missing username or password",
                    logger_name="auth_ui"
                )
                return {"success": False, "message": "Username and password are required"}
            
            # Authenticate user
            user = self.auth_manager.authenticate(username, password)
            if not user:
                structured_logger.warning(
                    "Invalid username or password",
                    logger_name="auth_ui",
                    username=username
                )
                
                # Increment failed login metric
                metrics_collector.increment_metric("failed_logins")
                
                return {"success": False, "message": "Invalid username or password"}
            
            # Create tokens
            tokens = self.auth_manager.create_token(username)
            if not tokens:
                structured_logger.error(
                    "Error creating tokens",
                    logger_name="auth_ui",
                    username=username
                )
                return {"success": False, "message": "Error creating tokens"}
            
            # Create session ID
            session_id = f"{username}_{int(time.time())}"
            
            # Store session data
            self.sessions[session_id] = {
                "username": username,
                "role": user["role"],
                "access_token": tokens["access_token"],
                "refresh_token": tokens["refresh_token"],
                "created_at": time.time(),
                "expires_at": time.time() + tokens["expires_in"]
            }
            
            # Log successful login
            structured_logger.info(
                "User logged in successfully",
                logger_name="auth_ui",
                username=username,
                role=user["role"]
            )
            
            # Increment successful login metric
            metrics_collector.increment_metric("successful_logins")
            
            return {
                "success": True,
                "message": f"Welcome, {username}!",
                "session_id": session_id,
                "role": user["role"]
            }
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error logging in",
                logger_name="auth_ui",
                error=str(e)
            )
            
            # Create alert
            alert_manager.create_alert(
                level="error",
                message=f"Error logging in: {str(e)}",
                source="auth_ui"
            )
            
            return {"success": False, "message": "An error occurred during login"}
    
    def logout(self, session_id: str) -> Dict[str, Any]:
        """
        Logout a user.
        
        Args:
            session_id: Session ID
            
        Returns:
            Logout result
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"success": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"success": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Remove session
            del self.sessions[session_id]
            
            # Log successful logout
            structured_logger.info(
                "User logged out successfully",
                logger_name="auth_ui",
                username=session["username"]
            )
            
            return {"success": True, "message": "Logged out successfully"}
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error logging out",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"success": False, "message": "An error occurred during logout"}
    
    def change_password(
        self,
        session_id: str,
        current_password: str,
        new_password: str,
        confirm_password: str
    ) -> Dict[str, Any]:
        """
        Change a user's password.
        
        Args:
            session_id: Session ID
            current_password: Current password
            new_password: New password
            confirm_password: Confirm new password
            
        Returns:
            Password change result
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"success": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"success": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Check if passwords are provided
            if not current_password or not new_password or not confirm_password:
                return {"success": False, "message": "All password fields are required"}
            
            # Check if new password and confirm password match
            if new_password != confirm_password:
                return {"success": False, "message": "New password and confirm password do not match"}
            
            # Check if new password is strong enough
            if len(new_password) < 8:
                return {"success": False, "message": "New password must be at least 8 characters long"}
            
            # Authenticate user with current password
            user = self.auth_manager.authenticate(session["username"], current_password)
            if not user:
                return {"success": False, "message": "Current password is incorrect"}
            
            # Update password
            success = self.auth_manager.update_user(
                username=session["username"],
                password=new_password
            )
            
            if not success:
                return {"success": False, "message": "Error updating password"}
            
            # Log successful password change
            structured_logger.info(
                "User changed password successfully",
                logger_name="auth_ui",
                username=session["username"]
            )
            
            return {"success": True, "message": "Password changed successfully"}
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error changing password",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"success": False, "message": "An error occurred while changing password"}
    
    def create_api_key(self, session_id: str, description: str) -> Dict[str, Any]:
        """
        Create an API key for a user.
        
        Args:
            session_id: Session ID
            description: API key description
            
        Returns:
            API key creation result
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"success": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"success": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Check if description is provided
            if not description:
                return {"success": False, "message": "Description is required"}
            
            # Create API key
            api_key = self.auth_manager.create_api_key(
                username=session["username"],
                description=description
            )
            
            if not api_key:
                return {"success": False, "message": "Error creating API key"}
            
            # Log successful API key creation
            structured_logger.info(
                "User created API key successfully",
                logger_name="auth_ui",
                username=session["username"]
            )
            
            return {
                "success": True,
                "message": "API key created successfully",
                "api_key": api_key
            }
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error creating API key",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"success": False, "message": "An error occurred while creating API key"}
    
    def get_api_keys(self, session_id: str) -> Dict[str, Any]:
        """
        Get API keys for a user.
        
        Args:
            session_id: Session ID
            
        Returns:
            API keys
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"success": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"success": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Get API keys
            api_keys = self.auth_manager.get_api_keys(session["username"])
            
            return {
                "success": True,
                "api_keys": api_keys
            }
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error getting API keys",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"success": False, "message": "An error occurred while getting API keys"}
    
    def delete_api_key(self, session_id: str, api_key_hash: str) -> Dict[str, Any]:
        """
        Delete an API key.
        
        Args:
            session_id: Session ID
            api_key_hash: API key hash
            
        Returns:
            API key deletion result
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"success": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"success": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Check if API key hash is provided
            if not api_key_hash:
                return {"success": False, "message": "API key hash is required"}
            
            # Get API keys
            api_keys = self.auth_manager.get_api_keys(session["username"])
            
            # Check if API key exists
            api_key_exists = False
            for api_key in api_keys:
                if api_key["hash"] == api_key_hash:
                    api_key_exists = True
                    break
            
            if not api_key_exists:
                return {"success": False, "message": "API key not found"}
            
            # Delete API key
            success = self.auth_manager.delete_api_key(
                username=session["username"],
                api_key=api_key_hash
            )
            
            if not success:
                return {"success": False, "message": "Error deleting API key"}
            
            # Log successful API key deletion
            structured_logger.info(
                "User deleted API key successfully",
                logger_name="auth_ui",
                username=session["username"]
            )
            
            return {"success": True, "message": "API key deleted successfully"}
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error deleting API key",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"success": False, "message": "An error occurred while deleting API key"}
    
    def validate_session(self, session_id: str) -> Dict[str, Any]:
        """
        Validate a session.
        
        Args:
            session_id: Session ID
            
        Returns:
            Session validation result
        """
        try:
            # Check if session ID is provided
            if not session_id:
                return {"valid": False, "message": "Session ID is required"}
            
            # Check if session exists
            if session_id not in self.sessions:
                return {"valid": False, "message": "Invalid session"}
            
            # Get session data
            session = self.sessions[session_id]
            
            # Check if session has expired
            if time.time() > session["expires_at"]:
                # Remove session
                del self.sessions[session_id]
                
                return {"valid": False, "message": "Session has expired"}
            
            # Validate token
            token_payload = self.auth_manager.validate_token(session["access_token"])
            if not token_payload:
                # Try to refresh token
                tokens = self.auth_manager.refresh_token(session["refresh_token"])
                if not tokens:
                    # Remove session
                    del self.sessions[session_id]
                    
                    return {"valid": False, "message": "Invalid token"}
                
                # Update session data
                session["access_token"] = tokens["access_token"]
                session["expires_at"] = time.time() + tokens["expires_in"]
            
            return {
                "valid": True,
                "username": session["username"],
                "role": session["role"]
            }
        
        except Exception as e:
            # Log error
            structured_logger.error(
                "Error validating session",
                logger_name="auth_ui",
                error=str(e)
            )
            
            return {"valid": False, "message": "An error occurred while validating session"}
    
    def create_web_interface(self):
        """Create the web interface."""
        with gr.Blocks(title="Bulo.Cloud Sentinel Authentication") as app:
            gr.Markdown("# Bulo.Cloud Sentinel Authentication")
            
            # Session state
            session_id = gr.State(value=None)
            
            # Login tab
            with gr.Tab("Login"):
                with gr.Row():
                    with gr.Column():
                        username = gr.Textbox(label="Username")
                        password = gr.Textbox(label="Password", type="password")
                        login_button = gr.Button("Login")
                        login_result = gr.JSON(label="Login Result")
                        
                        login_button.click(
                            fn=self.login,
                            inputs=[username, password],
                            outputs=[login_result]
                        )
            
            # Change password tab
            with gr.Tab("Change Password"):
                with gr.Row():
                    with gr.Column():
                        current_password = gr.Textbox(label="Current Password", type="password")
                        new_password = gr.Textbox(label="New Password", type="password")
                        confirm_password = gr.Textbox(label="Confirm Password", type="password")
                        change_password_button = gr.Button("Change Password")
                        change_password_result = gr.JSON(label="Change Password Result")
                        
                        change_password_button.click(
                            fn=self.change_password,
                            inputs=[session_id, current_password, new_password, confirm_password],
                            outputs=[change_password_result]
                        )
            
            # API keys tab
            with gr.Tab("API Keys"):
                with gr.Row():
                    with gr.Column():
                        api_key_description = gr.Textbox(label="API Key Description")
                        create_api_key_button = gr.Button("Create API Key")
                        create_api_key_result = gr.JSON(label="Create API Key Result")
                        
                        create_api_key_button.click(
                            fn=self.create_api_key,
                            inputs=[session_id, api_key_description],
                            outputs=[create_api_key_result]
                        )
                        
                        get_api_keys_button = gr.Button("Get API Keys")
                        api_keys_result = gr.JSON(label="API Keys")
                        
                        get_api_keys_button.click(
                            fn=self.get_api_keys,
                            inputs=[session_id],
                            outputs=[api_keys_result]
                        )
                        
                        api_key_hash = gr.Textbox(label="API Key Hash")
                        delete_api_key_button = gr.Button("Delete API Key")
                        delete_api_key_result = gr.JSON(label="Delete API Key Result")
                        
                        delete_api_key_button.click(
                            fn=self.delete_api_key,
                            inputs=[session_id, api_key_hash],
                            outputs=[delete_api_key_result]
                        )
            
            # Logout tab
            with gr.Tab("Logout"):
                with gr.Row():
                    with gr.Column():
                        logout_button = gr.Button("Logout")
                        logout_result = gr.JSON(label="Logout Result")
                        
                        logout_button.click(
                            fn=self.logout,
                            inputs=[session_id],
                            outputs=[logout_result]
                        )
        
        return app


def main():
    """Main function."""
    # Initialize metrics collector
    metrics_collector.start()
    
    # Create authentication UI
    auth_ui = AuthUI()
    
    # Create web interface
    app = auth_ui.create_web_interface()
    
    # Launch web interface with security settings
    app.launch(
        server_name="0.0.0.0",
        server_port=7862,
        max_threads=10,  # Limit number of concurrent requests
        share=False,     # Disable public sharing
        auth=None,       # No authentication (add if needed)
        ssl_verify=True  # Verify SSL certificates
    )
    
    # Log startup
    structured_logger.info(
        "Authentication UI started",
        logger_name="auth_ui",
        port=7862
    )


if __name__ == "__main__":
    main()
