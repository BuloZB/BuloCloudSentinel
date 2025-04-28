"""
Cookie handler for JWT tokens in Bulo.Cloud Sentinel Security Module.

This module provides secure cookie handling for JWT tokens.
"""

import os
from typing import Optional
from datetime import datetime, timedelta

from fastapi import Request, Response
from starlette.datastructures import MutableHeaders

# Default cookie settings
DEFAULT_ACCESS_COOKIE_NAME = "access_token"
DEFAULT_REFRESH_COOKIE_NAME = "refresh_token"
DEFAULT_ACCESS_COOKIE_PATH = "/api"
DEFAULT_REFRESH_COOKIE_PATH = "/api/auth"
DEFAULT_COOKIE_DOMAIN = None
DEFAULT_COOKIE_SECURE = True
DEFAULT_COOKIE_HTTPONLY = True
DEFAULT_COOKIE_SAMESITE = "lax"
DEFAULT_COOKIE_MAX_AGE_ACCESS = 30 * 60  # 30 minutes
DEFAULT_COOKIE_MAX_AGE_REFRESH = 7 * 24 * 60 * 60  # 7 days

class CookieHandler:
    """
    Handler for JWT token cookies.
    
    This class provides methods for setting and deleting JWT token cookies.
    """
    
    def __init__(
        self,
        access_cookie_name: str = DEFAULT_ACCESS_COOKIE_NAME,
        refresh_cookie_name: str = DEFAULT_REFRESH_COOKIE_NAME,
        access_cookie_path: str = DEFAULT_ACCESS_COOKIE_PATH,
        refresh_cookie_path: str = DEFAULT_REFRESH_COOKIE_PATH,
        cookie_domain: Optional[str] = DEFAULT_COOKIE_DOMAIN,
        cookie_secure: bool = DEFAULT_COOKIE_SECURE,
        cookie_httponly: bool = DEFAULT_COOKIE_HTTPONLY,
        cookie_samesite: str = DEFAULT_COOKIE_SAMESITE,
        cookie_max_age_access: int = DEFAULT_COOKIE_MAX_AGE_ACCESS,
        cookie_max_age_refresh: int = DEFAULT_COOKIE_MAX_AGE_REFRESH
    ):
        """
        Initialize the cookie handler.
        
        Args:
            access_cookie_name: Name of the access token cookie
            refresh_cookie_name: Name of the refresh token cookie
            access_cookie_path: Path for the access token cookie
            refresh_cookie_path: Path for the refresh token cookie
            cookie_domain: Domain for the cookies
            cookie_secure: Whether to set the Secure flag on the cookies
            cookie_httponly: Whether to set the HttpOnly flag on the cookies
            cookie_samesite: SameSite attribute for the cookies
            cookie_max_age_access: Max age of the access token cookie in seconds
            cookie_max_age_refresh: Max age of the refresh token cookie in seconds
        """
        self.access_cookie_name = access_cookie_name
        self.refresh_cookie_name = refresh_cookie_name
        self.access_cookie_path = access_cookie_path
        self.refresh_cookie_path = refresh_cookie_path
        self.cookie_domain = cookie_domain
        self.cookie_secure = cookie_secure
        self.cookie_httponly = cookie_httponly
        self.cookie_samesite = cookie_samesite
        self.cookie_max_age_access = cookie_max_age_access
        self.cookie_max_age_refresh = cookie_max_age_refresh
    
    def set_access_cookie(self, response: Response, token: str) -> None:
        """
        Set the access token cookie on the response.
        
        Args:
            response: HTTP response
            token: Access token
        """
        response.set_cookie(
            key=self.access_cookie_name,
            value=token,
            max_age=self.cookie_max_age_access,
            path=self.access_cookie_path,
            domain=self.cookie_domain,
            secure=self.cookie_secure,
            httponly=self.cookie_httponly,
            samesite=self.cookie_samesite
        )
    
    def set_refresh_cookie(self, response: Response, token: str) -> None:
        """
        Set the refresh token cookie on the response.
        
        Args:
            response: HTTP response
            token: Refresh token
        """
        response.set_cookie(
            key=self.refresh_cookie_name,
            value=token,
            max_age=self.cookie_max_age_refresh,
            path=self.refresh_cookie_path,
            domain=self.cookie_domain,
            secure=self.cookie_secure,
            httponly=self.cookie_httponly,
            samesite=self.cookie_samesite
        )
    
    def set_token_cookies(self, response: Response, access_token: str, refresh_token: str) -> None:
        """
        Set both access and refresh token cookies on the response.
        
        Args:
            response: HTTP response
            access_token: Access token
            refresh_token: Refresh token
        """
        self.set_access_cookie(response, access_token)
        self.set_refresh_cookie(response, refresh_token)
    
    def unset_access_cookie(self, response: Response) -> None:
        """
        Unset the access token cookie on the response.
        
        Args:
            response: HTTP response
        """
        response.delete_cookie(
            key=self.access_cookie_name,
            path=self.access_cookie_path,
            domain=self.cookie_domain
        )
    
    def unset_refresh_cookie(self, response: Response) -> None:
        """
        Unset the refresh token cookie on the response.
        
        Args:
            response: HTTP response
        """
        response.delete_cookie(
            key=self.refresh_cookie_name,
            path=self.refresh_cookie_path,
            domain=self.cookie_domain
        )
    
    def unset_token_cookies(self, response: Response) -> None:
        """
        Unset both access and refresh token cookies on the response.
        
        Args:
            response: HTTP response
        """
        self.unset_access_cookie(response)
        self.unset_refresh_cookie(response)
    
    def get_access_token_from_cookies(self, request: Request) -> Optional[str]:
        """
        Get the access token from the request cookies.
        
        Args:
            request: HTTP request
            
        Returns:
            Access token or None if not found
        """
        return request.cookies.get(self.access_cookie_name)
    
    def get_refresh_token_from_cookies(self, request: Request) -> Optional[str]:
        """
        Get the refresh token from the request cookies.
        
        Args:
            request: HTTP request
            
        Returns:
            Refresh token or None if not found
        """
        return request.cookies.get(self.refresh_cookie_name)

# Global cookie handler instance
cookie_handler = CookieHandler()

def set_jwt_cookies(response: Response, access_token: str, refresh_token: str) -> None:
    """
    Set JWT token cookies on the response.
    
    Args:
        response: HTTP response
        access_token: Access token
        refresh_token: Refresh token
    """
    cookie_handler.set_token_cookies(response, access_token, refresh_token)

def unset_jwt_cookies(response: Response) -> None:
    """
    Unset JWT token cookies on the response.
    
    Args:
        response: HTTP response
    """
    cookie_handler.unset_token_cookies(response)

def get_access_token_from_cookies(request: Request) -> Optional[str]:
    """
    Get the access token from the request cookies.
    
    Args:
        request: HTTP request
        
    Returns:
        Access token or None if not found
    """
    return cookie_handler.get_access_token_from_cookies(request)

def get_refresh_token_from_cookies(request: Request) -> Optional[str]:
    """
    Get the refresh token from the request cookies.
    
    Args:
        request: HTTP request
        
    Returns:
        Refresh token or None if not found
    """
    return cookie_handler.get_refresh_token_from_cookies(request)