"""
Database security utilities for the Bulo.Cloud Sentinel backend.

This module provides utilities for enhancing database security and preventing SQL injection.
"""

import re
import logging
from typing import Any, Dict, List, Optional, Set, Tuple, Union
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.sql.expression import Select, Insert, Update, Delete
from pydantic import BaseModel

logger = logging.getLogger(__name__)

# SQL injection patterns to detect
SQL_INJECTION_PATTERNS = [
    r"(\s|^)(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|CREATE|TRUNCATE|GRANT|REVOKE)(\s|$)",
    r"(\s|^)(UNION|JOIN|INNER|OUTER|LEFT|RIGHT|FULL|CROSS)(\s|$)",
    r"(\s|^)(WHERE|FROM|INTO|VALUES|SET|GROUP|ORDER|HAVING|LIMIT)(\s|$)",
    r"(\s|^)(AND|OR|NOT|IS|IN|BETWEEN|LIKE|REGEXP|MATCH)(\s|$)",
    r"(\s|^)(--|#|/\*|\*/)",
    r"(\s|^)(EXEC|EXECUTE|DECLARE|CAST|CONVERT|CHAR|NCHAR|VARCHAR|NVARCHAR)(\s|$)",
    r"(\s|^)(XP_|SP_)",
    r"(\s|^)(WAITFOR|DELAY|SLEEP)(\s|$)",
    r"(\s|^)(BENCHMARK|LOAD_FILE|OUTFILE|DUMPFILE)(\s|$)",
    r"(\s|^)(INFORMATION_SCHEMA|SYS|MYSQL|SQLITE_MASTER)(\s|$)",
]

# Compiled regex patterns for better performance
COMPILED_PATTERNS = [re.compile(pattern, re.IGNORECASE) for pattern in SQL_INJECTION_PATTERNS]


def detect_sql_injection(value: str) -> bool:
    """
    Detect potential SQL injection in a string.
    
    Args:
        value: The string to check.
        
    Returns:
        bool: True if SQL injection is detected, False otherwise.
    """
    if not isinstance(value, str):
        return False
    
    for pattern in COMPILED_PATTERNS:
        if pattern.search(value):
            return True
    
    return False


def sanitize_input(value: Any) -> Any:
    """
    Sanitize input to prevent SQL injection.
    
    Args:
        value: The value to sanitize.
        
    Returns:
        Any: The sanitized value.
    """
    if isinstance(value, str):
        # Replace dangerous characters
        value = value.replace("'", "''")
        value = value.replace("\\", "\\\\")
        value = value.replace("%", "\\%")
        value = value.replace("_", "\\_")
        
        # Check for SQL injection
        if detect_sql_injection(value):
            logger.warning(f"Potential SQL injection detected: {value}")
            raise ValueError("Invalid input: potential SQL injection detected")
    
    return value


def sanitize_dict(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Sanitize a dictionary of values to prevent SQL injection.
    
    Args:
        data: The dictionary to sanitize.
        
    Returns:
        Dict[str, Any]: The sanitized dictionary.
    """
    return {key: sanitize_input(value) for key, value in data.items()}


def sanitize_model(model: BaseModel) -> Dict[str, Any]:
    """
    Sanitize a Pydantic model to prevent SQL injection.
    
    Args:
        model: The Pydantic model to sanitize.
        
    Returns:
        Dict[str, Any]: The sanitized dictionary.
    """
    return sanitize_dict(model.dict())


async def execute_safe_query(
    session: AsyncSession,
    query: Union[str, Select, Insert, Update, Delete],
    params: Optional[Dict[str, Any]] = None,
) -> Any:
    """
    Execute a safe SQL query with parameterized values.
    
    Args:
        session: The SQLAlchemy session.
        query: The SQL query or SQLAlchemy expression.
        params: The query parameters.
        
    Returns:
        Any: The query result.
    """
    if isinstance(query, str):
        # If the query is a string, use parameterized queries
        if params:
            # Sanitize parameters
            sanitized_params = sanitize_dict(params)
            
            # Execute the query with parameters
            result = await session.execute(text(query), sanitized_params)
        else:
            # Execute the query without parameters
            result = await session.execute(text(query))
    else:
        # If the query is a SQLAlchemy expression, execute it directly
        result = await session.execute(query)
    
    return result
