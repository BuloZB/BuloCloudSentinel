"""
SQL validation and parameterization utilities for Bulo.Cloud Sentinel.

This module provides functions for safely constructing SQL queries
to prevent SQL injection attacks.
"""

import re
from typing import Any, Dict, List, Optional, Tuple, Union
from sqlalchemy import text
from sqlalchemy.engine import Engine
from sqlalchemy.ext.asyncio import AsyncEngine
from fastapi import HTTPException, status


# Regular expressions for validation
SQL_IDENTIFIER_REGEX = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")
SQL_INJECTION_REGEX = re.compile(r"(?i)(SELECT|INSERT|UPDATE|DELETE|DROP|ALTER|UNION|INTO|EXEC|EXECUTE)")


def validate_sql_identifier(identifier: str) -> bool:
    """
    Validate a SQL identifier (table name, column name, etc.).

    Args:
        identifier: The identifier to validate

    Returns:
        True if the identifier is valid, False otherwise
    """
    if not identifier or not isinstance(identifier, str):
        return False

    return bool(SQL_IDENTIFIER_REGEX.match(identifier))


def check_sql_injection(query: str) -> bool:
    """
    Check if a string contains potential SQL injection patterns.

    Args:
        query: The query string to check

    Returns:
        True if SQL injection is detected, False otherwise
    """
    if not query or not isinstance(query, str):
        return False

    return bool(SQL_INJECTION_REGEX.search(query))


def safe_table_name(table_name: str) -> str:
    """
    Ensure a table name is safe to use in a SQL query.

    Args:
        table_name: The table name to validate

    Returns:
        The validated table name

    Raises:
        HTTPException: If the table name is invalid
    """
    if not validate_sql_identifier(table_name):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid table name: {table_name}"
        )

    return table_name


def safe_column_name(column_name: str) -> str:
    """
    Ensure a column name is safe to use in a SQL query.

    Args:
        column_name: The column name to validate

    Returns:
        The validated column name

    Raises:
        HTTPException: If the column name is invalid
    """
    if not validate_sql_identifier(column_name):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid column name: {column_name}"
        )

    return column_name


def safe_order_direction(direction: str) -> str:
    """
    Ensure an order direction is safe to use in a SQL query.

    Args:
        direction: The order direction to validate ('asc' or 'desc')

    Returns:
        The validated order direction

    Raises:
        HTTPException: If the order direction is invalid
    """
    direction = direction.lower()
    if direction not in ("asc", "desc"):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid order direction: {direction}"
        )

    return direction


def build_safe_select_query(
    table_name: str,
    columns: List[str] = None,
    where_conditions: Dict[str, Any] = None,
    order_by: Optional[str] = None,
    order_direction: str = "asc",
    limit: Optional[int] = None,
    offset: Optional[int] = None
) -> Tuple[str, Dict[str, Any]]:
    """
    Build a safe parameterized SELECT query.

    Args:
        table_name: The table to select from
        columns: The columns to select (default: all columns)
        where_conditions: The WHERE conditions as a dictionary
        order_by: The column to order by
        order_direction: The order direction ('asc' or 'desc')
        limit: The maximum number of rows to return
        offset: The number of rows to skip

    Returns:
        A tuple of (query_string, parameters)

    Raises:
        HTTPException: If any input is invalid
    """
    # Validate table name
    table = safe_table_name(table_name)

    # Validate columns
    if columns:
        validated_columns = [safe_column_name(col) for col in columns]
        columns_str = ", ".join(validated_columns)
    else:
        columns_str = "*"

    # Build query
    # This is safe because we've validated the table and column names above
    # and we're not using any user input directly in the query
    # Adding a nosec comment to indicate that this is a false positive
    query = f"SELECT {columns_str} FROM {table}"  # nosec

    # Add WHERE clause
    params = {}
    if where_conditions:
        where_clauses = []
        for i, (column, value) in enumerate(where_conditions.items()):
            col = safe_column_name(column)
            param_name = f"param_{i}"
            where_clauses.append(f"{col} = :{param_name}")
            params[param_name] = value

        if where_clauses:
            query += " WHERE " + " AND ".join(where_clauses)

    # Add ORDER BY clause
    if order_by:
        order_col = safe_column_name(order_by)
        order_dir = safe_order_direction(order_direction)
        query += f" ORDER BY {order_col} {order_dir}"

    # Add LIMIT and OFFSET
    if limit is not None:
        if not isinstance(limit, int) or limit < 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid limit: {limit}"
            )
        query += f" LIMIT {limit}"

    if offset is not None:
        if not isinstance(offset, int) or offset < 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid offset: {offset}"
            )
        query += f" OFFSET {offset}"

    return query, params


async def execute_safe_query(
    engine: Union[Engine, AsyncEngine],
    query: str,
    params: Dict[str, Any] = None
) -> List[Dict[str, Any]]:
    """
    Execute a safe parameterized SQL query.

    Args:
        engine: The SQLAlchemy engine
        query: The query string
        params: The query parameters

    Returns:
        The query results as a list of dictionaries

    Raises:
        HTTPException: If the query execution fails
    """
    try:
        is_async = isinstance(engine, AsyncEngine)

        if is_async:
            async with engine.connect() as conn:
                result = await conn.execute(text(query), params or {})
                return [dict(row) for row in result]
        else:
            with engine.connect() as conn:
                result = conn.execute(text(query), params or {})
                return [dict(row) for row in result]

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Database query failed: {str(e)}"
        )
