"""
Database security utilities.

This module provides functions for secure database operations.
"""

from typing import Any, Dict, List, Optional, Tuple, Union
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.sql.expression import Select, Update, Delete, Insert

async def execute_safe_query(
    session: AsyncSession,
    query: str,
    params: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """
    Execute a SQL query safely with parameterized values.
    
    Args:
        session: SQLAlchemy async session
        query: SQL query string with named parameters
        params: Dictionary of parameter values
        
    Returns:
        List of result rows as dictionaries
    """
    # Create a text query with parameters
    sql = text(query)
    
    # Execute the query
    result = await session.execute(sql, params)
    
    # Convert result to list of dictionaries
    return [dict(row._mapping) for row in result]

def validate_order_by(
    order_by: str,
    allowed_columns: List[str]
) -> Tuple[bool, Optional[str]]:
    """
    Validate an ORDER BY clause to prevent SQL injection.
    
    Args:
        order_by: The ORDER BY clause to validate
        allowed_columns: List of allowed column names
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    if not order_by:
        return True, None
        
    # Split the order by clause into parts
    parts = order_by.split(',')
    
    for part in parts:
        # Remove whitespace and split by space to separate column and direction
        clean_part = part.strip()
        column_parts = clean_part.split(' ')
        
        # Get the column name
        column = column_parts[0].strip()
        
        # Check if the column is allowed
        if column not in allowed_columns:
            return False, f"Column '{column}' is not allowed in ORDER BY clause"
            
        # Check if the direction is valid (if specified)
        if len(column_parts) > 1:
            direction = column_parts[1].strip().upper()
            if direction not in ('ASC', 'DESC'):
                return False, f"Invalid sort direction '{direction}'"
                
    return True, None

def validate_where_condition(
    condition: str,
    allowed_columns: List[str],
    allowed_operators: List[str]
) -> Tuple[bool, Optional[str]]:
    """
    Validate a WHERE condition to prevent SQL injection.
    
    Args:
        condition: The WHERE condition to validate
        allowed_columns: List of allowed column names
        allowed_operators: List of allowed operators
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    import re
    
    if not condition:
        return True, None
        
    # Define regex pattern for column name, operator, and value
    pattern = r'(\w+)\s*([=<>!]+)\s*(.+)'
    
    # Find all matches
    matches = re.findall(pattern, condition)
    
    for match in matches:
        if len(match) != 3:
            continue
            
        column, operator, _ = match
        
        # Check if the column is allowed
        if column not in allowed_columns:
            return False, f"Column '{column}' is not allowed in WHERE condition"
            
        # Check if the operator is allowed
        if operator not in allowed_operators:
            return False, f"Operator '{operator}' is not allowed in WHERE condition"
            
    return True, None

def build_safe_query(
    table: str,
    columns: List[str],
    where: Optional[Dict[str, Any]] = None,
    order_by: Optional[str] = None,
    limit: Optional[int] = None,
    offset: Optional[int] = None
) -> Tuple[str, Dict[str, Any]]:
    """
    Build a safe SQL query with parameterized values.
    
    Args:
        table: Table name
        columns: List of column names to select
        where: Dictionary of column-value pairs for WHERE clause
        order_by: ORDER BY clause
        limit: LIMIT value
        offset: OFFSET value
        
    Returns:
        Tuple of (query_string, params_dict)
    """
    # Build SELECT clause
    query = f"SELECT {', '.join(columns)} FROM {table}"
    
    # Build WHERE clause
    params = {}
    if where:
        conditions = []
        for i, (column, value) in enumerate(where.items()):
            param_name = f"param_{i}"
            conditions.append(f"{column} = :{param_name}")
            params[param_name] = value
            
        if conditions:
            query += f" WHERE {' AND '.join(conditions)}"
            
    # Add ORDER BY clause
    if order_by:
        query += f" ORDER BY {order_by}"
        
    # Add LIMIT and OFFSET
    if limit is not None:
        query += f" LIMIT {limit}"
        
    if offset is not None:
        query += f" OFFSET {offset}"
        
    return query, params

async def paginate_query(
    session: AsyncSession,
    query: Select,
    page: int = 1,
    page_size: int = 10
) -> Tuple[List[Dict[str, Any]], int]:
    """
    Paginate a SQLAlchemy query.
    
    Args:
        session: SQLAlchemy async session
        query: SQLAlchemy select query
        page: Page number (1-based)
        page_size: Number of items per page
        
    Returns:
        Tuple of (items, total_count)
    """
    # Ensure page and page_size are valid
    page = max(1, page)
    page_size = max(1, min(100, page_size))  # Limit page size to prevent DoS
    
    # Calculate offset
    offset = (page - 1) * page_size
    
    # Get total count
    count_query = query.with_only_columns([text("COUNT(*)")]).order_by(None)
    total = await session.scalar(count_query)
    
    # Apply pagination
    paginated_query = query.limit(page_size).offset(offset)
    
    # Execute query
    result = await session.execute(paginated_query)
    
    # Convert result to list of dictionaries
    items = [dict(row._mapping) for row in result]
    
    return items, total
