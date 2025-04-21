"""
Secure logging utilities for the Bulo.Cloud Sentinel backend.

This module provides utilities for secure logging to prevent sensitive data exposure.
"""

import logging
import re
import json
from typing import Any, Dict, List, Optional, Set, Tuple, Union
import uuid
from datetime import datetime
import os
import sys

# Sensitive data patterns to mask
SENSITIVE_PATTERNS = [
    (re.compile(r"password\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "password=***"),
    (re.compile(r"token\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "token=***"),
    (re.compile(r"api[-_]?key\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "api_key=***"),
    (re.compile(r"secret\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "secret=***"),
    (re.compile(r"auth\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "auth=***"),
    (re.compile(r"credential\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "credential=***"),
    (re.compile(r"passphrase\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "passphrase=***"),
    (re.compile(r"private[-_]?key\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "private_key=***"),
    (re.compile(r"authorization\s*[=:]\s*['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "authorization=***"),
    (re.compile(r"bearer\s+['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "bearer ***"),
    (re.compile(r"basic\s+['\"]?([^'\"]+)['\"]?", re.IGNORECASE), "basic ***"),
]


class SensitiveDataFilter(logging.Filter):
    """
    Filter for masking sensitive data in log messages.
    
    This filter masks sensitive data like passwords, tokens, and API keys in log messages.
    """
    
    def filter(self, record: logging.LogRecord) -> bool:
        """
        Filter log records to mask sensitive data.
        
        Args:
            record: The log record.
            
        Returns:
            bool: Always True (the record is always processed).
        """
        if isinstance(record.msg, str):
            for pattern, replacement in SENSITIVE_PATTERNS:
                record.msg = pattern.sub(replacement, record.msg)
        
        if hasattr(record, "args") and record.args:
            if isinstance(record.args, dict):
                for key, value in record.args.items():
                    if isinstance(value, str):
                        for pattern, replacement in SENSITIVE_PATTERNS:
                            record.args[key] = pattern.sub(replacement, value)
            else:
                args_list = list(record.args)
                for i, arg in enumerate(args_list):
                    if isinstance(arg, str):
                        for pattern, replacement in SENSITIVE_PATTERNS:
                            args_list[i] = pattern.sub(replacement, arg)
                record.args = tuple(args_list)
        
        return True


class RequestIdFilter(logging.Filter):
    """
    Filter for adding request ID to log records.
    
    This filter adds a unique request ID to log records for request tracing.
    """
    
    def __init__(self, request_id: Optional[str] = None):
        """
        Initialize the filter.
        
        Args:
            request_id: The request ID to use. If None, a new UUID is generated.
        """
        super().__init__()
        self.request_id = request_id or str(uuid.uuid4())
    
    def filter(self, record: logging.LogRecord) -> bool:
        """
        Filter log records to add request ID.
        
        Args:
            record: The log record.
            
        Returns:
            bool: Always True (the record is always processed).
        """
        record.request_id = self.request_id
        return True


class JSONFormatter(logging.Formatter):
    """
    JSON formatter for structured logging.
    
    This formatter outputs log records as JSON objects for easier parsing and analysis.
    """
    
    def format(self, record: logging.LogRecord) -> str:
        """
        Format log records as JSON.
        
        Args:
            record: The log record.
            
        Returns:
            str: The formatted log record as a JSON string.
        """
        log_data = {
            "timestamp": self.formatTime(record, self.datefmt),
            "level": record.levelname,
            "name": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
            "process": record.process,
            "thread": record.thread,
        }
        
        if hasattr(record, "request_id"):
            log_data["request_id"] = record.request_id
        
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)
        
        return json.dumps(log_data)


def setup_logging(
    log_level: str = "INFO",
    log_file: Optional[str] = None,
    json_format: bool = False,
    max_bytes: int = 10 * 1024 * 1024,  # 10 MB
    backup_count: int = 5,
) -> None:
    """
    Set up secure logging.
    
    Args:
        log_level: The log level.
        log_file: The log file path. If None, logs are output to stderr.
        json_format: Whether to use JSON format for logs.
        max_bytes: The maximum size of the log file before rotation.
        backup_count: The number of backup log files to keep.
    """
    # Get log level
    level = getattr(logging, log_level.upper(), logging.INFO)
    
    # Create formatter
    if json_format:
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - [%(request_id)s] - %(message)s"
        )
    
    # Create handlers
    handlers = []
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stderr)
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)
    
    # File handler
    if log_file:
        # Create log directory if it doesn't exist
        log_dir = os.path.dirname(log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        
        # Create rotating file handler
        file_handler = logging.handlers.RotatingFileHandler(
            log_file, maxBytes=max_bytes, backupCount=backup_count
        )
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Add new handlers
    for handler in handlers:
        root_logger.addHandler(handler)
    
    # Add sensitive data filter
    sensitive_filter = SensitiveDataFilter()
    for handler in root_logger.handlers:
        handler.addFilter(sensitive_filter)
    
    # Configure library loggers
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.error").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy.engine").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy.pool").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy.orm").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy.dialects").setLevel(logging.WARNING)
    logging.getLogger("alembic").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.upgrade").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.downgrade").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.revision").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.script").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.render").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.rewriter").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.api").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.metadata").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.type").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.server_default").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.constraint").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.index").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.column").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.table").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.schema").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.postgresql").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.mysql").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.sqlite").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.mssql").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.oracle").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.firebird").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.sybase").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.postgresql.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.mysql.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.sqlite.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.mssql.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.oracle.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.firebird.compare").setLevel(logging.WARNING)
    logging.getLogger("alembic.runtime.migration.autogenerate.compare.dialect.sybase.compare").setLevel(logging.WARNING)
    
    # Log configuration
    logging.info(f"Logging configured with level {log_level}")


def get_request_logger(request_id: Optional[str] = None) -> logging.Logger:
    """
    Get a logger with request ID filter.
    
    Args:
        request_id: The request ID to use. If None, a new UUID is generated.
        
    Returns:
        logging.Logger: The logger with request ID filter.
    """
    logger = logging.getLogger("app")
    logger.addFilter(RequestIdFilter(request_id))
    return logger
