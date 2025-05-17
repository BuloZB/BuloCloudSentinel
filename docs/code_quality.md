# Code Quality Guidelines

This document outlines the code quality standards and tools used in the BuloCloud Sentinel project.

## Table of Contents

- [Development Environment Setup](#development-environment-setup)
- [Code Style and Formatting](#code-style-and-formatting)
- [Static Analysis Tools](#static-analysis-tools)
- [Security Analysis](#security-analysis)
- [Testing](#testing)
- [Pre-commit Hooks](#pre-commit-hooks)
- [CI/CD Pipeline](#cicd-pipeline)

## Development Environment Setup

### Prerequisites

- Python 3.12+
- Poetry 1.7.0+
- Node.js 18+
- Docker

### Installation

```bash
# Install dependencies with Poetry
poetry install --all-extras

# Install pre-commit hooks
poetry run pre-commit install
```

## Code Style and Formatting

### Python

- **Black**: Automatic code formatter with line length of 100
- **isort**: Import sorter configured to be compatible with Black
- **Ruff**: Fast Python linter that replaces multiple tools

### JavaScript/TypeScript

- **ESLint**: Linting for JavaScript/TypeScript
- **Prettier**: Code formatting

## Static Analysis Tools

### Ruff

Ruff is a fast Python linter that combines the functionality of multiple tools.

```bash
# Run Ruff linter
poetry run ruff check .

# Run Ruff with auto-fix
poetry run ruff check --fix .
```

### Black

Black is an uncompromising code formatter.

```bash
# Format code with Black
poetry run black .
```

### isort

isort sorts imports alphabetically and automatically separated into sections.

```bash
# Sort imports with isort
poetry run isort .
```

### mypy

mypy is a static type checker for Python.

```bash
# Run mypy type checking
poetry run mypy .
```

### flake8-bugbear

flake8-bugbear is a plugin for Flake8 that finds likely bugs and design problems.

```bash
# Run flake8 with bugbear
poetry run flake8 .
```

## Security Analysis

### Bandit

Bandit is a tool designed to find common security issues in Python code.

```bash
# Run Bandit security analysis
poetry run bandit -r .
```

### Safety

Safety checks Python dependencies for known security vulnerabilities.

```bash
# Check dependencies for vulnerabilities
poetry run safety check
```

### Semgrep

Semgrep is a lightweight static analysis tool for finding bugs and enforcing code standards.

```bash
# Run Semgrep with CI ruleset
poetry run semgrep --config=p/ci .
```

### Trivy

Trivy is a comprehensive security scanner for containers and other artifacts.

```bash
# Scan Docker image
trivy image bulocloud-sentinel:latest
```

## Testing

### Python Tests

```bash
# Run Python tests with coverage
poetry run pytest --cov=backend --cov-report=term-missing --cov-fail-under=90
```

### JavaScript/TypeScript Tests

```bash
# Run frontend tests
cd frontend && npm test
```

## Pre-commit Hooks

Pre-commit hooks are configured to run automatically before each commit to ensure code quality.

```bash
# Run all pre-commit hooks manually
poetry run pre-commit run --all-files
```

## CI/CD Pipeline

Our CI/CD pipeline runs the following checks:

1. Code formatting and linting
2. Static type checking
3. Security analysis
4. Unit and integration tests
5. Docker image scanning

All these checks must pass before code can be merged into the main branch.

## Acceptance Criteria

- `poetry run ruff check .` returns 0
- `poetry run mypy .` passes in strict mode
- `pytest` passes with coverage ≥ 90%
- `bandit -r .` returns no high/critical issues
- Docker image scan (`trivy image`) reports 0 CRITICAL vulnerabilities
- All GitHub Actions jobs succeed
