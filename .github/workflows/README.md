# GitHub Workflows Documentation

This document provides comprehensive information about the GitHub Actions workflows in this repository, including best practices, troubleshooting, and optimization strategies.

## Workflow Overview

| Workflow | Purpose | Trigger | Key Dependencies |
|----------|---------|---------|------------------|
| CI | Basic continuous integration | Push to main, PR, daily | PostgreSQL, Node.js |
| Unit Tests | Run unit tests | Daily, manual | pytest |
| Integration Tests | Run integration tests | Daily, manual | pytest, PostgreSQL |
| Docker Build | Build Docker images | Daily, manual | Docker, Buildx |
| Security Scan | Security vulnerability scanning | Daily, manual | safety, bandit, npm audit, Trivy |
| Bandit | Python security scanning | Push to main, PR, daily | bandit |
| CodeQL | Code quality and security | Push to main, PR, weekly | CodeQL |
| Safety | Dependency vulnerability scanning | Push to main, PR, daily | safety |
| Daily Tests | Comprehensive test suite | Daily, manual | pytest, npm test |

## Common Issues and Solutions

### Missing Files or Dependencies

1. **Missing requirements-secure.txt**
   - **Issue**: Workflows reference a non-existent requirements-secure.txt file
   - **Solution**: We've created this file with security-focused dependencies

2. **Missing Dockerfiles**
   - **Issue**: Docker build workflows fail when Dockerfiles don't exist
   - **Solution**: We've added checks to skip builds for missing Dockerfiles and created missing ones

3. **Missing Test Files**
   - **Issue**: Test workflows fail when no tests exist
   - **Solution**: We've added checks to create empty coverage reports when tests don't exist

### Configuration Issues

1. **Slack Notifications**
   - **Issue**: Workflows fail when SLACK_WEBHOOK secret is not configured
   - **Solution**: We've added conditional checks to skip notification steps when the secret is missing

2. **Format Conversion**
   - **Issue**: Bandit outputs JSON but CodeQL expects SARIF
   - **Solution**: We've added a conversion step to transform JSON to SARIF format

## Optimization Strategies

### Caching

All workflows should use caching to improve performance:

```yaml
- name: Cache dependencies
  uses: actions/cache@v3
  with:
    path: ~/.cache/pip
    key: ${{ runner.os }}-pip-${{ hashFiles('**/requirements.txt') }}
    restore-keys: |
      ${{ runner.os }}-pip-
```

### Parallelization

Split large test suites into parallel jobs:

```yaml
jobs:
  test:
    strategy:
      matrix:
        test-group: [group1, group2, group3]
    steps:
      - name: Run tests
        run: pytest tests/${{ matrix.test-group }}
```

### Conditional Execution

Use conditional execution to skip unnecessary steps:

```yaml
- name: Run expensive step
  if: github.event_name == 'push' && github.ref == 'refs/heads/main'
  run: ./expensive-operation.sh
```

## Security Considerations

1. **Secrets Management**
   - Never log secrets or sensitive data
   - Use GitHub Secrets for all credentials
   - Rotate secrets regularly

2. **Dependency Scanning**
   - Regular scanning with safety, npm audit, and Trivy
   - Automatic updates with Dependabot

3. **Code Scanning**
   - Regular scanning with CodeQL and Bandit
   - Review and fix all high-severity issues promptly

## Best Practices

1. **Workflow Organization**
   - Use numbered prefixes for execution order
   - Group related workflows in subdirectories
   - Use consistent naming conventions

2. **Error Handling**
   - Use `continue-on-error` for non-critical steps
   - Provide clear error messages
   - Upload artifacts for debugging

3. **Performance**
   - Minimize unnecessary steps
   - Use the smallest possible runner
   - Cache dependencies and build artifacts

4. **Maintenance**
   - Review and update workflows quarterly
   - Remove unused workflows
   - Keep action versions up to date

## Troubleshooting Guide

1. **Workflow Fails with "File Not Found"**
   - Check if the file exists in the repository
   - Verify the file path is correct
   - Add conditional checks to handle missing files

2. **Docker Build Fails**
   - Verify Dockerfile exists
   - Check Docker syntax
   - Ensure all dependencies are available

3. **Test Coverage Reports Missing**
   - Verify tests are running correctly
   - Check coverage tool configuration
   - Ensure report paths are correct

4. **Security Scan False Positives**
   - Review and document known false positives
   - Use .banditignore or similar files
   - Adjust severity thresholds as needed