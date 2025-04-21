# Contributing to SentinelWeb

Thank you for your interest in contributing to SentinelWeb! This document provides guidelines and instructions for contributing to the project.

## Code of Conduct

Please read and follow our [Code of Conduct](CODE_OF_CONDUCT.md).

## How to Contribute

### Reporting Bugs

1. Check if the bug has already been reported in the Issues section.
2. If not, create a new issue with a clear title and description.
3. Include steps to reproduce the bug, expected behavior, and actual behavior.
4. Include screenshots or logs if applicable.

### Suggesting Features

1. Check if the feature has already been suggested in the Issues section.
2. If not, create a new issue with a clear title and description.
3. Explain why the feature would be useful to most users.
4. Provide examples of how the feature would work.

### Pull Requests

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes and commit them with clear, descriptive commit messages.
4. Write or update tests for your changes.
5. Run the tests to make sure they pass.
6. Submit a pull request with a clear title and description.

## Development Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/sentinelweb.git
   cd sentinelweb
   ```

2. Set up the development environment:
   - On Windows:
     ```bash
     run.bat
     ```
   - On Linux/macOS:
     ```bash
     chmod +x run.sh
     ./run.sh
     ```

3. Make your changes and test them locally.

## Coding Standards

- Follow PEP 8 style guide for Python code.
- Write docstrings for all functions, classes, and modules.
- Write unit tests for all new functionality.
- Keep functions small and focused on a single task.
- Use meaningful variable and function names.

## Testing

Run the tests to make sure your changes don't break existing functionality:

- On Windows:
  ```bash
  run_tests.bat
  ```
- On Linux/macOS:
  ```bash
  chmod +x run_tests.sh
  ./run_tests.sh
  ```

## Documentation

- Update documentation for any changes to the API or functionality.
- Write clear and concise documentation that is easy to understand.
- Include examples where appropriate.

## Review Process

1. All pull requests will be reviewed by at least one maintainer.
2. Feedback will be provided on the pull request.
3. Changes may be requested before the pull request is merged.
4. Once approved, the pull request will be merged by a maintainer.

## License

By contributing to SentinelWeb, you agree that your contributions will be licensed under the project's [MIT License](LICENSE).
