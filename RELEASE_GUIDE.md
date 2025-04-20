# Bulo.Cloud Sentinel - Release Guide

This document provides detailed instructions for creating and managing releases of the Bulo.Cloud Sentinel system on GitHub.

---

## Release Overview

A release is a snapshot of the project at a specific point in time, typically associated with a version number. Releases help users and developers track changes, download stable versions, and manage deployments.

---

## Creating a Release on GitHub

1. **Prepare the Codebase:**
   - Ensure all changes are committed and pushed to the `main` branch.
   - Verify that all CI tests have passed successfully.

2. **Tagging the Release:**
   - Use semantic versioning (e.g., `v1.0.0`).
   - Create a git tag locally:
     ```bash
     git tag -a v1.0.0 -m "Release version 1.0.0"
     git push origin v1.0.0
     ```
   - Alternatively, create a tag via GitHub UI when creating the release.

3. **Create the Release on GitHub:**
   - Navigate to the repository on GitHub.
   - Click on "Releases" > "Draft a new release".
   - Select the tag (or create a new one).
   - Provide a release title and detailed description including:
     - Summary of new features and fixes.
     - Important notes or breaking changes.
     - Upgrade or migration instructions.
   - Attach any compiled binaries or assets if applicable.
   - Publish the release.

---

## Versioning Strategy

- Follow [Semantic Versioning](https://semver.org/):
  - **MAJOR** version when you make incompatible API changes.
  - **MINOR** version when you add functionality in a backward-compatible manner.
  - **PATCH** version when you make backward-compatible bug fixes.

---

## Deployment from Release

- Use the tagged release to deploy the system in production.
- Pull the specific tag in your deployment environment:
  ```bash
  git checkout tags/v1.0.0
  ```
- Build and run Docker containers using the release code.

---

## Changelog Maintenance

- Maintain a `CHANGELOG.md` file documenting all changes per release.
- Include:
  - Added features
  - Fixed bugs
  - Known issues
  - Security updates

---

## Automation

- Consider automating release creation with GitHub Actions or other CI/CD tools.
- Automate changelog generation and tagging based on commit messages.

---

## Additional Notes

- Ensure all sensitive data is excluded from releases.
- Verify that documentation is updated and included.
- Communicate releases to stakeholders and users.

---

*This guide ensures consistent and reliable release management for Bulo.Cloud Sentinel.*
