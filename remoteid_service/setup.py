"""
Setup script for the Remote ID & Regulatory Compliance Service.
"""

import os
from setuptools import setup, find_packages

# Read the contents of the README file
with open(os.path.join(os.path.dirname(__file__), "README.md"), encoding="utf-8") as f:
    long_description = f.read()

# Read the requirements file
with open(os.path.join(os.path.dirname(__file__), "requirements.txt"), encoding="utf-8") as f:
    requirements = f.read().splitlines()

setup(
    name="remoteid-service",
    version="1.0.0",
    description="Remote ID & Regulatory Compliance Service for Bulo.Cloud Sentinel",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Bulo.Cloud Sentinel Team",
    author_email="support@bulo.cloud",
    url="https://github.com/BuloZB/BuloCloudSentinel",
    packages=find_packages(exclude=["tests", "tests.*"]),
    include_package_data=True,
    install_requires=requirements,
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    entry_points={
        "console_scripts": [
            "remoteid=remoteid_service.cli.remoteid:main",
            "flightplan=remoteid_service.cli.flightplan:main",
            "notam=remoteid_service.cli.notam:main",
            "remoteid-simulator=remoteid_service.cli.simulator:main",
        ],
    },
)
