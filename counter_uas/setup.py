"""
Setup script for the Counter-UAS module.
"""

from setuptools import setup, find_packages

setup(
    name="counter_uas",
    version="0.1.0",
    description="Counter-UAS / Intrusion Detection Module for Bulo.Cloud Sentinel",
    author="Bulo.Cloud Sentinel Team",
    author_email="info@bulo.cloud",
    url="https://github.com/BuloZB/BuloCloudSentinel",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.20.0",
        "scipy>=1.7.0",
        "matplotlib>=3.4.0",
        "pika>=1.2.0",
        "fastapi>=0.68.0",
        "uvicorn>=0.15.0",
        "pyyaml>=6.0",
        "pytest>=6.2.5",
    ],
    extras_require={
        "dev": [
            "pytest>=6.2.5",
            "pytest-asyncio>=0.15.1",
            "pytest-cov>=2.12.1",
            "black>=21.6b0",
            "isort>=5.9.2",
            "mypy>=0.910",
            "flake8>=3.9.2",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
    python_requires=">=3.8",
)
