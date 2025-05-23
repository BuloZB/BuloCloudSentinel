"""
Setup script for the SATCOM / 5G Fallback Connectivity module.
"""

from setuptools import setup, find_packages

setup(
    name="comms_fallback",
    version="0.1.0",
    description="SATCOM / 5G Fallback Connectivity module for Bulo.Cloud Sentinel",
    author="Bulo.Cloud Sentinel Team",
    author_email="info@bulo.cloud",
    url="https://bulo.cloud",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "pyyaml>=6.0",
        "redis>=4.5.1",
        "pytest>=7.0.0",
        "pytest-asyncio>=0.20.0",
    ],
    entry_points={
        "console_scripts": [
            "comms-fallback=comms_fallback.main:run",
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
