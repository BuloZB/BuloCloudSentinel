"""
Setup script for the Drone Show microservice.
"""

from setuptools import setup, find_packages

setup(
    name="drone_show_service",
    version="0.1.0",
    description="Drone Show microservice for Bulo.Cloud Sentinel",
    author="Bulo.Cloud Sentinel Team",
    author_email="info@bulocloud-sentinel.example.com",
    url="https://github.com/BuloZB/BuloCloudSentinel",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "fastapi>=0.95.0",
        "uvicorn>=0.21.1",
        "pydantic>=1.10.7",
        "sqlalchemy>=2.0.9",
        "asyncpg>=0.27.0",
        "aioboto3>=11.2.0",
        "httpx>=0.24.0",
        "websockets>=11.0.2",
        "numpy>=1.24.3",
        "python-multipart>=0.0.18",  # Updated from 0.0.6 to fix CVE-2024-53981
        # Replacing python-jose with PyJWT due to vulnerabilities CVE-2024-33664 and CVE-2024-33663
        # "python-jose>=3.3.0",
        "pyjwt>=2.10.1",
        "passlib>=1.7.4",
        "bcrypt>=4.0.1",
        "pymavlink>=2.4.37",
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.9",
    entry_points={
        "console_scripts": [
            "drone-show-service=drone_show_service.run:main",
        ],
    },
)
argon2-cffi==23.1.0  # For secure password hashing
pydantic==2.11.4  # For secure data validation
python-magic==0.4.27  # For secure file type detection
safety==2.3.5  # For dependency vulnerability scanning
bandit==1.7.7  # For security static analysis
cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119
pyopenssl==24.0.0  # Secure version
