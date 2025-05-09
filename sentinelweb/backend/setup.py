from setuptools import setup, find_packages

setup(
    name="sentinel_web",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "fastapi>=0.95.0",
        "uvicorn>=0.21.1",
        "httpx>=0.24.0",
        "python-multipart>=0.0.6",
        "pydantic>=2.0.0",
        "starlette>=0.27.0",
        "itsdangerous>=2.1.2",
    ],
    author="BuloCloud",
    author_email="info@bulocloud.com",
    description="Web interface for BuloCloudSentinel, based on OpenWebUI",
    keywords="drone, surveillance, web interface",
    python_requires=">=3.9",
)
argon2-cffi==23.1.0  # For secure password hashing
pydantic==2.11.4  # For secure data validation
python-magic==0.4.27  # For secure file type detection
safety==2.3.5  # For dependency vulnerability scanning
bandit==1.7.7  # For security static analysis
cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119
pyopenssl==24.0.0  # Secure version
