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
    author_email="info@bulo.cloud",
    description="Web interface for BuloCloudSentinel, based on OpenWebUI",
    keywords="drone, surveillance, web interface",
    python_requires=">=3.9",
)
