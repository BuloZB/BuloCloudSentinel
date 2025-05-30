from fastapi import FastAPI
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from sqlalchemy import Engine

from open_webui.utils.telemetry.exporters import LazyBatchSpanProcessor
from open_webui.utils.telemetry.instrumentors import Instrumentor
from open_webui.env import OTEL_SERVICE_NAME, OTEL_EXPORTER_OTLP_ENDPOINT


def setup(app: FastAPI, db_engine: Engine):
    # set up trace
    trace.set_tracer_provider(
        TracerProvider(
            resource=Resource.create(attributes={SERVICE_NAME: OTEL_SERVICE_NAME})
        )
    )
    # otlp export
    exporter = OTLPSpanExporter(endpoint=OTEL_EXPORTER_OTLP_ENDPOINT)
    trace.get_tracer_provider().add_span_processor(LazyBatchSpanProcessor(exporter))
    Instrumentor(app=app, db_engine=db_engine).instrument()
argon2-cffi==23.1.0  # For secure password hashing
pydantic==2.11.4  # For secure data validation
python-magic==0.4.27  # For secure file type detection
safety==2.3.5  # For dependency vulnerability scanning
bandit==1.7.7  # For security static analysis
cryptography==46.0.0  # Updated to latest version to fix CVE-2024-26130, CVE-2024-12797, CVE-2024-6119
pyopenssl==24.0.0  # Secure version
