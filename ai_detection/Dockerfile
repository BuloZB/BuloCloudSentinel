# AI Detection Service Dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY ai_detection/requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

COPY ai_detection/ .

EXPOSE 5000

CMD ["python", "detect.py"]
