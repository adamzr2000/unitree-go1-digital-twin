# Use a lightweight Python image
FROM python:3.10-slim

COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

# Install system dependencies required for confluent-kafka
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    librdkafka-dev \
    && rm -rf /var/lib/apt/lists/*  # Clean up to reduce image size

# Set working directory inside container
WORKDIR /app

COPY app/ /app/

# Declare a volume for configuration
VOLUME ["/config"]

# Define entrypoint for flexible script execution
ENTRYPOINT ["python3"]
