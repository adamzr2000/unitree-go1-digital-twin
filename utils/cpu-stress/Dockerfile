FROM python:3.11-slim

LABEL maintainer="azahir@pa.uc3m.es"

# Set environment variables to avoid user interaction during the installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
      iproute2 iputils-ping ca-certificates curl \
  && rm -rf /var/lib/apt/lists/*
    
WORKDIR /app

# Copy the application code
COPY app/ .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Set the entrypoint to execute main.py
ENTRYPOINT ["python3", "-m", "uvicorn", "main:app", "--reload", "--host", "0.0.0.0", "--port", "5000"]