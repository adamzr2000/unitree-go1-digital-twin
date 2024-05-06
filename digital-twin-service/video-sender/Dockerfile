# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user 'go1' with sudo privileges and set a password
RUN useradd -m go1 && \
    echo "go1:go1" | chpasswd && adduser go1 sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

RUN pip3 install opencv-python-headless requests

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1

RUN mkdir -p /home/go1/app
COPY app/ /home/go1/app/
WORKDIR /home/go1/app/

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

# Default command, can be overridden by command line args
CMD ["python3", "client.py"]
