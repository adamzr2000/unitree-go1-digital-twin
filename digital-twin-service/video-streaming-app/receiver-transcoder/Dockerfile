# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Install dependencies
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    gstreamer1.0-rtsp \
    libgirepository1.0-dev \
    gcc \
    libcairo2-dev \
    pkg-config \
    python3-dev \
    gir1.2-gtk-4.0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY app/video_receiver_transcoder.py .

# Install necessary Python packages
RUN pip3 install pycairo PyGObject influxdb-client

# Set the default command to run the script
CMD ["python3", "-u", "video_receiver_transcoder.py"]