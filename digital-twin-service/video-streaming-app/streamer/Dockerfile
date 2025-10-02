# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Install necessary packages and clean up
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
    libgirepository1.0-dev \
    gcc \
    libcairo2-dev \
    pkg-config \
    python3-dev \
    gir1.2-gtk-4.0 \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /app

COPY app/video_streamer.py .

# Install RealSense Python bindings
RUN pip3 install pyrealsense2 numpy opencv-python Flask && \
    pip3 install pycairo PyGObject pyzmq

CMD ["python3", "-u", "video_streamer.py"]
