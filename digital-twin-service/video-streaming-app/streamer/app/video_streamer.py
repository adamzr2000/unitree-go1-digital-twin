import gi
import os
import traceback
import logging

import subprocess
import re

import pyrealsense2 as rs
import cv2
import numpy as np
import time
import signal
import sys

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize logging
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

def find_realsense_color_camera():
    """Automatically identify the RealSense color camera device."""
    try:
        result = subprocess.run(["v4l2-ctl", "--list-devices"], text=True, capture_output=True, check=True)
        devices_output = result.stdout

        realsense_devices = []
        current_device = None
        for line in devices_output.splitlines():
            if "RealSense" in line:
                current_device = []
            elif current_device is not None:
                match = re.search(r"/dev/video\d+", line)
                if match:
                    current_device.append(match.group(0))
                if not line.strip():  # End of device section
                    realsense_devices.extend(current_device)
                    current_device = None

        for device in realsense_devices:
            try:
                result = subprocess.run(
                    ["v4l2-ctl", f"--device={device}", "--list-formats-ext"],
                    text=True, capture_output=True, check=True,
                )
                if "YUYV" in result.stdout or "MJPEG" in result.stdout:
                    # logger.info(f"Color camera found: {device}")
                    return device
            except subprocess.CalledProcessError:
                pass

        logger.error("No RealSense color camera with YUYV format found.")
        return None
    except Exception as e:
        logger.error(f"An error occurred during detection: {e}")
        return None
    
def on_message(bus, message, loop):
    """Callback for GStreamer bus messages."""
    msg_type = message.type

    if msg_type == Gst.MessageType.EOS:
        logger.info("End of stream")
        loop.quit()
    elif msg_type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        logger.error(f"{err.message} ({debug})")
        loop.quit()
    elif msg_type == Gst.MessageType.WARNING:
        warn, debug = message.parse_warning()
        logger.warning(f"{warn.message} ({debug})")
    elif msg_type == Gst.MessageType.STATE_CHANGED:
        old, new, _ = message.parse_state_changed()
        logger.info(f"'{message.src.get_name()}' changed from {old.value_name} to {new.value_name}")
    elif msg_type == Gst.MessageType.BUFFERING:
        percent = message.parse_buffering()
        logger.info(f"{percent}%")
    else:
        logger.info(f"Message: {Gst.MessageType.get_name(msg_type)}")


# Global variables to track streaming state
pipeline = None
gst_process = None

def shutdown_handler(signum, frame):
    """Handles graceful shutdown of RealSense and GStreamer."""
    global pipeline, gst_process

    logger.info("Stopping RealSense streaming and GStreamer.")

    # Stop GStreamer process if running
    if gst_process and gst_process.poll() is None:
        gst_process.terminate()
        gst_process.wait()

    # Stop RealSense pipeline if running
    if pipeline:
        try:
            pipeline.stop()
        except RuntimeError as e:
            logger.warning(f"RealSense already stopped: {e}")

    logger.info("Streaming stopped cleanly.")
    sys.exit(0)  # Properly exit without raising SystemExit exception

def start_streaming_d435i(width, height, framerate, host, port, use_h264, bitrate):
    """Uses RealSense SDK and GStreamer Subprocess for stable streaming."""
    global pipeline, gst_process

    logger.info("Starting D435i streaming with RealSense SDK.")

    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, framerate)
    profile = pipeline.start(config)

    # Get connected device information
    device = profile.get_device()
    logger.info(f"Connected to device: {device.get_info(rs.camera_info.name)}")
    logger.info(f"Serial number: {device.get_info(rs.camera_info.serial_number)}")
    logger.info(f"Firmware version: {device.get_info(rs.camera_info.firmware_version)}")

    # Choose encoding method
    if use_h264:
        logger.info("Using H.264 encoding (x264enc)")
        gst_command = (
            f"gst-launch-1.0 -v fdsrc ! image/jpeg, width={width}, height={height}, framerate={framerate}/1 ! "
            "jpegparse ! jpegdec ! video/x-raw,format=I420 ! queue ! "
            f"x264enc bframes=0 tune=zerolatency byte-stream=true bitrate={bitrate} speed-preset=ultrafast key-int-max=10 ! "
            "h264parse ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={host} port={port} sync=false"
        )
    else:
        logger.info("Using MJPEG encoding (default)")
        gst_command = (
            f"gst-launch-1.0 -v fdsrc ! image/jpeg, width={width}, height={height}, framerate={framerate}/1 ! "
            f"jpegparse ! queue max-size-buffers=5 max-size-bytes=500000 max-size-time=2000000000 ! rtpjpegpay ! "
            f"udpsink host={host} port={port} sync=false"
        )

    logger.info(f"Starting GStreamer pipeline:\n{gst_command}")

    # Start GStreamer as a subprocess (binary mode, capture stderr)
    gst_process = subprocess.Popen(
        gst_command, shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE
    )

    # Allow GStreamer to initialize fully
    time.sleep(1)

    # Register SIGINT handler
    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())

            # Encode as JPEG
            success, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not success:
                continue

            # Write to GStreamer's stdin in BINARY mode
            try:
                gst_process.stdin.write(encoded_frame.tobytes())  # Ensure binary write
                gst_process.stdin.flush()
            except BrokenPipeError:
                logger.error("GStreamer pipeline broke (Broken pipe). Exiting.")
                break

    except KeyboardInterrupt:
        shutdown_handler(None, None)

    finally:
        # Stop processes only if they haven't been stopped yet
        if gst_process and gst_process.poll() is None:
            gst_process.terminate()
            gst_process.wait()

        if pipeline:
            try:
                pipeline.stop()
            except RuntimeError as e:
                logger.warning(f"RealSense pipeline already stopped: {e}")

        logger.info("Pipeline shut down successfully.")

def start_streaming(device, width, height, framerate, host, port, use_h264, bitrate):
    """Start streaming video over UDP."""
    Gst.init(None)
    if use_h264:
        pipeline_desc = (
            f"v4l2src device={device} ! "
            f"image/jpeg, width={width}, height={height}, framerate={framerate}/1 ! "
            "jpegdec ! "
            "queue ! "
            f"x264enc bframes=0 tune=zerolatency bitrate={bitrate} speed-preset=ultrafast key-int-max=10 ! "
            "h264parse ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={host} port={port} sync=false"
        )
        
    else:
        pipeline_desc = (
            f"v4l2src device={device} ! "
            f"image/jpeg, width={width}, height={height}, framerate={framerate}/1 ! "
            "queue ! "
            "rtpjpegpay ! "
            f"udpsink host={host} port={port} sync=false"
        )


    logger.info("Pipeline description:")
    print(pipeline_desc)
    pipeline = Gst.parse_launch(pipeline_desc)

    # Set up bus to handle messages
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    loop = GLib.MainLoop()
    bus.connect("message", on_message, loop)

    try:
        # Start the pipeline
        pipeline.set_state(Gst.State.PLAYING)
        logger.info(f"Streaming to {host}:{port} ... Press Ctrl+C to stop.")
        loop.run()
    except KeyboardInterrupt:
        logger.info("\n[INFO] Streaming interrupted. Shutting down...")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        traceback.print_exc()
    finally:
        pipeline.set_state(Gst.State.NULL)
        loop.quit()
        logger.info("Pipeline shut down.")

if __name__ == "__main__":
    use_d435i = os.getenv("USE_D435I", "False").lower() == "true"
    use_h264 = os.getenv("USE_H264", "False").lower() == "true"

    device = None
    if use_d435i:
        device = find_realsense_color_camera()
        if not device:
            logger.error("No suitable RealSense color camera found. Exiting.")
            exit(1)
    else:
        device = os.getenv("DEVICE", "/dev/video0")

    # Read environment variables
    width = int(os.getenv("WIDTH", 640))
    height = int(os.getenv("HEIGHT", 480))
    framerate = int(os.getenv("FRAMERATE", 30))
    host = os.getenv("RECEIVER_IP", "127.0.0.1")
    port = int(os.getenv("RECEIVER_PORT", 5554))
    bitrate = int(os.getenv("BITRATE", 2000))

    logger.info("Starting MJPG video stream with the following properties:")
    print(f"  Device:     {device}")
    print(f"  Resolution: {width}x{height}")
    print(f"  Framerate:  {framerate}")
    print(f"  Receiver:   {host}:{port}")
    print(f"  Use D435i:  {use_d435i}")
    if use_h264:
        print(f"  Use H264:  {use_h264}")
        print(f"  Bitrate:  {bitrate}")


    if use_d435i:
        start_streaming_d435i(width, height, framerate, host, port, use_h264, bitrate)
    else:
        start_streaming(device, width, height, framerate, host, port, use_h264, bitrate)
