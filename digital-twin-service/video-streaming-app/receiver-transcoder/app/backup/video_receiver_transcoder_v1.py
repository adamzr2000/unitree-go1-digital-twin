import gi
import os
import time
import threading
import traceback

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


def on_message(bus, message, loop):
    """Callback for GStreamer bus messages."""
    msg_type = message.type

    if msg_type == Gst.MessageType.EOS:
        print("[INFO] End of stream received.")
        loop.quit()
    elif msg_type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"[ERROR] {err.message}")
        print(f"[DEBUG] {debug}")
        loop.quit()
    elif msg_type == Gst.MessageType.WARNING:
        warn, debug = message.parse_warning()
        print(f"[WARNING] {warn.message}")
        print(f"[DEBUG] {debug}")


def pad_probe(pad, info, data):
    """Pad probe to track the size of packets."""
    buffer = info.get_buffer()
    if buffer:
        with data["lock"]:
            data["bytes"] += buffer.get_size()  # Increment bytes
            data["frames"] += 1  # Increment frame count
    return Gst.PadProbeReturn.OK


def calculate_bandwidth(data):
    """Calculate and display bandwidth and frame statistics."""
    start_time = time.time()

    while not data["stop"]:
        time.sleep(1)  # Calculate every second
        elapsed_time = time.time() - start_time
        start_time = time.time()

        with data["lock"]:
            bytes_received = data["bytes"]
            frames_received = data["frames"]
            data["bytes"] = 0
            data["frames"] = 0

        bandwidth = (bytes_received * 8) / (1_000_000 * elapsed_time)  # Mbps
        avg_frame_size = bytes_received / frames_received if frames_received > 0 else 0

        if frames_received > 0:
            print(f"[METRICS] Bandwidth: {bandwidth:.2f} Mbps, Frames: {frames_received}, Avg Frame Size: {avg_frame_size:.2f} bytes")
        else:
            print("[METRICS] No frames received in the last interval. Possible packet loss.")


def start_receiver(port, width, height, bitrate, speed_preset, srt_ip, srt_port, stream_name):
    """Sets up the pipeline to receive MJPG, encode to H.264, and send via SRT."""
    Gst.init(None)

    pipeline_desc = (
        f'udpsrc port={port} ! '
        'application/x-rtp, encoding-name=JPEG, payload=26 ! '
        'rtpjpegdepay ! '
        'jpegdec ! '
        'videoconvert ! '
        'videoscale ! '
        f'video/x-raw, width={width}, height={height} ! '
        f'x264enc name=my_enc bitrate={bitrate} '
        f'speed-preset={speed_preset} key-int-max=10 '
        'bframes=0 tune=zerolatency ! '
        'h264parse ! '
        'mpegtsmux alignment=7 ! '
        f'srtsink uri="srt://{srt_ip}:{srt_port}?streamid=publish:{stream_name}" sync=false'
    )

    print("[PIPELINE] Pipeline description:")
    print(pipeline_desc)

    pipeline = Gst.parse_launch(pipeline_desc)

    # Set up bus to handle messages
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    loop = GLib.MainLoop()
    bus.connect("message", on_message, loop)

    # Metrics data
    data = {
        "bytes": 0,
        "frames": 0,
        "stop": False,
        "lock": threading.Lock()
    }

    # Add probe to measure metrics
    sink_pad = pipeline.get_by_name("my_enc").get_static_pad("src")
    sink_pad.add_probe(Gst.PadProbeType.BUFFER, pad_probe, data)

    # Start metrics thread
    metrics_thread = threading.Thread(target=calculate_bandwidth, args=(data,))
    metrics_thread.start()

    try:
        print("[INFO] Starting video receiver and transcoder:")
        print(f"Receiving MJPG stream on UDP port {port}.")
        print(f"Decoding MJPG to raw video frames.")
        print(f"Encoding to H.264 with resolution {width}x{height}, bitrate {bitrate} kbps, and speed preset {speed_preset}.")
        print(f"Publishing as SRT to srt://{srt_ip}:{srt_port}?streamid=publish:{stream_name}.")
        pipeline.set_state(Gst.State.PLAYING)
        loop.run()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user, stopping...")
    except Exception as e:
        print(f"[ERROR] Exception occurred: {e}")
        traceback.print_exc()
    finally:
        data["stop"] = True
        metrics_thread.join()
        pipeline.set_state(Gst.State.NULL)
        loop.quit()
        print("[INFO] Pipeline stopped.")


if __name__ == "__main__":
    # Read environment variables
    port = int(os.getenv("RECEIVER_PORT", 5554))
    width = int(os.getenv("WIDTH", 640))
    height = int(os.getenv("HEIGHT", 480))
    bitrate = int(os.getenv("BITRATE", 2000))
    speed_preset = os.getenv("SPEED_PRESET", "medium")
    srt_ip = os.getenv("SRT_IP", "127.0.0.1")
    srt_port = int(os.getenv("SRT_PORT", 8890))
    stream_name = os.getenv("STREAM_NAME", "my_stream")

    print("[CONFIG] Video Receiver and Transcoder Configuration:")
    print(f"UDP Port for MJPG input: {port}")
    print(f"Output Resolution: {width}x{height}")
    print(f"H.264 Bitrate: {bitrate} kbps")
    print(f"Encoding Speed Preset: {speed_preset}")
    print(f"SRT Destination IP: {srt_ip}")
    print(f"SRT Destination Port: {srt_port}")
    print(f"Stream Name: {stream_name}")

    # Start the receiver
    start_receiver(port, width, height, bitrate, speed_preset, srt_ip, srt_port, stream_name)
