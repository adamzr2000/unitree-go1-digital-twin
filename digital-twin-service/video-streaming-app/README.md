# Low Latency Video Streaming App

A real-time, low-latency video streaming system designed for edge-to-cloud robotics using Docker and Kubernetes. It supports MJPEG and H.264 encoding with SRT delivery and WebRTC playback.

![Video Streaming App Architecture](video-streaming-app-config.png)

1. **Streamer**: Captures video from a webcam and encodes it in either MJPEG or H.264, depending on the selected mode. The encoded stream is then transmitted over UDP (port 554) to the `receiver-transcoder` component.
2. **Receiver-Transcoder**: Receives the incoming video stream, decodes it, and re-encodes it as H.264 with the desired bitrate and resolution. The processed stream is then transmitted via SRT (port 8890) to the `MediaMTX` server.
3. **MediaMTX**: Functions as a media server, receiving the H.264 SRT stream and distributing it to clients via WebRTC, RTSP, or HLS.

---

## 👨‍💻 Author

**Adam Zahir Rodriguez** 

---

## **Quick Start**  
### **Start the System**
```bash
docker compose up -d
```
🔗 **Access Stream:** [http://127.0.0.1:8889/test_stream](http://127.0.0.1:8889/test_stream)  

### **Stop the System**
```bash
docker compose down
```

---

## **Usage (5TONIC Setup)**  

### **Using MJPEG Encoding**  
#### **Edge Server (Receiver-Transcoder)**
```bash
./run_example_edge.sh --stream-name go1_camera --enable-monitoring --export-to-influxdb --webrtc-additional-hosts 10.5.1.21
```
**Stop the process:**
```bash
./stop_example_edge.sh
```

#### **Robot (Streamer)**
```bash
cd streamer
./run_example.sh --width 640 --height 480 --framerate 30 --receiver-ip 10.11.7.4 --receiver-port 5554 --use-d435i
```
**Stop the streamer:**
```bash
docker kill video-streamer
```

---

### **Using H.264 Encoding**  
#### **Edge Server (Receiver-Transcoder)**
```bash
./run_example_edge.sh --stream-name go1_camera --enable-monitoring --webrtc-additional-hosts 10.5.1.21 --use-h264
```
**Stop the process:**
```bash
./stop_example_edge.sh
```

#### **Robot (Streamer)**
```bash
cd streamer
./run_example.sh --width 1280 --height 720 --framerate 30 --receiver-ip 10.11.7.4 --receiver-port 5554 --use-d435i --use-h264 --bitrate 5000
```
**Stop the streamer:**
```bash
docker kill video-streamer
```
