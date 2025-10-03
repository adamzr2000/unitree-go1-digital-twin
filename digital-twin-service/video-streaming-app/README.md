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
🔗 **Access Stream:** [http://127.0.0.1:8889/go1_camera](http://127.0.0.1:8889/go1_camera)  

### **Stop the System**
```bash
docker compose down
```