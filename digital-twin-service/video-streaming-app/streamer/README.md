# **Usage**  

Run the script with the desired options:  

- **Basic streaming (MJPEG)**  
  ```bash
  ./run_example.sh --width 1280 --height 720 --framerate 30 --receiver-ip 10.5.1.21 --receiver-port 5554 --device /dev/video0
  ```

- **Using Intel RealSense D435i (MJPEG)**  
  ```bash
  ./run_example.sh --width 1280 --height 720 --framerate 30 --receiver-ip 10.5.1.21 --receiver-port 5554 --use-d435i
  ```

- **Using D435i with H.264 Encoding & Custom Bitrate**  
  ```bash
  ./run_example.sh --width 1280 --height 720 --framerate 30 --receiver-ip 10.5.1.21 --receiver-port 5554 --use-d435i --use-h264 --bitrate 4000
  ```

- **Static Stream (Predefined Configuration)**  
  ```bash
  ./run_example_static.sh --receiver-ip 10.5.1.21 --receiver-port 5554
  ```