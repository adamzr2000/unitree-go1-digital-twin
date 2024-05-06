import cv2
import requests
import os
import argparse

def send_frame_to_server(frame, url):
    # Encode frame to JPEG format
    # _, buffer = cv2.imencode('.jpg', frame)
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])  # Lower quality (less compression)

    # Send this frame as a POST request to the server
    response = requests.post(url, data=buffer.tobytes())
    print(f"Sent frame to {url}; Status Code: {response.status_code}")

def main(server_url):
    # Start video capture from the webcam and set resolution to 1280x720
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Send the frame to the server
            send_frame_to_server(frame, server_url)

    finally:
        # When everything done, release the capture
        cap.release()
        print("Released Video Resource")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send webcam video to server.')
    parser.add_argument('--server_url', type=str, help='URL of the server to send video frames to.')
    args = parser.parse_args()

    # Check if a server URL was provided as an argument, otherwise look for an environment variable
    server_url = args.server_url if args.server_url else os.getenv('SERVER_URL')

    if not server_url:
        raise ValueError("No server URL provided. Set the SERVER_URL environment variable or use the --server_url argument.")

    main(server_url)
