import cv2
import requests
import os
import argparse

def send_frame_to_server(frame, url, session):
    # Encode frame to JPEG format with lower quality (more compression)
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # Use session to send frame as POST request
    response = session.post(url, data=buffer.tobytes())
    print(f"Sent frame to {url}; Status Code: {response.status_code}")


def main(server_url):
    # Start video capture from the webcam and set resolution to 1280x720
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    session = requests.Session()

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Send the frame to the server
            send_frame_to_server(frame, server_url, session)

    finally:
        cap.release()
        print("Released Video Resource")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send webcam video to server.')
    parser.add_argument('--server_url', type=str, help='URL of the server to send video frames to.')
    args = parser.parse_args()

    server_url = args.server_url if args.server_url else os.getenv('SERVER_URL')

    if not server_url:
        raise ValueError("No server URL provided. Set the SERVER_URL environment variable or use the --server_url argument.")

    main(server_url)
