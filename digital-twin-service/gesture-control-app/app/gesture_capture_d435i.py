import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp
import os

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

path_to_directory = '/home/go1/app/'
if not os.path.exists(path_to_directory):
    os.makedirs(path_to_directory)
file_path = os.path.join(path_to_directory, 'command.txt')

with mp_hands.Hands(
    max_num_hands=1,
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    cv2.namedWindow('MediaPipe Hands', cv2.WINDOW_NORMAL)  # Create a resizable window
    cv2.resizeWindow('MediaPipe Hands', 1280, 960)  # Set the initial size of the window

    while True:
        frame = pipe.wait_for_frames()
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    color_image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())

                # Extracting landmark coordinates and calculating parameters for gestures
                Wrist = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y]
                Thumb_cmc = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].y]
                Thumb_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y]
                Thumb_ip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y]
                Thumb_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y]
                Index_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y]
                Index_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y]
                Index_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y]
                Index_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y]
                Middle_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y]
                Middle_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y]
                Middle_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y]
                Middle_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y]
                ring_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y]
                ring_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y]
                ring_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y]
                ring_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y]
                pinky_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y]
                pinky_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y]
                pinky_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].y]
                pinky_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y]

                mcp_delta_y = abs(pinky_mcp[1] - Index_mcp[1])
                wrist_to_mcp_y = abs(Middle_mcp[1] - Wrist[1])
                wrist_to_mcp_x = abs(Middle_mcp[0] - Wrist[0])
                hands_orient_param = mcp_delta_y/wrist_to_mcp_y
                orient_ratio = wrist_to_mcp_x/wrist_to_mcp_y

                hands_x_side = Wrist[0] - Middle_mcp[0]
                tip_mcp_middle_delta_y = Middle_tip[1]-Middle_mcp[1]
                tip_mcp_middle_delta_x = Middle_tip[0]-Middle_mcp[0]
                
                tip_mcp_index_delta_y = Index_tip[1] - Index_mcp[1]
                tip_mcp_index_delta_x = Index_tip[0] - Index_mcp[0]
                
                tip_pip_middle_delta_x =Middle_tip[0]-Middle_pip[0]
                tip_pip_index_delta_x = Index_tip[0]-Index_pip[0]

                with open(file_path, 'w') as f:
                    if orient_ratio <0.5:
                        print('normal')
                        if tip_mcp_middle_delta_y >0:
                            if tip_mcp_index_delta_y<0:
                                print('up')
                                f.write("u")
                            else:
                                print('gum')
                                f.write("d")
                        else:
                            print('bae')
                            f.write("n")
                    else:
                        print('parallel')
                        if hands_x_side <0:
                            #left
                            if tip_pip_middle_delta_x >0:
                                print('bae')
                                f.write("n")
                            else:
                                if tip_pip_index_delta_x>0:
                                    print('left')
                                    f.write("l")
                                else:
                                    print('gum')
                                    f.write("d")
                        
                        else:
                            #rigth
                            if tip_pip_middle_delta_x <0:
                                print('bae')
                                f.write("n")
                            else:
                                if tip_pip_index_delta_x<0:
                                    print('right')
                                    f.write("r")
                                else:
                                    print('gum')
                                    f.write("d")

        else:
            with open(file_path, 'w') as f:
                # Write a neutral command when no hands are detected
                f.write("n")

        cv2.imshow('MediaPipe Hands', color_image)

        if cv2.waitKey(1) == ord('q'):
            break

pipe.stop()
cv2.destroyAllWindows()
