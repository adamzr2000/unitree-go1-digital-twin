# Importing necessary libraries
import cv2  # OpenCV for image processing and computer vision tasks
import mediapipe as mp  # MediaPipe for hand tracking
import os

# Setting up MediaPipe drawing utilities for visualization
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Setting up webcam input
cap = cv2.VideoCapture(0) # Start capturing video from the webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)

# Define a specific path to save the file
# Replace 'path_to_directory' with your desired directory path
path_to_directory = '/home/go1/app/'
if not os.path.exists(path_to_directory):
    os.makedirs(path_to_directory)
file_path = os.path.join(path_to_directory, 'command.txt')


# Configuring MediaPipe Hands
with mp_hands.Hands(
    max_num_hands=1,  # Maximum number of hands to detect
    model_complexity=0,  # Model complexity (0 is the fastest)
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

  # Main loop to process video frames
  while cap.isOpened():
    success, image = cap.read() # Read a frame from the webcam
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Process the image for hand tracking
    results = hands.process(image)
    
    # Post-processing and visualization
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # If hands are detected in the frame
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
          # Drawing hand landmarks on the image
            mp_drawing.draw_landmarks(
                image,
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
        

        # Change the directory according to your sdk build file
        with open(file_path, 'w') as f: # write command to file
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
    
    # In case no hands are detected
    else:
        print('bae')
        with open(file_path, 'w') as f:
            # Write a neutral command when no hands are detected
            f.write("n")
    
    # image = cv2.resize((image, (1200, 1000)))
    f.close()

    # Displaying the processed image
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

    # Break the loop if 'ESC' key is pressed
    if cv2.waitKey(5) & 0xFF == 27:
        break

# Releasing the webcam and closing all OpenCV windows
cap.release()

