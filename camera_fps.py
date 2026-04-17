import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Error opening video stream")

# Read the first frame


# Calculate the FPS
desired_fps = 15
cap.set(cv2.CAP_PROP_FPS, desired_fps)
ret, frame = cap.read()

# Release the VideoCapture object
cap.release()