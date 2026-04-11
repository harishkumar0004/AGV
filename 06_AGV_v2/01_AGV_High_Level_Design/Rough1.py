import apriltag
import cv2
import numpy as np
import serial, time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0.1)
time.sleep(1)

def send(cmd):
    print("SEND:", cmd)
    ser.write((cmd + "\n").encode())

current_tag = None
target_tag = None
state = "IDLE"

MAP = {
    1: (2, 500),
    2: (3, 1000),
    3: (4, 500),
    4: (None, 0)
}

cap = cv2.VideoCapture(0)

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

def detect_tag():
    
    ret, frame = cap.read()
    if not ret:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    tag_id = None

    for det in detections:
        pts = det.corners.astype(int)
        cv2.polylines(frame, [pts], True, (0,255,0), 2)

        cx, cy = map(int, det.center)
        cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)

        tag_id = det.tag_id

        cv2.putText(frame, f"ID:{tag_id}", (pts[0][0], pts[0][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

    cv2.imshow("AGV View", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        return "QUIT"

    return tag_id

state = "IDLE"
current_tag = None
target_tag = None

while True:
    tag = detect_tag()
    if state == "IDLE":
        if tag is not None:
            current_tag = tag
            state = "AT_TAG"
    
    elif state == "AT_TAG":
        if current_tag not in 

    