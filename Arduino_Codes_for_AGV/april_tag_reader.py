import cv2
from pupil_apriltags import Detector
import serial
import time

image = None
cap = cv2.VideoCapture(0)
detector = Detector(families="tag36h11")

usb = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

robot_state = "MOVING"
usb.write(b'f')
expected_tag = None


try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        # Draw detections
        for det in detections:
            pts = det.corners.astype(int)
            cv2.polylines(frame, [pts], True, (0,255,0), 2)
            cx, cy = map(int, det.center)
            cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)

            tag_id = det.tag_id

            # position text above tag
            x = pts[:,0].min()
            y = pts[:,1].min() - 10

            cv2.putText(frame, f"ID:{tag_id}", (x,y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255,0,0), 2)

            if (expected_tag is None or tag_id == expected_tag) and robot_state == "MOVING":
                print("Tag detected -> STOP")
                usb.write(b's')
                robot_state = "STOPPED"

        cv2.imshow('image', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    usb.close()
    cv2.destroyAllWindows()

