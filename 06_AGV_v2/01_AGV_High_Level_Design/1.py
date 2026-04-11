import apriltag
import cv2
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2)

def send(cmd):
    print("SEND:", cmd)
    ser.write((cmd + "\n").encode())

MAP = {
    1: (2, 1000),
    2: (1, 0),
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

    if tag == "QUIT":
        break

    if state == "IDLE":
        if tag is not None:
            current_tag = tag
            print("Detected Tag:", current_tag)
            state = "AT_TAG"

    elif state == "AT_TAG":

        if current_tag not in MAP:
            print("Unknown tag")
            state = "IDLE"
            continue

        target_tag, distance = MAP[current_tag]

        if target_tag is None:
            print("Final destination reached")
            send("CMD:STOP")
            break

        print(f"Moving from Tag {current_tag} → Tag {target_tag} | Distance: {distance}")

        send(f"CMD:FWD:{distance}")
        state = "WAIT_DONE"

    # -------- WAIT_DONE --------
    elif state == "WAIT_DONE":
        line = ser.readline().decode().strip()

        if line:
            print("Mega:", line)

        if line == "EVT:DONE":
            print("Motion Completed")
            state = "VERIFY_TAG"

    # -------- VERIFY_TAG --------
    elif state == "VERIFY_TAG":

        if tag == target_tag:
            print("Reached Tag:", target_tag)
            current_tag = tag
            state = "AT_TAG"
        else:
            # small forward correction if not yet detected
            send("CMD:FWD:50")
            time.sleep(0.2)

# -------- CLEANUP --------
cap.release()
cv2.destroyAllWindows()
ser.close()