#!/usr/bin/env python3
"""AGV Navigation using AprilTag + Arduino (Final Stable Version)"""

import cv2
import serial
import time
import apriltag

# ===== CONFIG =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

TAG_START = 1
TAG_TURN = 2

PIVOT_TIMEOUT = 20

# ===== GLOBAL =====
serial_conn = None
camera = None
detector = None
running = True
serial_buffer = ""
last_event = None


# ===== SERIAL =====
def send_command(cmd):
    try:
        serial_conn.write((cmd + "\n").encode())
        print(f">>> {cmd}")
    except:
        pass


def read_serial():
    global serial_buffer, last_event

    try:
        if serial_conn.in_waiting > 0:
            serial_buffer += serial_conn.read(serial_conn.in_waiting).decode(errors="ignore")

            while "\n" in serial_buffer:
                line, serial_buffer = serial_buffer.split("\n", 1)
                line = line.strip()

                if line:
                    print(f"<<< {line}")

                    if line.startswith("EVENT:"):
                        last_event = line
    except:
        pass


# ===== INIT =====
def initialize():
    global serial_conn, camera, detector

    print("\n[INIT] Starting...\n")

    # SERIAL
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        send_command('s')
        print("[SERIAL] OK")
    except Exception as e:
        print("Serial Error:", e)
        return False

    # CAMERA
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        ret, _ = camera.read()
        if not ret:
            raise Exception("Camera failed")

        print("[CAMERA] OK")
    except Exception as e:
        print("Camera Error:", e)
        return False

    # APRILTAG (works for your installed version)
    try:
        detector = apriltag.apriltag("tag36h11")
        print("[APRILTAG] OK")
    except Exception as e:
        print("AprilTag Error:", e)
        return False

    print("\n[READY]\n")
    return True


# ===== CLEANUP =====
def cleanup():
    print("\n[CLEANUP]")

    try:
        send_command('s')
        time.sleep(0.2)
        serial_conn.close()
    except:
        pass

    try:
        camera.release()
    except:
        pass

    cv2.destroyAllWindows()


# ===== DETECTION COMPATIBILITY =====
def get_tag_info(det):
    """Handle both dict-style and object-style apriltag"""
    if isinstance(det, dict):
        tag_id = det['id']
        corners = det['lb-rb-rt-lt']
    else:
        tag_id = det.tag_id
        corners = det.corners

    return tag_id, corners.astype(int)


# ===== MAIN =====
def main():
    global running, last_event

    if not initialize():
        return

    state = "WAIT_TAG1"
    pivot_time = 0

    while running:

        ret, frame = camera.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        read_serial()

        visible_tags = set()

        # Center line
        cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), (0,255,255), 2)

        # ===== DETECT TAGS =====
        for det in detections:
            tag_id, pts = get_tag_info(det)

            if tag_id not in [TAG_START, TAG_TURN]:
                continue

            visible_tags.add(tag_id)

            color = (0,255,0) if tag_id == TAG_START else (0,165,255)
            cv2.polylines(frame, [pts], True, color, 2)

        # ===== STATE MACHINE =====

        if state == "WAIT_TAG1":
            if TAG_START in visible_tags:
                send_command('f')
                state = "GO_TAG2"
                status = "Tag1 → Forward"
            else:
                status = "Waiting Tag1"

        elif state == "GO_TAG2":
            if TAG_TURN in visible_tags:
                send_command('p')
                pivot_time = time.time()
                state = "WAIT_PIVOT"
                status = "Tag2 → Pivot"
            else:
                status = "Moving forward..."

        elif state == "WAIT_PIVOT":
            if last_event == "EVENT:PIVOT_DONE":
                send_command('f')
                state = "RETURN_TAG1"
                status = "Pivot done → Return"
            elif time.time() - pivot_time > PIVOT_TIMEOUT:
                send_command('s')
                state = "STOP"
                status = "Pivot timeout"
            else:
                status = "Waiting pivot..."

        elif state == "RETURN_TAG1":
            if TAG_START in visible_tags:
                send_command('s')
                state = "STOP"
                status = "Reached Tag1 → STOP"
            else:
                status = "Returning..."

        else:
            status = "DONE"

        # ===== DISPLAY =====
        cv2.putText(frame, status, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.imshow("AGV", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cleanup()


if __name__ == "__main__":
    main()