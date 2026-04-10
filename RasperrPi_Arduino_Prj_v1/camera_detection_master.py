#!/usr/bin/env python3
"""AGV navigation using AprilTags with proper Arduino handshaking"""

import apriltag
import cv2
import serial
import time

# ===== CONFIG =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2

BAUD_RATE = 115200   # MUST match Arduino
SERIAL_PORT = "/dev/ttyUSB0"

TAG_START = 1
TAG_TURN = 2

PIVOT_ACK_TIMEOUT_SECONDS = 20.0

# ===== GLOBAL STATE =====
running = True
camera = None
detector = None
serial_conn = None
serial_rx_buffer = ""
last_event = None

# ===== SERIAL =====
def send_command(cmd):
    """Send command to Arduino with newline"""
    if serial_conn and serial_conn.is_open:
        try:
            serial_conn.write((cmd + '\n').encode())
            print(f">>> CMD: {cmd}")
        except:
            pass


def read_serial_messages():
    """Read messages and capture events"""
    global serial_rx_buffer, last_event

    messages = []

    if not serial_conn or not serial_conn.is_open:
        return messages

    try:
        if serial_conn.in_waiting > 0:
            serial_rx_buffer += serial_conn.read(serial_conn.in_waiting).decode(errors="ignore")

            while "\n" in serial_rx_buffer:
                line, serial_rx_buffer = serial_rx_buffer.split("\n", 1)
                line = line.strip()

                if line:
                    print(f"<<< {line}")
                    messages.append(line)

                    if line.startswith("EVENT:"):
                        last_event = line

    except:
        pass

    return messages


# ===== INIT =====
def initialize():
    global camera, detector, serial_conn, serial_rx_buffer

    print("\n[INIT] Starting...\n")

    # SERIAL
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        serial_conn.reset_input_buffer()
        send_command('s')
        print("[SERIAL] Ready")
    except Exception as e:
        print(f"[SERIAL ERROR] {e}")
        return False

    # CAMERA
    try:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        ret, _ = camera.read()
        if not ret:
            raise Exception("Camera failed")

        print("[CAMERA] Ready")
    except Exception as e:
        print(f"[CAMERA ERROR] {e}")
        return False

    # APRILTAG
    detector = apriltag.apriltag()

    print("[READY]\n")
    return True


# ===== CLEANUP =====
def cleanup():
    global running

    print("\n[CLEANUP]")

    running = False

    if serial_conn and serial_conn.is_open:
        send_command('s')
        time.sleep(0.2)
        serial_conn.close()

    if camera:
        camera.release()

    cv2.destroyAllWindows()


# ===== MAIN =====
def main():
    global running, last_event

    if not initialize():
        return

    stage_state = "WAITING_FOR_TAG_1"
    pivot_command_time = 0

    window_name = "AGV"

    try:
        while running:

            ret, frame = camera.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)

            read_serial_messages()

            visible_tag_ids = set()

            # Draw center line
            cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), (0,255,255), 2)

            # ===== DETECTION =====
            for det in detections:
                tag_id = det.tag_id

                if tag_id not in [TAG_START, TAG_TURN]:
                    continue

                visible_tag_ids.add(tag_id)

                pts = det.corners.astype(int)
                color = (0,255,0) if tag_id == TAG_START else (0,165,255)
                cv2.polylines(frame, [pts], True, color, 2)

            # ===== STATE MACHINE =====

            if stage_state == "WAITING_FOR_TAG_1":

                if TAG_START in visible_tag_ids:
                    send_command('f')
                    stage_state = "MOVING_TO_TAG_2"
                    status = "Tag1 → Forward"

                else:
                    status = "Waiting Tag1"


            elif stage_state == "MOVING_TO_TAG_2":

                if TAG_TURN in visible_tag_ids:
                    send_command('p')
                    pivot_command_time = time.time()
                    stage_state = "WAITING_FOR_PIVOT_DONE"
                    status = "Tag2 → Pivot"

                else:
                    status = "Moving forward..."


            elif stage_state == "WAITING_FOR_PIVOT_DONE":

                if last_event == "EVENT:PIVOT_DONE":
                    send_command('f')
                    stage_state = "RETURNING_TO_TAG_1"
                    status = "Pivot done → Return"

                elif time.time() - pivot_command_time > PIVOT_ACK_TIMEOUT_SECONDS:
                    send_command('s')
                    stage_state = "FINAL_STOP"
                    status = "Timeout Stop"

                else:
                    status = "Waiting pivot finish"


            elif stage_state == "RETURNING_TO_TAG_1":

                if TAG_START in visible_tag_ids:
                    send_command('s')
                    stage_state = "FINAL_STOP"
                    status = "Reached Tag1 → STOP"

                else:
                    status = "Returning..."


            else:
                status = "DONE"


            # Display status
            cv2.putText(frame, status, (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imshow(window_name, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cleanup()


if __name__ == "__main__":
    main()