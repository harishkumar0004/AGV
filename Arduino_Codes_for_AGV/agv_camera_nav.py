#!/usr/bin/env python3
"""Stage 1 AGV navigation using AprilTags on the Raspberry Pi."""

import apriltag
import cv2
import serial
import time

# ===== CONFIG =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2
BAUD_RATE = 9600
SERIAL_PORT = "/dev/ttyUSB0"

TAG_START = 1
TAG_TURN = 2
FORWARD_REFRESH_INTERVAL = 1.0
PIVOT_DURATION_SECONDS = 4.0

# ===== GLOBAL STATE =====
running = True
camera = None
detector = None
serial_conn = None

# ===== FUNCTIONS =====
def send_command(cmd):
    """Send command to Arduino"""
    if serial_conn and serial_conn.is_open:
        try:
            serial_conn.write(cmd.encode())
            print(f">>> CMD: '{cmd}'")
        except:
            pass

def initialize():
    """Initialize all systems"""
    global camera, detector, serial_conn
    
    print("\n[INIT] Starting AGV Navigation...\n")
    
    # Serial
    try:
        print("[SERIAL] Connecting...")
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        send_command('s')
        print("[SERIAL] ✓ Ready\n")
    except Exception as e:
        print(f"[SERIAL] ✗ Failed: {e}\n")
        return False
    
    # Camera
    try:
        print("[CAMERA] Opening...")
        camera = cv2.VideoCapture(CAMERA_INDEX)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        ret, _ = camera.read()
        if not ret:
            raise Exception("Can't read from camera")
        
        print("[CAMERA] ✓ Ready\n")
    except Exception as e:
        print(f"[CAMERA] ✗ Failed: {e}\n")
        return False
    
    # Detector
    try:
        print("[DETECTOR] Initializing...")
        detector = apriltag.apriltag(
            family='tag36h11',
            threads=4,
            maxhamming=1,
            decimate=2.0,
            blur=0.0,
            refine_edges=True,
            debug=False
        )
        print("[DETECTOR] ✓ Ready\n")
    except Exception as e:
        print(f"[DETECTOR] ✗ Failed: {e}\n")
        return False
    
    print("[READY] All systems initialized\n")
    return True

def cleanup():
    """Clean up and exit"""
    global running, camera, serial_conn
    
    print("\n[CLEANUP] Stopping...\n")
    
    running = False
    
    # Stop motors
    if serial_conn and serial_conn.is_open:
        try:
            send_command('s')
            time.sleep(0.2)
            serial_conn.close()
        except:
            pass
    
    # Close camera
    if camera:
        try:
            camera.release()
        except:
            pass
    
    # Close all windows
    cv2.destroyAllWindows()
    
    print("[CLEANUP] ✓ Done\n")

# ===== MAIN LOOP =====
def main():
    """Main loop - single window, simple logic"""
    global running, camera, detector
    
    if not initialize():
        return
    
    # WINDOW CREATED ONCE HERE - NOT IN LOOP
    window_name = "AGV Navigation"
    
    # Stage 1 state tracking
    stage_state = "WAITING_FOR_TAG_1"
    last_forward_command_time = 0.0
    pivot_start_time = 0.0
    
    print("Press 'q' to quit\n")
    
    try:
        while running:
            # Read frame
            ret, frame = camera.read()
            if not ret:
                time.sleep(0.01)
                continue
            
            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            
            current_time = time.time()
            visible_tag_ids = set()
            status = ""
            
            # Draw center line
            cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), (0, 255, 255), 2)
            
            # Draw title
            cv2.putText(frame, "Stage 1 | Tag1: Start Forward | Tag2: Turn 180 + Stop", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw detections
            if detections:
                for det in detections:
                    tag_id = det['id']
                    if tag_id not in [TAG_START, TAG_TURN]:
                        continue

                    visible_tag_ids.add(tag_id)
                    pts = det['lb-rb-rt-lt'].astype(int)
                    color = (0, 255, 0) if tag_id == TAG_START else (0, 165, 255)
                    cv2.polylines(frame, [pts], True, color, 3)

                    cx, cy = int(det['center'][0]), int(det['center'][1])
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.line(frame, (FRAME_CENTER, cy), (cx, cy), (255, 0, 0), 2)
                    offset = cx - FRAME_CENTER

                    cv2.putText(frame, f"ID:{tag_id} | Offset:{offset}px", (cx - 70, cy - 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ===== STAGE 1 STATE MACHINE =====
            if stage_state == "WAITING_FOR_TAG_1":
                if TAG_START in visible_tag_ids:
                    send_command('f')
                    last_forward_command_time = current_time
                    stage_state = "MOVING_TO_TAG_2"
                    status = "Tag 1 detected -> Forward started"
                else:
                    status = "Waiting for Tag 1 to start moving"

            elif stage_state == "MOVING_TO_TAG_2":
                if TAG_TURN in visible_tag_ids:
                    send_command('p')
                    pivot_start_time = current_time
                    stage_state = "TURNING_180"
                    status = "Tag 2 detected -> Turning 180"
                elif current_time - last_forward_command_time >= FORWARD_REFRESH_INTERVAL:
                    send_command('f')
                    last_forward_command_time = current_time
                    status = "Moving forward until Tag 2 is detected"
                else:
                    status = "Moving forward until Tag 2 is detected"

            elif stage_state == "TURNING_180":
                elapsed = current_time - pivot_start_time
                remaining = max(0.0, PIVOT_DURATION_SECONDS - elapsed)
                status = f"Turning 180... {remaining:.1f}s remaining"

                if elapsed >= PIVOT_DURATION_SECONDS:
                    send_command('s')
                    stage_state = "STAGE_1_COMPLETE"
                    status = "Stage 1 complete -> AGV stopped"

            else:
                status = "Stage 1 complete. Press q to exit."

            cv2.putText(frame, status, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # ===== DISPLAY IN SINGLE WINDOW =====
            # This window is created ONCE above, then updated every frame
            cv2.imshow(window_name, frame)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n[USER] Interrupted")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
    
    finally:
        cleanup()

if __name__ == "__main__":
    main()
