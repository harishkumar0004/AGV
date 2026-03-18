#!/usr/bin/env python3
"""
AGV Navigation - ULTRA SIMPLE VERSION
Single window only - no complex code
Guaranteed to work
"""

import apriltag
import cv2
import serial
import time
import math
import numpy as np

# ===== CONFIG =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2
BAUD_RATE = 9600
SERIAL_PORT = "/dev/ttyUSB0"

CENTER_THRESHOLD = 40
TAG_FORWARD = 1
TAG_ALIGN = 2

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

def get_tag_angle(det):
    """Get tag angle"""
    try:
        corners = det['lb-rb-rt-lt']
        p1 = np.array(corners[2])
        p2 = np.array(corners[3])
        top_edge = p1 - p2
        angle_rad = math.atan2(top_edge[1], top_edge[0])
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        return int(angle_deg)
    except:
        return 0

# ===== MAIN LOOP =====
def main():
    """Main loop - single window, simple logic"""
    global running, camera, detector
    
    if not initialize():
        return
    
    # WINDOW CREATED ONCE HERE - NOT IN LOOP
    window_name = "AGV Navigation"
    
    # State tracking
    tag1_time = 0
    tag2_time = 0
    tag2_align_sent = False
    tag2_align_time = 0
    
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
            
            tag_id = None
            tag_angle = None
            offset = 0
            
            # Draw center line
            cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), (0, 255, 255), 2)
            
            # Draw title
            cv2.putText(frame, "Tag1: Forward | Tag2: Align+Pivot", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Process detections
            if detections:
                det = detections[0]
                tag_id = det['id']
                
                if tag_id in [TAG_FORWARD, TAG_ALIGN]:
                    # Draw tag
                    pts = det['lb-rb-rt-lt'].astype(int)
                    color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
                    cv2.polylines(frame, [pts], True, color, 3)
                    
                    # Draw center and offset
                    cx, cy = int(det['center'][0]), int(det['center'][1])
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.line(frame, (FRAME_CENTER, cy), (cx, cy), (255, 0, 0), 2)
                    
                    # Calculate offset
                    offset = cx - FRAME_CENTER
                    
                    # Get angle for Tag 2
                    if tag_id == TAG_ALIGN:
                        tag_angle = get_tag_angle(det)
                        cv2.putText(frame, f"Angle: {tag_angle}°", (cx-50, cy+30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Label
                    cv2.putText(frame, f"ID:{tag_id}", (cx-20, cy-30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # Navigation logic
                    current_time = time.time()
                    
                    if tag_id == TAG_FORWARD:
                        if current_time - tag1_time > 0.3:
                            send_command('f')
                            tag1_time = current_time
                        status = f"Tag 1 | Offset: {offset}px"
                    
                    elif tag_id == TAG_ALIGN:
                        if abs(offset) < CENTER_THRESHOLD:
                            if not tag2_align_sent:
                                if tag_angle is not None:
                                    angle_cmd = f"a{tag_angle:03d}"
                                    send_command(angle_cmd)
                                    tag2_align_sent = True
                                    tag2_align_time = current_time
                            
                            elif current_time - tag2_align_time >= 3.0:
                                send_command('p')
                                tag2_align_sent = False
                        
                        status = f"Tag 2 | Offset: {offset}px | Angle: {tag_angle}°"
                    
                    cv2.putText(frame, status, (10, 70),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "Waiting for tag...", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
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