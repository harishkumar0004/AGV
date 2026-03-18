#!/usr/bin/env python3
"""
AGV Camera Navigation - Stage 1 with Pivot Rotation (FIXED)
- Single window only
- No memory leaks
- Prevents multiple instances
"""

import apriltag
import cv2
import serial
import time
import math
import numpy as np
import os
import signal

# ===== CONFIGURATION =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2
BAUD_RATE = 9600
SERIAL_PORT = "/dev/ttyUSB0"

# Navigation
CENTER_THRESHOLD = 40
TURN_SENSITIVITY = 100

# Tags
TAG_FORWARD = 1
TAG_ALIGN = 2

# ===== PREVENT MULTIPLE INSTANCES =====
LOCKFILE = "/tmp/agv_nav.lock"

def check_single_instance():
    """Ensure only one instance of this script is running"""
    if os.path.exists(LOCKFILE):
        print("[ERROR] AGV Navigation already running!")
        print(f"[ERROR] Delete {LOCKFILE} if stuck, then retry")
        exit(1)
    
    # Create lock file
    with open(LOCKFILE, 'w') as f:
        f.write(str(os.getpid()))

def remove_lockfile():
    """Clean up lock file on exit"""
    if os.path.exists(LOCKFILE):
        try:
            os.remove(LOCKFILE)
        except:
            pass

# ===== SERIAL COMMUNICATION =====
class SerialComm:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        
    def connect(self):
        try:
            print(f"[SERIAL] Connecting to {self.port}...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.connected = True
            print(f"[SERIAL] ✓ Connected\n")
            return True
        except Exception as e:
            print(f"[SERIAL] ✗ Failed: {e}")
            return False
    
    def send(self, cmd):
        if not self.connected:
            return False
        try:
            print(f">>> COMMAND SENT: '{cmd}'")
            self.ser.write(cmd.encode())
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"[ERROR] {e}")
            return False
    
    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.connected = False

# ===== CAMERA =====
class Camera:
    def __init__(self, index, width, height):
        self.index = index
        self.width = width
        self.height = height
        self.cap = None
        
    def initialize(self):
        try:
            print("[CAMERA] Initializing...")
            self.cap = cv2.VideoCapture(self.index)
            if not self.cap.isOpened():
                print("[CAMERA] Failed to open camera")
                return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer
            
            ret, frame = self.cap.read()
            if not ret:
                print("[CAMERA] Failed to read")
                return False
            
            print(f"[CAMERA] ✓ Ready\n")
            return True
        except Exception as e:
            print(f"[CAMERA] Error: {e}")
            return False
    
    def read(self):
        if not self.cap:
            return None, None
        try:
            ret, frame = self.cap.read()
            return ret, frame
        except:
            return None, None
    
    def close(self):
        if self.cap:
            try:
                self.cap.release()
            except:
                pass

# ===== APRILTAG DETECTOR =====
class Detector:
    def __init__(self):
        self.detector = None
        
    def initialize(self):
        try:
            print("[DETECTOR] Initializing AprilTag...")
            self.detector = apriltag.apriltag(
                family='tag36h11',
                threads=4,
                maxhamming=1,
                decimate=2.0,
                blur=0.0,
                refine_edges=True,
                debug=False
            )
            print("[DETECTOR] ✓ Ready\n")
            return True
        except Exception as e:
            print(f"[DETECTOR] Error: {e}")
            return False
    
    def detect(self, gray_frame):
        if not self.detector:
            return []
        try:
            return self.detector.detect(gray_frame)
        except:
            return []

# ===== TAG ANGLE EXTRACTION =====
def get_tag_angle(detection):
    """Extract tag rotation angle from AprilTag detection"""
    try:
        corners = detection['lb-rb-rt-lt']
        p1 = np.array(corners[2])  # rt
        p2 = np.array(corners[3])  # lt
        
        top_edge = p1 - p2
        angle_rad = math.atan2(top_edge[1], top_edge[0])
        angle_deg = math.degrees(angle_rad)
        
        if angle_deg < 0:
            angle_deg += 360
        
        return int(angle_deg)
    except:
        return 0

# ===== NAVIGATION =====
class Navigator:
    def __init__(self, serial_comm):
        self.serial = serial_comm
        self.last_command_time = 0
        self.command_interval = 0.3
        self.state = "STOPPED"
        self.tag2_align_sent = False
        self.tag2_align_start_time = 0
        self.pivot_sent = False
        
    def get_offset(self, detection):
        cx = int(detection['center'][0])
        return cx - FRAME_CENTER
    
    def navigate(self, tag_id, offset, tag_angle=None):
        current_time = time.time()
        
        if (current_time - self.last_command_time) < self.command_interval:
            return
        
        print(f"\n[NAV] Tag {tag_id} detected | Offset: {offset}px", end="")
        if tag_angle is not None:
            print(f" | Angle: {tag_angle}°", end="")
        print()
        
        if tag_id == TAG_FORWARD:
            if abs(offset) < CENTER_THRESHOLD:
                print("[NAV] → Tag 1 FORWARD (centered)")
                self.serial.send('f')
                self.state = "FORWARD"
            else:
                print("[NAV] → Tag 1 FORWARD (correcting)")
                self.serial.send('f')
            
            self.last_command_time = current_time
        
        elif tag_id == TAG_ALIGN:
            if abs(offset) < CENTER_THRESHOLD:
                if not self.tag2_align_sent:
                    if tag_angle is not None:
                        angle_str = f"a{tag_angle:03d}"
                        print(f"[NAV] → Tag 2 ALIGN to {tag_angle}°")
                        self.serial.send(angle_str)
                        self.tag2_align_sent = True
                        self.tag2_align_start_time = current_time
                        self.state = "ALIGNING"
                        self.pivot_sent = False
                
                elif (current_time - self.tag2_align_start_time) >= 3.0:
                    if not self.pivot_sent:
                        print("[NAV] → Tag 2 PIVOT 180°")
                        self.serial.send('p')
                        self.pivot_sent = True
                        self.state = "PIVOTING"
            
            self.last_command_time = current_time
    
    def no_tag(self):
        current_time = time.time()
        if (current_time - self.last_command_time) > self.command_interval:
            if self.state != "STOPPED":
                print("[NAV] No tag → STOP")
                self.serial.send('s')
                self.state = "STOPPED"
                self.tag2_align_sent = False
                self.pivot_sent = False
            self.last_command_time = current_time

# ===== VISUALIZATION =====
def draw_overlay(frame, tag_id, offset, angle):
    """Draw overlay on frame"""
    try:
        # Center line
        cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT),
                (0, 255, 255), 2)
        
        # Title
        cv2.putText(frame, "Tag 1: Forward | Tag 2: Align → Pivot 180°", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if tag_id:
            status = f"Tag {tag_id} | Offset: {offset}px"
            if angle is not None:
                status += f" | Angle: {angle}°"
            color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
        else:
            status = "Waiting for tag..."
            color = (0, 165, 255)
        
        cv2.putText(frame, status, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return frame
    except:
        return frame

# ===== MAIN =====
def main():
    print("\n" + "="*70)
    print("AGV STAGE 1 - Pivot 180° Rotation (SINGLE WINDOW)")
    print("="*70)
    print("Behavior:")
    print("  Tag 1: Move forward")
    print("  Tag 2: Detect tag angle → Align → Pivot 180° in place → Stop")
    print("="*70 + "\n")
    
    # Check single instance
    check_single_instance()
    
    # Initialize
    serial_comm = SerialComm(SERIAL_PORT, BAUD_RATE)
    if not serial_comm.connect():
        remove_lockfile()
        return
    
    camera = Camera(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    if not camera.initialize():
        serial_comm.close()
        remove_lockfile()
        return
    
    detector = Detector()
    if not detector.initialize():
        camera.close()
        serial_comm.close()
        remove_lockfile()
        return
    
    navigator = Navigator(serial_comm)
    
    print("[READY] System initialized\n")
    print("="*70 + "\n")
    
    frame_count = 0
    window_name = "AGV Navigation - Pivot 180°"  # SINGLE WINDOW NAME
    
    try:
        while camera.cap and camera.cap.isOpened():
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            
            tag_id = None
            offset = None
            tag_angle = None
            
            if detections:
                det = detections[0]
                tag_id = det['id']
                
                if tag_id in [TAG_FORWARD, TAG_ALIGN]:
                    # Draw tag
                    pts = det['lb-rb-rt-lt'].astype(int)
                    color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
                    cv2.polylines(frame, [pts], True, color, 3)
                    
                    # Draw center
                    cx, cy = int(det['center'][0]), int(det['center'][1])
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    
                    # Draw offset line
                    cv2.line(frame, (FRAME_CENTER, cy), (cx, cy), (255, 0, 0), 2)
                    
                    # Get offset and angle
                    offset = navigator.get_offset(det)
                    
                    if tag_id == TAG_ALIGN:
                        tag_angle = get_tag_angle(det)
                        angle_text = f"Angle: {tag_angle}°"
                        cv2.putText(frame, angle_text, (cx-50, cy+30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Tag label
                    cv2.putText(frame, f"ID:{tag_id}", (cx-20, cy-30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # Navigate
                    navigator.navigate(tag_id, offset, tag_angle)
            else:
                navigator.no_tag()
            
            # Draw overlay
            frame = draw_overlay(frame, tag_id, offset, tag_angle)
            
            # Display in SINGLE window (reused, not creating new ones)
            cv2.imshow(window_name, frame)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[USER] Quit requested")
                break
    
    except KeyboardInterrupt:
        print("\n[USER] Interrupted")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
    
    finally:
        print("\n[CLEANUP] Stopping motors...")
        serial_comm.send('s')
        time.sleep(0.5)
        
        print("[CLEANUP] Closing resources...")
        
        # Close camera properly
        camera.close()
        
        # Close serial properly
        serial_comm.close()
        
        # Close ALL OpenCV windows
        cv2.destroyAllWindows()
        
        # Clean up lock file
        remove_lockfile()
        
        print("[CLEANUP] Complete\n")

# ===== SIGNAL HANDLERS =====
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n[SIGNAL] Caught interrupt")
    raise KeyboardInterrupt

# ===== ENTRY POINT =====
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        main()
    except:
        remove_lockfile()
        cv2.destroyAllWindows()