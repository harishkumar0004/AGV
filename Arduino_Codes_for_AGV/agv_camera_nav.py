#!/usr/bin/env python3
"""
AGV Camera-Based Navigation - STAGE 1
Clean, simple implementation for:
1. Tag 1 detected → Move forward
2. Tag 2 detected → Adjust position → 180° turn

No unnecessary debug output - only command flow
"""

import apriltag
import cv2
import serial
import time

# ===== CONFIGURATION =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2
BAUD_RATE = 9600
SERIAL_PORT = "/dev/ttyUSB0"

# Navigation parameters
CENTER_THRESHOLD = 40      # ±40px from center
TURN_SENSITIVITY = 100     # When to turn

# Tags
TAG_FORWARD = 1
TAG_ADJUST = 2

# ===== SERIAL COMMUNICATION =====
class SerialComm:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        
    def connect(self):
        """Connect to Arduino"""
        try:
            print(f"[SERIAL] Connecting to {self.port} @ {self.baudrate} baud...")
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
        """Send command and show what was sent"""
        if not self.connected:
            print("[SERIAL] Not connected!")
            return False
        
        try:
            print(f">>> COMMAND SENT: '{cmd}'")
            self.ser.write(cmd.encode())
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send: {e}")
            return False
    
    def close(self):
        if self.ser:
            self.ser.close()
            self.connected = False

# ===== CAMERA SETUP =====
class Camera:
    def __init__(self, index, width, height):
        self.index = index
        self.width = width
        self.height = height
        self.cap = None
        
    def initialize(self):
        try:
            print(f"[CAMERA] Initializing...")
            self.cap = cv2.VideoCapture(self.index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            ret, frame = self.cap.read()
            if not ret:
                print("[CAMERA] Failed to read")
                return False
            
            print(f"[CAMERA] ✓ Ready ({self.width}x{self.height})\n")
            return True
        except Exception as e:
            print(f"[CAMERA] Error: {e}")
            return False
    
    def read(self):
        if not self.cap:
            return None, None
        return self.cap.read()
    
    def close(self):
        if self.cap:
            self.cap.release()

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
        except Exception as e:
            print(f"[DETECTOR] Error: {e}")
            return []

# ===== NAVIGATION LOGIC =====
class Navigator:
    def __init__(self, serial_comm):
        self.serial = serial_comm
        self.last_command_time = 0
        self.command_interval = 0.3  # seconds
        self.current_state = "STOPPED"
        self.tag2_detected_time = 0
        self.turn_initiated = False
        
    def get_offset(self, detection):
        """Get horizontal offset from frame center"""
        cx = int(detection['center'][0])
        return cx - FRAME_CENTER
    
    def navigate(self, tag_id, offset):
        """Navigate based on tag detection"""
        current_time = time.time()
        
        # Rate limit commands
        if (current_time - self.last_command_time) < self.command_interval:
            return
        
        print(f"\n[NAV] Tag {tag_id} detected | Offset: {offset}px")
        
        if tag_id == TAG_FORWARD:
            # ===== TAG 1: MOVE FORWARD =====
            if abs(offset) < CENTER_THRESHOLD:
                # Tag centered → move forward
                print("[NAV] → FORWARD (tag centered)")
                self.serial.send('f')
                self.current_state = "FORWARD"
                
            elif offset < -TURN_SENSITIVITY:
                # Tag left → turn left to center
                print("[NAV] → FORWARD (correcting left)")
                self.serial.send('f')
                
            elif offset > TURN_SENSITIVITY:
                # Tag right → turn right to center
                print("[NAV] → FORWARD (correcting right)")
                self.serial.send('f')
            
            self.last_command_time = current_time
        
        elif tag_id == TAG_ADJUST:
            # ===== TAG 2: ADJUST POSITION → 180° TURN =====
            if abs(offset) < CENTER_THRESHOLD:
                # Tag centered → start adjustment phase
                if self.current_state != "ADJUSTING":
                    print("[NAV] → ADJUST (tag centered)")
                    self.serial.send('a')  # Send adjust command
                    self.current_state = "ADJUSTING"
                    self.tag2_detected_time = current_time
                    self.turn_initiated = False
                
                # After 2 seconds, initiate 180° turn
                elif (current_time - self.tag2_detected_time) >= 2.0:
                    if not self.turn_initiated:
                        print("[NAV] → TURN 180°")
                        self.serial.send('t')  # Send turn command
                        self.turn_initiated = True
                        self.current_state = "TURNING"
            
            self.last_command_time = current_time
    
    def no_tag(self):
        """Handle no tag detected"""
        current_time = time.time()
        if (current_time - self.last_command_time) > self.command_interval:
            if self.current_state != "STOPPED":
                print("[NAV] No tag → STOP")
                self.serial.send('s')
                self.current_state = "STOPPED"
            self.last_command_time = current_time

# ===== VISUALIZATION =====
def draw_overlay(frame, tag_id, offset):
    """Draw minimal overlay on frame"""
    # Center line
    cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT),
            (0, 255, 255), 2)
    
    # Title
    cv2.putText(frame, "AGV Stage 1: Forward -> Adjust -> 180°", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    if tag_id:
        status = f"Tag {tag_id} | Offset: {offset}px"
        color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
    else:
        status = "Waiting for tag..."
        color = (0, 165, 255)
    
    cv2.putText(frame, status, (10, 70),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    return frame

# ===== MAIN =====
def main():
    print("\n" + "="*60)
    print("AGV STAGE 1 - Clean Command-Based Navigation")
    print("="*60)
    print("Behavior:")
    print("  Tag 1: Move forward")
    print("  Tag 2: Adjust position → 180° turn")
    print("="*60 + "\n")
    
    # Initialize
    serial_comm = SerialComm(SERIAL_PORT, BAUD_RATE)
    if not serial_comm.connect():
        return
    
    camera = Camera(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    if not camera.initialize():
        serial_comm.close()
        return
    
    detector = Detector()
    if not detector.initialize():
        camera.close()
        serial_comm.close()
        return
    
    navigator = Navigator(serial_comm)
    
    print("[READY] System initialized and ready\n")
    print("="*60 + "\n")
    
    frame_count = 0
    
    try:
        while camera.cap.isOpened():
            ret, frame = camera.read()
            if not ret:
                break
            
            frame_count += 1
            
            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            
            tag_id = None
            offset = None
            
            if detections:
                det = detections[0]
                tag_id = det['id']
                
                if tag_id in [TAG_FORWARD, TAG_ADJUST]:
                    # Draw tag
                    pts = det['lb-rb-rt-lt'].astype(int)
                    color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
                    cv2.polylines(frame, [pts], True, color, 3)
                    
                    # Draw center
                    cx, cy = int(det['center'][0]), int(det['center'][1])
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    
                    # Draw offset line
                    cv2.line(frame, (FRAME_CENTER, cy), (cx, cy), (255, 0, 0), 2)
                    
                    # Tag label
                    cv2.putText(frame, f"ID:{tag_id}", (cx-20, cy-30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # Navigate
                    offset = navigator.get_offset(det)
                    navigator.navigate(tag_id, offset)
            else:
                navigator.no_tag()
            
            # Draw overlay
            frame = draw_overlay(frame, tag_id, offset)
            
            # Display
            cv2.imshow('AGV Stage 1', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n[USER] Quit requested")
    
    finally:
        print("\n[CLEANUP] Stopping motors...")
        serial_comm.send('s')
        time.sleep(0.5)
        
        camera.close()
        serial_comm.close()
        cv2.destroyAllWindows()
        
        print("[CLEANUP] Complete\n")

if __name__ == "__main__":
    main()