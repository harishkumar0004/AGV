#!/usr/bin/env python3
"""
AGV Camera-Based Navigation with AprilTags
COMPLETE DEBUG VERSION - All processes logged
"""

import apriltag
import cv2
import serial
import time
import sys

# ===== CONFIGURATION =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2
BAUD_RATE = 9600
SERIAL_PORT = "/dev/ttyUSB0"

# Navigation tuning (in pixels)
CENTER_THRESHOLD = 40
TURN_SENSITIVITY = 100

# Tags
TAG_FORWARD = 1
TAG_TURN = 2

# Debug flags
DEBUG = True
DEBUG_SERIAL = True
DEBUG_CAMERA = True
DEBUG_NAVIGATION = True

def debug_print(module, message, level="INFO"):
    """Print debug message with timestamp and module"""
    timestamp = time.strftime("%H:%M:%S")
    color_map = {
        "INFO": "\033[92m",      # Green
        "WARNING": "\033[93m",   # Yellow
        "ERROR": "\033[91m",     # Red
        "DEBUG": "\033[94m"      # Blue
    }
    reset = "\033[0m"
    color = color_map.get(level, reset)
    
    if DEBUG:
        print(f"{color}[{timestamp}] [{module}] {message}{reset}")

# ===== SERIAL COMMUNICATION =====
class SerialComm:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        self.commands_sent = 0
        self.last_command = None
        
    def connect(self):
        """Establish serial connection"""
        try:
            debug_print("SERIAL", f"Attempting to connect on {self.port} @ {self.baudrate} baud...", "DEBUG")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            
            # Clear any garbage data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.connected = True
            debug_print("SERIAL", f"✓ Connected successfully", "INFO")
            debug_print("SERIAL", f"Port: {self.port} | Baud: {self.baudrate} | Timeout: 1s", "DEBUG")
            return True
            
        except serial.SerialException as e:
            debug_print("SERIAL", f"✗ Connection failed: {e}", "ERROR")
            debug_print("SERIAL", "Try: ls /dev/tty* to find available ports", "WARNING")
            return False
    
    def send(self, cmd):
        """Send command to Arduino"""
        if not self.connected:
            debug_print("SERIAL", "Not connected, cannot send command", "ERROR")
            return False
        
        try:
            # Send command
            cmd_bytes = cmd.encode()
            debug_print("SERIAL", f"Sending: '{cmd}' (bytes: {cmd_bytes})", "DEBUG")
            
            self.ser.write(cmd_bytes)
            self.commands_sent += 1
            self.last_command = cmd
            time.sleep(0.05)
            
            # Try to read response
            response = ""
            if self.ser.in_waiting:
                try:
                    response = self.ser.readline().decode().strip()
                    debug_print("SERIAL", f"Arduino response: {response}", "DEBUG")
                except Exception as e:
                    debug_print("SERIAL", f"Error reading response: {e}", "WARNING")
            else:
                debug_print("SERIAL", "No response from Arduino", "WARNING")
            
            return True
            
        except Exception as e:
            debug_print("SERIAL", f"Error sending command: {e}", "ERROR")
            return False
    
    def status(self):
        """Print connection status"""
        debug_print("SERIAL", f"Status: {'Connected' if self.connected else 'Disconnected'}", "INFO")
        debug_print("SERIAL", f"Commands sent: {self.commands_sent}", "DEBUG")
        debug_print("SERIAL", f"Last command: {self.last_command}", "DEBUG")
    
    def close(self):
        """Close serial connection"""
        if self.ser:
            self.ser.close()
            self.connected = False
            debug_print("SERIAL", "Connection closed", "INFO")

# ===== CAMERA =====
class Camera:
    def __init__(self, index, width, height):
        self.index = index
        self.width = width
        self.height = height
        self.cap = None
        self.frame_count = 0
        
    def initialize(self):
        """Initialize camera"""
        try:
            debug_print("CAMERA", f"Initializing camera {self.index}...", "DEBUG")
            self.cap = cv2.VideoCapture(self.index)
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Try to read one frame
            ret, frame = self.cap.read()
            if not ret:
                debug_print("CAMERA", "Failed to read frame", "ERROR")
                return False
            
            debug_print("CAMERA", "✓ Camera initialized successfully", "INFO")
            debug_print("CAMERA", f"Resolution: {self.width}x{self.height} @ 30 FPS", "DEBUG")
            return True
            
        except Exception as e:
            debug_print("CAMERA", f"Error initializing camera: {e}", "ERROR")
            return False
    
    def read(self):
        """Read frame from camera"""
        if not self.cap:
            debug_print("CAMERA", "Camera not initialized", "ERROR")
            return None, None
        
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
        return ret, frame
    
    def close(self):
        """Close camera"""
        if self.cap:
            self.cap.release()
            debug_print("CAMERA", "Camera closed", "INFO")

# ===== APRILTAG DETECTOR =====
class AprilTagDetector:
    def __init__(self):
        self.detector = None
        self.detections_count = 0
        
    def initialize(self):
        """Initialize detector"""
        try:
            debug_print("DETECTOR", "Initializing AprilTag detector...", "DEBUG")
            self.detector = apriltag.apriltag(
                family='tag36h11',
                threads=4,
                maxhamming=1,
                decimate=2.0,
                blur=0.0,
                refine_edges=True,
                debug=False
            )
            debug_print("DETECTOR", "✓ AprilTag detector initialized", "INFO")
            debug_print("DETECTOR", "Family: tag36h11 | Threads: 4 | Decimate: 2.0", "DEBUG")
            return True
        except Exception as e:
            debug_print("DETECTOR", f"Error: {e}", "ERROR")
            return False
    
    def detect(self, gray_frame):
        """Detect tags in frame"""
        if not self.detector:
            debug_print("DETECTOR", "Detector not initialized", "ERROR")
            return []
        
        try:
            detections = self.detector.detect(gray_frame)
            if detections:
                self.detections_count += 1
                if DEBUG_CAMERA:
                    debug_print("DETECTOR", f"Found {len(detections)} tag(s)", "INFO")
                    for det in detections:
                        debug_print("DETECTOR", f"  Tag ID: {det['id']} | Center: ({int(det['center'][0])}, {int(det['center'][1])})", "DEBUG")
            return detections
        except Exception as e:
            debug_print("DETECTOR", f"Detection error: {e}", "ERROR")
            return []

# ===== NAVIGATION =====
class Navigator:
    def __init__(self, serial_comm):
        self.serial = serial_comm
        self.last_navigation_time = 0
        self.navigation_interval = 0.3  # seconds
        self.navigation_count = 0
        
    def get_tag_offset(self, detection):
        """Calculate horizontal offset from frame center"""
        center_x = int(detection['center'][0])
        offset = center_x - FRAME_CENTER
        return offset
    
    def navigate(self, tag_id, offset):
        """Decide movement based on tag"""
        current_time = time.time()
        
        # Rate limit navigation commands
        if (current_time - self.last_navigation_time) < self.navigation_interval:
            return None
        
        self.last_navigation_time = current_time
        self.navigation_count += 1
        
        if DEBUG_NAVIGATION:
            debug_print("NAV", f"[{self.navigation_count}] Tag {tag_id} detected | Offset: {offset}px", "INFO")
        
        decision = None
        command = None
        
        if tag_id == TAG_FORWARD:
            # Move forward with centering
            if abs(offset) < CENTER_THRESHOLD:
                decision = "FORWARD (centered)"
                command = 'f'
            elif offset < -TURN_SENSITIVITY:
                decision = f"TURN LEFT (offset: {offset})"
                command = 'l'
            elif offset > TURN_SENSITIVITY:
                decision = f"TURN RIGHT (offset: {offset})"
                command = 'r'
            else:
                decision = f"FORWARD (correcting)"
                command = 'f'
        
        elif tag_id == TAG_TURN:
            # Turn decision based on position
            if offset < -50:
                decision = f"TURN LEFT (tag at {offset})"
                command = 'l'
            elif offset > 50:
                decision = f"TURN RIGHT (tag at {offset})"
                command = 'r'
            else:
                decision = "STOP (tag centered)"
                command = 's'
        
        if command:
            if DEBUG_NAVIGATION:
                debug_print("NAV", f"Decision: {decision}", "INFO")
                debug_print("NAV", f"Sending command: '{command}'", "DEBUG")
            
            self.serial.send(command)
        
        return decision
    
    def no_tag_detected(self):
        """Handle no tag detected"""
        current_time = time.time()
        if (current_time - self.last_navigation_time) > self.navigation_interval:
            debug_print("NAV", "No tags detected - STOP", "WARNING")
            self.serial.send('s')
            self.last_navigation_time = current_time

# ===== VISUALIZATION =====
def draw_debug_info(frame, tag_id, offset, decision):
    """Draw debug overlay on frame"""
    # Frame center line
    cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), 
            (0, 255, 255), 2)
    
    # Header
    cv2.putText(frame, "AGV Navigation [DEBUG]", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Status
    if decision:
        cv2.putText(frame, f"Decision: {decision}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Waiting for tag...", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
    
    if tag_id:
        cv2.putText(frame, f"Tag {tag_id} | Offset: {offset}px", (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    return frame

# ===== MAIN =====
def main():
    """Main navigation loop"""
    print("\n" + "="*70)
    print(" "*15 + "AGV CAMERA-BASED NAVIGATION [DEBUG VERSION]")
    print("="*70 + "\n")
    
    # Initialize serial
    debug_print("MAIN", "Starting AGV Navigation System...", "INFO")
    serial_comm = SerialComm(SERIAL_PORT, BAUD_RATE)
    
    if not serial_comm.connect():
        debug_print("MAIN", "Fatal: Cannot connect to Arduino", "ERROR")
        return
    
    # Initialize camera
    camera = Camera(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)
    if not camera.initialize():
        debug_print("MAIN", "Fatal: Cannot initialize camera", "ERROR")
        serial_comm.close()
        return
    
    # Initialize detector
    detector = AprilTagDetector()
    if not detector.initialize():
        debug_print("MAIN", "Fatal: Cannot initialize detector", "ERROR")
        camera.close()
        serial_comm.close()
        return
    
    # Initialize navigator
    navigator = Navigator(serial_comm)
    
    debug_print("MAIN", "All systems ready!", "INFO")
    print("\n" + "-"*70)
    print("Configuration:")
    print(f"  Tag {TAG_FORWARD}: Move forward")
    print(f"  Tag {TAG_TURN}: Turn decision")
    print(f"  Center threshold: {CENTER_THRESHOLD}px")
    print(f"  Turn sensitivity: {TURN_SENSITIVITY}px")
    print("-"*70 + "\n")
    
    try:
        frame_count = 0
        
        while camera.cap.isOpened():
            # Read frame
            ret, frame = camera.read()
            if not ret:
                debug_print("MAIN", "Cannot read frame", "ERROR")
                break
            
            frame_count += 1
            
            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            
            # Process
            tag_id = None
            offset = None
            decision = None
            
            if detections:
                det = detections[0]
                tag_id = det['id']
                
                if tag_id in [TAG_FORWARD, TAG_TURN]:
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
                    offset = navigator.get_tag_offset(det)
                    decision = navigator.navigate(tag_id, offset)
                else:
                    debug_print("MAIN", f"Tag ID {tag_id} not in navigation list", "WARNING")
            else:
                navigator.no_tag_detected()
            
            # Draw debug overlay
            frame = draw_debug_info(frame, tag_id, offset, decision)
            
            # Display
            cv2.imshow('AGV Navigation [DEBUG]', frame)
            
            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                debug_print("MAIN", "Quit requested by user", "INFO")
                break
    
    except KeyboardInterrupt:
        debug_print("MAIN", "Interrupted by user", "WARNING")
    
    except Exception as e:
        debug_print("MAIN", f"Unexpected error: {e}", "ERROR")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        debug_print("MAIN", "Cleaning up...", "INFO")
        serial_comm.send('s')  # Stop motors
        time.sleep(0.5)
        
        camera.close()
        serial_comm.status()
        serial_comm.close()
        cv2.destroyAllWindows()
        
        debug_print("MAIN", "Shutdown complete", "INFO")
        print("\n" + "="*70 + "\n")

if __name__ == "__main__":
    main()
