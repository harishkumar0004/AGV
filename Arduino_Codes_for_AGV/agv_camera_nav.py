#!/usr/bin/env python3
"""
AGV Camera-Based Navigation using AprilTags
Simple, clean code for autonomous navigation
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

# Navigation tuning (in pixels)
CENTER_THRESHOLD = 40      # Tag must be within ±40px of center
TURN_SENSITIVITY = 100     # How far off-center before turning

# ===== TAGS =====
TAG_FORWARD = 1
TAG_TURN = 2

# ===== SERIAL CONNECTION =====
def connect_arduino(port="/dev/ttyACM0", baudrate=9600):
    """Connect to Arduino"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print(f"✓ Connected to Arduino on {port}")
        return ser
    except:
        print(f"✗ Failed to connect to Arduino on {port}")
        print("  Try: ls /dev/tty* to find your port")
        exit()

def send_command(ser, cmd):
    """Send command to Arduino (f/l/r/s)"""
    try:
        ser.write(cmd.encode())
        time.sleep(0.05)
    except Exception as e:
        print(f"Error: {e}")

# ===== CAMERA SETUP =====
def setup_camera():
    """Initialize camera"""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap

# ===== APRILTAG SETUP =====
def setup_detector():
    """Initialize AprilTag detector"""
    return apriltag.apriltag(
        family='tag36h11',
        threads=4,
        maxhamming=1,
        decimate=2.0,
        blur=0.0,
        refine_edges=True,
        debug=False
    )

# ===== NAVIGATION LOGIC =====
def get_tag_offset(det):
    """Get horizontal offset of tag from frame center"""
    center_x = int(det['center'][0])
    offset = center_x - FRAME_CENTER
    return offset

def navigate(tag_id, offset, ser):
    """
    Decide movement based on detected tag and its position
    """
    if tag_id == TAG_FORWARD:
        # Move forward with centering
        if abs(offset) < CENTER_THRESHOLD:
            send_command(ser, 'f')
            return "FORWARD (centered)"
        elif offset < -TURN_SENSITIVITY:
            send_command(ser, 'l')
            return f"LEFT (offset: {offset})"
        elif offset > TURN_SENSITIVITY:
            send_command(ser, 'r')
            return f"RIGHT (offset: {offset})"
        else:
            send_command(ser, 'f')
            return f"FORWARD (correcting)"
    
    elif tag_id == TAG_TURN:
        # Turn decision based on tag position
        if offset < -50:
            send_command(ser, 'l')
            return f"TURN LEFT (tag left: {offset})"
        elif offset > 50:
            send_command(ser, 'r')
            return f"TURN RIGHT (tag right: {offset})"
        else:
            send_command(ser, 's')
            return "STOP (tag centered)"
    
    return None

# ===== VISUALIZATION =====
def draw_navigation_overlay(frame, tag_id, offset, status):
    """Draw navigation info on frame"""
    # Frame center line (reference)
    cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), 
            (0, 255, 255), 2)  # Yellow
    
    # Title
    cv2.putText(frame, "AGV Navigation", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Status
    if status:
        cv2.putText(frame, f"Command: {status}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame

# ===== MAIN NAVIGATION LOOP =====
def main():
    """Main navigation loop"""
    print("\n" + "="*50)
    print("AGV Camera-Based Navigation")
    print("="*50)
    print(f"Tag {TAG_FORWARD}: Move forward")
    print(f"Tag {TAG_TURN}: Turn decision")
    print("Press 'q' to quit\n")
    
    # Initialize
    cap = setup_camera()
    detector = setup_detector()
    arduino = connect_arduino()
    
    frame_count = 0
    last_command_time = 0
    command_interval = 0.3  # Send command every 0.3 seconds
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect tags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)
            
            # Process detections
            navigation_status = None
            
            if detections:
                det = detections[0]  # Use first detected tag
                tag_id = det['id']
                
                # Draw tag boundary
                if tag_id in [TAG_FORWARD, TAG_TURN]:
                    pts = det['lb-rb-rt-lt'].astype(int)
                    color = (0, 255, 0) if tag_id == TAG_FORWARD else (0, 165, 255)
                    cv2.polylines(frame, [pts], True, color, 3)
                    
                    # Draw center point
                    cx, cy = int(det['center'][0]), int(det['center'][1])
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    
                    # Draw offset line
                    cv2.line(frame, (FRAME_CENTER, cy), (cx, cy), (255, 0, 0), 2)
                    
                    # Display tag ID and offset
                    cv2.putText(frame, f"ID:{tag_id}", (cx-20, cy-30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # Get offset and navigate
                    offset = get_tag_offset(det)
                    current_time = time.time()
                    
                    if (current_time - last_command_time) > command_interval:
                        navigation_status = navigate(tag_id, offset, arduino)
                        last_command_time = current_time
                        print(f"[{frame_count}] {navigation_status}")
            else:
                # No tag detected - stop
                send_command(arduino, 's')
            
            # Draw overlay
            frame = draw_navigation_overlay(frame, 
                                           detections[0]['id'] if detections else None,
                                           get_tag_offset(detections[0]) if detections else 0,
                                           navigation_status)
            
            # Display
            cv2.imshow('AGV Navigation', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            frame_count += 1
    
    except KeyboardInterrupt:
        print("\n✓ Navigation stopped")
    
    finally:
        # Cleanup
        send_command(arduino, 's')  # Stop motors
        cap.release()
        cv2.destroyAllWindows()
        arduino.close()
        print("✓ Cleanup complete")

if __name__ == "__main__":
    main()
