#!/usr/bin/env python3
"""
Stage 1 AGV Navigation using AprilTags on Raspberry Pi
Master sends commands to Arduino (Slave) via Serial
Reads feedback from Arduino and adjusts navigation accordingly
"""

import apriltag
import cv2
import serial
import time
import threading

# ===== CONFIG =====
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER = FRAME_WIDTH // 2

# IMPORTANT: Arduino code uses 115200 baud, not 9600
BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyUSB0"

TAG_START = 1
TAG_TURN = 2
FORWARD_REFRESH_INTERVAL = 1.0
PIVOT_ACK_TIMEOUT_SECONDS = 30.0

# ===== GLOBAL STATE =====
running = True
camera = None
detector = None
serial_conn = None
serial_rx_buffer = ""
serial_lock = threading.Lock()

# Arduino Status
arduino_status = {
    "ready": False,
    "state": "UNKNOWN",
    "speed_rpm": 0.0,
    "last_event": None,
    "event_time": 0.0
}

# ===== FUNCTIONS =====
def send_command(cmd):
    """Send command to Arduino (Slave)"""
    global serial_conn
    
    if serial_conn and serial_conn.is_open:
        try:
            with serial_lock:
                serial_conn.write((cmd + '\n').encode())
                print(f">>> CMD: '{cmd}'")
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

def read_serial_messages():
    """Read complete lines from Arduino without blocking the video loop."""
    global serial_rx_buffer, arduino_status

    messages = []

    if not serial_conn or not serial_conn.is_open:
        return messages

    try:
        with serial_lock:
            waiting = serial_conn.in_waiting
            if waiting <= 0:
                return messages

            serial_rx_buffer += serial_conn.read(waiting).decode(errors="ignore")

        while "\n" in serial_rx_buffer:
            line, serial_rx_buffer = serial_rx_buffer.split("\n", 1)
            line = line.strip()
            if line:
                print(f"<<< {line}")
                messages.append(line)
                
                # Parse Arduino feedback
                if line.startswith("READY:"):
                    arduino_status["ready"] = True
                    print("[MASTER] Arduino is ready (Slave connected)")
                
                elif line.startswith("EVENT:"):
                    event = line.replace("EVENT:", "")
                    arduino_status["last_event"] = event
                    arduino_status["event_time"] = time.time()
                
                elif line.startswith("STATUS:"):
                    # Parse status message from Arduino
                    status_parts = line.replace("STATUS:", "").split("|")
                    for part in status_parts:
                        if "state=" in part:
                            arduino_status["state"] = part.split("=")[1]
                        elif "speed=" in part:
                            speed_str = part.split("=")[1].replace("rpm", "").strip()
                            arduino_status["speed_rpm"] = float(speed_str)
    except Exception as e:
        print(f"[ERROR] Serial read error: {e}")

    return messages

def initialize():
    """Initialize all systems"""
    global camera, detector, serial_conn, serial_rx_buffer, arduino_status
    
    print("\n" + "="*60)
    print("[MASTER] AGV Navigation - Master Mode")
    print("Python: Master (Raspberry Pi)")
    print("Arduino: Slave (Motor Controller)")
    print("="*60 + "\n")
    
    # Serial Connection
    try:
        print("[SERIAL] Connecting to Arduino Slave...")
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        serial_conn.reset_input_buffer()
        serial_rx_buffer = ""
        
        # Wait for Arduino READY message
        start_time = time.time()
        while not arduino_status["ready"] and time.time() - start_time < 5:
            read_serial_messages()
            time.sleep(0.1)
        
        if not arduino_status["ready"]:
            print("[SERIAL] ✗ Arduino did not respond")
            return False
        
        print("[SERIAL] ✓ Connected to Arduino Slave\n")
        
        # Send initial stop command
        send_command('s')
        time.sleep(0.5)
        
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
        print("[READY] All systems initialized\n")
    except Exception as e:
        print(f"[DETECTOR] ✗ Failed: {e}\n")
        return False
    
    return True

def cleanup():
    """Clean up and exit"""
    global running, camera, serial_conn
    
    print("\n" + "="*60)
    print("[CLEANUP] Shutting down Master...")
    print("="*60 + "\n")
    
    running = False
    
    # Stop motors (tell Slave to stop)
    if serial_conn and serial_conn.is_open:
        try:
            send_command('s')
            time.sleep(0.5)
            serial_conn.close()
            print("[CLEANUP] Serial connection closed")
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
    """Main loop - Master control logic"""
    global running, camera, detector, arduino_status
    
    if not initialize():
        return
    
    window_name = "AGV Navigation - Master Control"
    
    # Stage 1 state tracking
    stage_state = "WAITING_FOR_TAG_1"
    last_forward_command_time = 0.0
    pivot_command_time = 0.0
    
    print("Press 'q' to quit\n")
    print("="*60)
    print("STAGE 1 Navigation Logic:")
    print("  1. Wait for Tag 1 → Send FORWARD to Slave")
    print("  2. Detect Tag 2 → Send PIVOT to Slave")
    print("  3. Wait for PIVOT_DONE event from Slave → Complete")
    print("="*60 + "\n")
    
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
            
            # Read all pending serial messages from Arduino
            serial_messages = read_serial_messages()
            
            # Check for completion events from Slave
            pivot_done = any("PIVOT_DONE" in msg for msg in serial_messages)
            motion_complete = any("MOTION_COMPLETE" in msg for msg in serial_messages)
            
            current_time = time.time()
            visible_tag_ids = set()
            status = ""
            
            # Draw center line
            cv2.line(frame, (FRAME_CENTER, 0), (FRAME_CENTER, FRAME_HEIGHT), (0, 255, 255), 2)
            
            # Draw title
            cv2.putText(frame, "Master: Stage 1 | Tag1: Forward | Tag2: Pivot+Stop", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw Slave status
            slave_status_text = f"Slave: {arduino_status['state']} | Speed: {arduino_status['speed_rpm']:.1f} RPM"
            cv2.putText(frame, slave_status_text, (10, 450),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
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

                    tag_name = "START" if tag_id == TAG_START else "TURN"
                    cv2.putText(frame, f"{tag_name}(ID:{tag_id}) | Offset:{offset}px", (cx - 70, cy - 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ===== STAGE 1 STATE MACHINE (MASTER) =====
            if stage_state == "WAITING_FOR_TAG_1":
                if TAG_START in visible_tag_ids:
                    send_command('f')  # Tell Slave to move forward
                    last_forward_command_time = current_time
                    stage_state = "MOVING_TO_TAG_2"
                    status = "[MASTER] Tag 1 detected → Sent FORWARD to Slave"
                else:
                    status = "[MASTER] Waiting for Tag 1 to start"

            elif stage_state == "MOVING_TO_TAG_2":
                if TAG_TURN in visible_tag_ids:
                    send_command('p')  # Tell Slave to pivot
                    pivot_command_time = current_time
                    stage_state = "WAITING_FOR_PIVOT_DONE"
                    status = "[MASTER] Tag 2 detected → Sent PIVOT to Slave"
                    
                elif current_time - last_forward_command_time >= FORWARD_REFRESH_INTERVAL:
                    # Periodically send forward to ensure motion continues
                    send_command('f')
                    last_forward_command_time = current_time
                    status = "[MASTER] Refreshing FORWARD command"
                else:
                    status = "[MASTER] Moving forward, waiting for Tag 2"

            elif stage_state == "WAITING_FOR_PIVOT_DONE":
                if pivot_done:
                    # Slave finished pivot
                    send_command('s')  # Safety stop
                    stage_state = "STAGE_1_COMPLETE"
                    status = "[MASTER] Received PIVOT_DONE from Slave → STAGE 1 COMPLETE"
                    
                elif current_time - pivot_command_time >= PIVOT_ACK_TIMEOUT_SECONDS:
                    # Timeout waiting for pivot completion
                    send_command('s')  # Emergency stop
                    stage_state = "STAGE_1_COMPLETE"
                    status = "[MASTER] Pivot timeout → Sent STOP to Slave (safety)"
                else:
                    elapsed = current_time - pivot_command_time
                    status = f"[MASTER] Waiting for Slave to complete pivot ({elapsed:.1f}s / {PIVOT_ACK_TIMEOUT_SECONDS}s timeout)"

            else:  # STAGE_1_COMPLETE
                status = "[MASTER] Stage 1 complete. Press 'q' to exit."

            # Display status
            cv2.putText(frame, status, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Display window
            cv2.imshow(window_name, frame)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\n[MASTER] Interrupted by user")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
    
    finally:
        cleanup()

if __name__ == "__main__":
    main()
