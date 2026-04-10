#!/usr/bin/env python3
"""
AGV Serial Communication Tester
Tests communication between Raspberry Pi and Arduino Slave
Run this before running the main camera detection code
"""

import serial
import time
import sys

def test_serial_connection(port="/dev/ttyUSB0", baud=115200):
    """Test basic serial communication"""
    
    print("="*60)
    print("AGV Serial Communication Tester")
    print("="*60)
    print()
    
    # Test connection
    print(f"[TEST 1] Connecting to {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=2)
        time.sleep(2)  # Wait for Arduino reset
        print("✓ Serial connection successful")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        print("  Solutions:")
        print("  - Check USB cable")
        print("  - Verify port: ls /dev/tty*")
        print("  - Check permissions: ls -la /dev/ttyUSB0")
        return False
    
    print()
    
    # Test READY message
    print("[TEST 2] Waiting for Arduino READY message...")
    ser.reset_input_buffer()
    ready_received = False
    start = time.time()
    
    while time.time() - start < 5:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"  Received: {line}")
                if "READY" in line:
                    ready_received = True
                    print("✓ Arduino is responding")
                    break
        time.sleep(0.1)
    
    if not ready_received:
        print("✗ No READY message received")
        print("  Solutions:")
        print("  - Upload agv_controller_slave.ino to Arduino")
        print("  - Check Arduino board selection (should be Mega 2560)")
        print("  - Verify baud rate is 115200 in Arduino code")
        ser.close()
        return False
    
    print()
    
    # Test FORWARD command
    print("[TEST 3] Sending FORWARD command ('f')...")
    ser.write(b"f\n")
    time.sleep(0.5)
    
    forward_received = False
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"  Received: {line}")
            if "FORWARD" in line:
                forward_received = True
    
    if forward_received:
        print("✓ Forward command acknowledged")
    else:
        print("⚠ No response to forward command")
    
    print()
    
    # Test STATUS query
    print("[TEST 4] Querying STATUS...")
    ser.write(b"STATUS\n")
    time.sleep(0.5)
    
    status_received = False
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"  Status: {line}")
            if "STATUS:" in line:
                status_received = True
    
    if status_received:
        print("✓ Status query successful")
    else:
        print("⚠ No status response")
    
    print()
    
    # Test STOP command
    print("[TEST 5] Sending STOP command ('s')...")
    ser.write(b"s\n")
    time.sleep(0.5)
    
    stop_received = False
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"  Received: {line}")
            if "STOPPED" in line:
                stop_received = True
    
    if stop_received:
        print("✓ Stop command acknowledged")
    else:
        print("⚠ No response to stop command")
    
    print()
    ser.close()
    
    # Summary
    print("="*60)
    print("Test Summary:")
    print("="*60)
    
    all_passed = ready_received and forward_received and status_received and stop_received
    
    if all_passed:
        print("✓ All tests PASSED")
        print("\nYou can now run:")
        print("  python3 camera_detection_master.py")
    else:
        print("✗ Some tests FAILED")
        print("\nDebug options:")
        print("  1. Check Arduino Serial Monitor for raw output")
        print("  2. Verify agv_controller_slave.ino is uploaded")
        print("  3. Check USB cable and port connection")
    
    print()
    return all_passed

def test_camera():
    """Test camera availability"""
    try:
        import cv2
        print("[BONUS] Testing Camera...")
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            print(f"✓ Camera works - Resolution: {frame.shape[1]}x{frame.shape[0]}")
            return True
        else:
            print("✗ Camera not accessible")
            return False
    except Exception as e:
        print(f"✗ Camera test failed: {e}")
        return False

def test_apriltag():
    """Test apriltag library"""
    try:
        print("[BONUS] Testing AprilTag library...")
        import apriltag
        detector = apriltag.apriltag()
        print("✓ AprilTag library loaded")
        return True
    except Exception as e:
        print(f"✗ AprilTag test failed: {e}")
        return False

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Using port: {port}\n")
    
    if test_serial_connection(port):
        print()
        test_camera()
        print()
        test_apriltag()
    
    print()
    print("For detailed help, see README.md")
