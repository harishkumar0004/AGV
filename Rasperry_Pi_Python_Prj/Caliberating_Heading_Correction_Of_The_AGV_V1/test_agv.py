#!/usr/bin/env python3
"""
Simple AGV Test Script
Tests basic functionality without interactive prompts
"""

import serial
import time
import sys

def test_connection(port='/dev/ttyUSB0', baudrate=115200):
    """Test connection to Arduino"""
    print("Testing connection...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Connected to {port}")
        time.sleep(2)

        # Read initial messages
        print("\nInitial messages from Arduino:")
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  {line}")

        return ser
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return None


def test_commands(ser):
    """Test each command"""
    print("\nTesting commands...")
    commands = [
        ('f', 'Forward for 2 seconds'),
        ('s', 'Stop for 1 second'),
        ('b', 'Backward for 2 seconds'),
        ('s', 'Stop for 1 second'),
        ('l', 'Turn Left for 2 seconds'),
        ('s', 'Stop for 1 second'),
        ('r', 'Turn Right for 2 seconds'),
        ('s', 'Stop'),
    ]

    for cmd, desc in commands:
        print(f"\n{desc}...")
        ser.write(cmd.encode())
        time.sleep(0.5)

        # Read response
        start_time = time.time()
        while time.time() - start_time < 0.5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"  {line}")

        if cmd == 'f' or cmd == 'b' or cmd == 'l' or cmd == 'r':
            time.sleep(1.5)
        else:
            time.sleep(0.5)

    print("\n✓ Command test complete")


def main():
    port = '/dev/ttyUSB0'

    if len(sys.argv) > 1:
        port = sys.argv[1]

    print("="*50)
    print("AGV Control System - Connection Test")
    print("="*50)

    ser = test_connection(port)
    if ser is None:
        sys.exit(1)

    try:
        test_commands(ser)
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    finally:
        ser.close()
        print("\n✓ Connection closed")

if __name__ == '__main__':
    main()
