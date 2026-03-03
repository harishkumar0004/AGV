#!/usr/bin/env python3
"""
AGV Control System for Raspberry Pi
Controls the AGV via Serial Communication with Arduino
Commands: f (forward), b (backward), l (left), r (right), s (stop)

Fix: Added heartbeat keep-alive so Arduino doesn't auto-stop.
Fix: Replaced input()+select combo with reliable readline approach.
Fix: Safe cleanup on exit.
"""

import serial
import sys
import time
import select
from threading import Thread, Event


class AGVController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.1):
        """
        Initialize AGV Controller

        Args:
            port: Serial port (default: /dev/ttyUSB0)
            baudrate: Baud rate (default: 115200)
            timeout: Serial timeout in seconds
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Connected to {port} at {baudrate} baud")
            time.sleep(2)  # Wait for Arduino to initialize

            self.ser.flushInput()
            self.ser.flushOutput()

        except serial.SerialException as e:
            print(f"Error: Could not open serial port {port}")
            print(f"Details: {e}")
            print("\nAvailable ports:")
            self.list_ports()
            sys.exit(1)

        self.running = True
        self.current_command = 's'
        self._stop_event = Event()

        # Heartbeat interval must be less than Arduino's commandTimeout (600ms)
        self.heartbeat_interval = 0.2  # 200ms

        # Start background threads
        self.read_thread = Thread(target=self._read_serial, daemon=True)
        self.read_thread.start()

        self.heartbeat_thread = Thread(target=self._heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def _read_serial(self):
        """Read and display serial data from Arduino"""
        while self.running and not self._stop_event.is_set():
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  [Arduino] {line}")
            except serial.SerialException:
                break
            except Exception as e:
                print(f"Read error: {e}")
            time.sleep(0.01)

    def _heartbeat(self):
        """
        Continuously resend the current command so Arduino doesn't auto-stop.
        Arduino has a 600ms command timeout; we send every 200ms.
        """
        while self.running and not self._stop_event.is_set():
            try:
                if self.ser.is_open:
                    self.ser.write(self.current_command.encode())
            except serial.SerialException:
                break
            except Exception as e:
                print(f"Heartbeat error: {e}")
            time.sleep(self.heartbeat_interval)

    def send_command(self, cmd):
        """
        Send command to Arduino.

        Args:
            cmd: Single character command (f, b, l, r, s)
        """
        if cmd not in ['f', 'b', 'l', 'r', 's']:
            print("Invalid command. Use: f (forward), b (backward), l (left), r (right), s (stop)")
            return False

        try:
            self.current_command = cmd  # Heartbeat will keep sending this
            self.ser.write(cmd.encode())
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def list_ports(self):
        """List available serial ports"""
        try:
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                for port in ports:
                    print(f"  {port}")
            else:
                print("  No serial ports found")
        except Exception as e:
            print(f"  Error listing ports: {e}")

    def close(self):
        """Close serial connection safely"""
        if not self.running:
            return  # Already closed
        self.running = False
        self._stop_event.set()
        try:
            if self.ser.is_open:
                self.ser.write(b's')  # Stop command before closing
                time.sleep(0.2)
                self.ser.close()
                print("Connection closed")
        except Exception:
            pass


def interactive_mode(controller):
    """Interactive keyboard command mode"""
    print("\n" + "=" * 50)
    print("AGV Control System - Interactive Mode")
    print("=" * 50)
    print("  f - Move Forward")
    print("  b - Move Backward")
    print("  l - Turn Left")
    print("  r - Turn Right")
    print("  s - Stop")
    print("  h - Help")
    print("  q - Quit")
    print("=" * 50 + "\n")

    try:
        while controller.running:
            # Use select for non-blocking stdin check
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                line = sys.stdin.readline()
                if not line:  # EOF
                    break
                cmd = line.strip().lower()

                if not cmd:
                    continue
                elif cmd == 'q':
                    print("Exiting...")
                    break
                elif cmd == 'h':
                    print("Commands: f=forward, b=backward, l=left, r=right, s=stop, q=quit")
                elif cmd in ['f', 'b', 'l', 'r', 's']:
                    if controller.send_command(cmd):
                        labels = {
                            'f': 'Forward', 'b': 'Backward',
                            'l': 'Left',    'r': 'Right', 's': 'Stop'
                        }
                        print(f">> {labels[cmd]}")
                else:
                    print(f"Unknown command '{cmd}'. Type 'h' for help.")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.close()


def autonomous_mode(controller, sequence):
    """
    Execute a sequence of commands.

    Sequence characters:
      f/b/l/r/s = movement commands
      p         = pause 1 second
      0-9       = hold previous command for N seconds
    """
    print(f"\nExecuting sequence: {sequence}")
    print("=" * 50)

    try:
        i = 0
        while i < len(sequence):
            cmd = sequence[i]
            if cmd == 'p':
                print("Pause 1s")
                controller.send_command('s')
                time.sleep(1)
            elif cmd.isdigit():
                duration = int(cmd)
                print(f"Hold {controller.current_command} for {duration}s")
                time.sleep(duration)
            elif cmd in ['f', 'b', 'l', 'r', 's']:
                controller.send_command(cmd)
                labels = {'f': 'Forward', 'b': 'Backward', 'l': 'Left', 'r': 'Right', 's': 'Stop'}
                print(f"CMD: {labels[cmd]}")
                time.sleep(0.3)  # Short settle time
            i += 1

        controller.send_command('s')
        print("Sequence complete")
    except KeyboardInterrupt:
        print("\nSequence interrupted")
        controller.send_command('s')
    finally:
        time.sleep(0.5)
        controller.close()


def continuous_mode(controller, command, duration):
    """Send a command continuously for a specified duration (heartbeat handles resending)"""
    labels = {'f': 'Forward', 'b': 'Backward', 'l': 'Left', 'r': 'Right', 's': 'Stop'}
    label = labels.get(command, command)
    print(f"\nRunning '{label}' for {duration} seconds...")
    print("=" * 50)

    try:
        controller.send_command(command)
        time.sleep(duration)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        controller.send_command('s')
        time.sleep(0.3)
        controller.close()


def main():
    """Main entry point"""
    print("AGV Control System v2.0")
    print("========================")

    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

        if mode in ('--help', '-h'):
            print("\nUsage:")
            print("  python3 agv_control.py [MODE] [OPTIONS]")
            print("\nModes:")
            print("  interactive               - Interactive keyboard control (default)")
            print("  sequence [cmds]           - Execute command sequence")
            print("                              Digits 0-9 = hold for N seconds")
            print("                              p = pause 1 second")
            print("                              e.g. 'f3l1r1b3s' = forward 3s, left 1s...")
            print("  continuous [cmd] [secs]   - Run command for N seconds")
            print("\nSerial Port:")
            print("  Default: /dev/ttyUSB0")
            print("  Edit AGVController(port=...) to change")
            sys.exit(0)

        elif mode == 'sequence' and len(sys.argv) > 2:
            controller = AGVController()
            autonomous_mode(controller, sys.argv[2])

        elif mode == 'continuous' and len(sys.argv) > 3:
            cmd = sys.argv[2].lower()
            try:
                duration = float(sys.argv[3])
                controller = AGVController()
                continuous_mode(controller, cmd, duration)
            except ValueError:
                print("Error: Duration must be a number")
                sys.exit(1)

        elif mode == 'interactive':
            controller = AGVController()
            interactive_mode(controller)

        else:
            print("Invalid arguments. Use --help for usage.")
            sys.exit(1)

    else:
        controller = AGVController()
        interactive_mode(controller)


if __name__ == '__main__':
    main()
