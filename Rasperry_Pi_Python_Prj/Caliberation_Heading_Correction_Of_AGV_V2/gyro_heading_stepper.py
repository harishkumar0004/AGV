#!/usr/bin/env python3
"""
AGV Control System v4 - Raspberry Pi
Clean rewrite: heartbeat only during active movement,
rotation commands wait for ROT_DONE before accepting next command.
"""

import serial
import sys
import time
import select
from threading import Thread, Event, Lock


class AGVController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"Connected: {port} @ {baudrate}")
            time.sleep(2)
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            print(f"Cannot open {port}: {e}")
            self._list_ports()
            sys.exit(1)

        self.running          = True
        self.current_cmd      = 's'
        self.motors_active    = False   # True only when AGV is actually moving
        self.rotating         = False   # True during l/r command until ROT_DONE
        self._stop_event      = Event()
        self._serial_lock     = Lock()

        # Heartbeat: only resends command while motors are active
        # Interval must be < Arduino CMD_TIMEOUT (700ms)
        self.heartbeat_interval = 0.25  # 250ms

        self._read_thread = Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()

        self._hb_thread = Thread(target=self._heartbeat, daemon=True)
        self._hb_thread.start()

        # Verify connection
        time.sleep(0.5)
        self._ping()

    # --------------------------------------------------
    def _ping(self):
        try:
            with self._serial_lock:
                self.ser.write(b'?')
            start = time.time()
            while time.time() - start < 2.0:
                time.sleep(0.1)
                # response printed by _read_serial thread
            print("Ping sent — check above for ACK")
        except Exception as e:
            print(f"Ping error: {e}")

    # --------------------------------------------------
    def _read_serial(self):
        """Background thread: read and print Arduino output."""
        while self.running and not self._stop_event.is_set():
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    # Detect rotation complete
                    if line.startswith("ROT_DONE"):
                        self.rotating     = False
                        self.motors_active = False
                        self.current_cmd  = 's'
                        print(f"\n[Arduino] {line}  ← rotation complete")
                    else:
                        print(f"  [Arduino] {line}")

            except serial.SerialException:
                break
            except Exception as e:
                print(f"Read error: {e}")
            time.sleep(0.005)

    # --------------------------------------------------
    def _heartbeat(self):
        """
        Resend current command only while motors are active.
        This prevents the Arduino CMD_TIMEOUT from stopping the AGV
        during long forward/backward runs.
        When stopped or after rotation completes, sends nothing
        (Arduino is already stopped, no need to keep pinging 's').
        """
        while self.running and not self._stop_event.is_set():
            if self.motors_active:
                try:
                    with self._serial_lock:
                        self.ser.write(self.current_cmd.encode())
                except Exception as e:
                    print(f"Heartbeat error: {e}")
            time.sleep(self.heartbeat_interval)

    # --------------------------------------------------
    def send_command(self, cmd):
        """Send a single-char command to Arduino."""
        if cmd not in ('f', 'b', 'l', 'r', 's'):
            print("Invalid command. Use: f b l r s")
            return False

        # Block new movement commands while rotating
        if self.rotating and cmd in ('f', 'b', 'l', 'r'):
            print("Busy rotating — wait for ROT_DONE or send 's' to abort")
            return False

        try:
            with self._serial_lock:
                self.ser.write(cmd.encode())

            self.current_cmd = cmd

            if cmd == 's':
                self.motors_active = False
                self.rotating      = False
            elif cmd in ('l', 'r'):
                self.motors_active = True
                self.rotating      = True   # wait for ROT_DONE
            else:
                self.motors_active = True
                self.rotating      = False

            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    # --------------------------------------------------
    def wait_rotation_done(self, timeout=10.0):
        """Block until rotation completes or timeout."""
        start = time.time()
        while self.rotating:
            if time.time() - start > timeout:
                print("Rotation timeout!")
                return False
            time.sleep(0.1)
        return True

    # --------------------------------------------------
    def _list_ports(self):
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        print("Available ports:", ports if ports else "none found")

    # --------------------------------------------------
    def close(self):
        if not self.running:
            return
        self.running = False
        self._stop_event.set()
        try:
            if self.ser.is_open:
                self.ser.write(b's')
                time.sleep(0.2)
                self.ser.close()
                print("Disconnected")
        except Exception:
            pass


# ======================================================
# INTERACTIVE MODE
# ======================================================
def interactive_mode(agv):
    print("\n" + "=" * 45)
    print(" AGV Control  v4 — Interactive")
    print("=" * 45)
    print("  f  — Forward (hold key, send 's' to stop)")
    print("  b  — Backward")
    print("  l  — Turn Left 90°  (auto-stops)")
    print("  r  — Turn Right 90° (auto-stops)")
    print("  s  — Stop")
    print("  q  — Quit")
    print("=" * 45)
    print("NOTE: For f/b keep sending the command.")
    print("      l/r auto-stop when 90° is reached.\n")

    try:
        while agv.running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue

            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if not cmd:
                continue

            if cmd == 'q':
                print("Quitting...")
                break
            elif cmd in ('f', 'b', 'l', 'r', 's'):
                if agv.send_command(cmd):
                    labels = {
                        'f': 'Forward',
                        'b': 'Backward',
                        'l': 'Left 90° (waiting for completion...)',
                        'r': 'Right 90° (waiting for completion...)',
                        's': 'Stop'
                    }
                    print(f">> {labels[cmd]}")
            else:
                print(f"Unknown '{cmd}'. Use f/b/l/r/s/q")

    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        agv.close()


# ======================================================
# MAIN
# ======================================================
def main():
    print("AGV Control System v4")
    print("=" * 30)

    port = '/dev/ttyUSB0'

    # Allow port override: python3 agv.py /dev/ttyACM0
    if len(sys.argv) > 1 and sys.argv[1].startswith('/dev/'):
        port = sys.argv[1]

    agv = AGVController(port=port)
    interactive_mode(agv)


if __name__ == '__main__':
    main()
