# AGV Navigation - Master-Slave Architecture

## Overview

This project implements a **Master-Slave** architecture for autonomous ground vehicle (AGV) navigation using AprilTag detection and trapezoidal motion profiles.

- **Master**: Python on Raspberry Pi (camera + navigation logic)
- **Slave**: Arduino/Mega (motor controller + motion execution)

The Python master detects AprilTags via camera and sends movement commands to the Arduino slave, which executes precise motor control with acceleration/deceleration profiles.

---

## Architecture Diagram

```
Raspberry Pi (Master)           Arduino/Mega (Slave)
┌─────────────────────┐         ┌──────────────────┐
│  Camera (AprilTag)  │         │  Stepper Motors  │
│      Detector       │         │  Trapezoidal     │
└──────────┬──────────┘         │    Profiles      │
           │                    └────────┬─────────┘
           │                            │
           ├─ Serial Commands ────→ ──┤
           │  (f/p/s)                  │
           │                            │
           ←─ Status Feedback ─────←───┤
           │  (EVENT messages)
```

### Command Protocol

| Command | Effect | Arduino Response |
|---------|--------|------------------|
| `f` | Forward motion | `EVENT:FORWARD_START` → `EVENT:MOTION_COMPLETE` |
| `p` | Pivot 180° | `EVENT:PIVOT_START` → `EVENT:PIVOT_DONE` |
| `s` | Stop immediately | `EVENT:STOPPED` |
| `STATUS` | Query state | `STATUS:state=...` |

---

## File Structure

```
RasperrPi_Arduino_Prj/
├── agv_controller_slave.ino          # Arduino code (SLAVE)
├── camera_detection_master.py         # Python code (MASTER) - USE THIS
├── camera_detection_code_using_Pi.py # Old version - DO NOT USE
├── setup_rpi.sh                      # Raspberry Pi setup script
└── README.md                         # This file
```

---

## Installation

### 1. Raspberry Pi Setup

```bash
# Run setup script
bash setup_rpi.sh

# Or manually install dependencies
pip3 install pyserial opencv-python apriltag numpy
```

### 2. Arduino/Mega Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Open: `agv_controller_slave.ino`
3. Select board: **Arduino Mega 2560**
4. Connect Arduino via USB
5. Upload code (Ctrl+U)
6. Verify upload succeeded in Serial Monitor

### 3. Configuration

Edit `camera_detection_master.py`:

```python
SERIAL_PORT = "/dev/ttyUSB0"  # Change if different (use: ls /dev/tty*)
BAUD_RATE = 115200             # Must match Arduino code
TAG_START = 1                  # AprilTag ID for start
TAG_TURN = 2                   # AprilTag ID for turn
```

---

## Running on Raspberry Pi

### Start the Master (Python)

```bash
python3 camera_detection_master.py
```

The program will:
1. Connect to Arduino slave via serial
2. Wait for Arduino ready message
3. Open camera and initialize AprilTag detector
4. Display live video with tag detection
5. Send commands to Arduino based on detected tags
6. Display slave status (speed, motor state)

### Keyboard Controls

- **`q`**: Quit and shutdown
- **`Ctrl+C`**: Emergency stop

---

## Program Flow (Stage 1)

```
[START]
   ↓
[Wait for Tag 1] ← Send 'f' continuously ← [Detect Tag 1]
   ↓
[Moving Forward] ← Refresh 'f' every 1.0s ← [Wait for Tag 2]
   ↓
[Detect Tag 2] → Send 'p' (pivot) ← [Start Pivot]
   ↓
[Pivoting] ← Wait for EVENT:PIVOT_DONE (timeout: 30s)
   ↓
[Receive PIVOT_DONE] → Send 's' (stop)
   ↓
[STAGE 1 COMPLETE] ← Display "Stage 1 complete"
   ↓
[Press q to quit]
   ↓
[END]
```

---

## Serial Communication Details

### Baud Rate
- **Important**: Both Python and Arduino use **115200 baud** (not 9600)
- This allows fast, reliable motor command updates

### Message Format
All messages end with `\n` (newline)

**Arduino → Python (Slave sends):**
```
READY:AGV_Controller_Slave      # On startup
EVENT:FORWARD_START              # When motion begins
EVENT:MOTION_COMPLETE            # When trapezoid finishes
EVENT:PIVOT_START
EVENT:PIVOT_DONE                 # Critical feedback for stage completion
EVENT:STOPPED
STATUS:state=STOPPED|speed=0.0rpm
```

**Python → Arduino (Master sends):**
```
f\n                              # Forward command
p\n                              # Pivot command
s\n                              # Stop command
STATUS\n                         # Query current state
```

---

## Motor Configuration

### Pins (Arduino/Mega)
```
Motor 1:
  STEP_PIN1 = 22
  DIR_PIN1  = 24
  EN_PIN1   = 26

Motor 2:
  STEP_PIN2 = 23
  DIR_PIN2  = 25
  EN_PIN2   = 27
```

### Motion Profiles
```python
PULSES_PER_REV = 10000       # Pulses per revolution
MAX_RPM = 100.0              # Max speed
INITIAL_RPM = 5.0            # Starting speed
ACC_RPM_PER_SEC = 20.0       # Acceleration rate
ACCEL_TIME = 4.76s           # Time to reach max RPM
CRUISE_TIME = 3.0s           # Constant speed duration
```

**Trapezoidal motion profile:**
```
Speed
  │     ╱╲
  │    ╱  ╲      Cruise
  │   ╱    ╲
  │  ╱      ╲    Decel
  │ ╱        ╲
  │╱          ╲
  └────────────────── Time
    Accel
```

---

## Display Information

### Video Feed Shows:
- **Yellow line**: Center of frame (reference)
- **Green box**: AprilTag 1 (START)
- **Orange box**: AprilTag 2 (TURN)
- **Red dot**: Tag center
- **Blue line**: Offset from center
- **Text overlays**: Status messages and debug info

### Status Messages:
- Master state (waiting, moving, pivoting, complete)
- Slave state (motor speed, direction)
- Time remaining (for timeouts)
- Event history from Arduino

---

## Troubleshooting

### Arduino not detected
```bash
# List serial ports
ls /dev/tty*
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Set permissions if needed
sudo chmod 666 /dev/ttyUSB0
```

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Communication fails
- Check USB cable connection
- Verify Arduino is programmed with `agv_controller_slave.ino`
- Check baud rate matches (115200)
- Try different USB port

### Camera not detected
```bash
# Test camera
raspistill -o test.jpg

# Check camera interface in raspi-config
sudo raspi-config
# → Interfacing Options → Camera → Enable
```

### AprilTags not detected
- Ensure good lighting
- Tags must be flat and fully visible
- Try different camera angle
- Check tag IDs (default: Tag 1 and 2)

### Motor not moving
- Check motor wiring
- Verify enable pins are LOW (motors enabled)
- Check step/direction pins match configuration
- Test with simple Arduino sketch

---

## Performance Tips

1. **Lighting**: Use good lighting for reliable tag detection
2. **Camera angle**: Mount camera to see tags ahead of motion
3. **Tag size**: Use sufficiently large tags (10-15cm for distance)
4. **Serial priority**: Serial reads are non-blocking to keep video smooth
5. **Monitoring**: Check Arduino after upload for any errors

---

## Next Steps / Extensions

- **Stage 2**: Add waypoint navigation
- **Stage 3**: Implement full path planning with obstacle avoidance
- **Odometry**: Add encoders for position tracking
- **PID Tuning**: Optimize acceleration/deceleration rates
- **Safety**: Add bumper/proximity sensors
- **Logging**: Record navigation data to file

---

## Support

- Check Arduino IDE Serial Monitor for raw messages
- Enable debug output in Python (uncomment verbose logging)
- Test serial connection with: `cat /dev/ttyUSB0` (Ctrl+C to exit)

---

**Author**: AGV Navigation Project  
**Last Updated**: 2026  
**Architecture**: Master-Slave (Synchronized)
