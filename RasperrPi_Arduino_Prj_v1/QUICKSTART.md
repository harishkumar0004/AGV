# AGV Master-Slave Quick Start Guide

## ⚡ 5-Minute Setup

### Step 1: Upload Arduino Code
```
1. Connect Arduino/Mega via USB to computer
2. Open Arduino IDE
3. Open: agv_controller_slave.ino
4. Select: Tools → Board → Arduino Mega 2560
5. Click: Upload (Ctrl+U)
6. Wait for "Upload successful" message
```

### Step 2: Raspberry Pi - Install Dependencies
```bash
# SSH into your Raspberry Pi
ssh pi@raspberrypi.local

# Navigate to project folder
cd /path/to/RasperrPi_Arduino_Prj

# Run setup (or install manually)
bash setup_rpi.sh

# OR manual install:
pip3 install pyserial opencv-python apriltag
```

### Step 3: Test Serial Connection
```bash
# First, verify Arduino-RPi communication works
python3 test_serial.py

# If connected via different USB port:
python3 test_serial.py /dev/ttyACM0
```

### Step 4: Run AGV Navigation
```bash
python3 camera_detection_master.py
```

✅ Ready! The program will:
- Connect to Arduino slave
- Open camera and detect AprilTags
- Send commands to motors
- Display live status

---

## 🔧 Configuration Checklist

Before running, verify:

```
✓ Arduino uploaded with agv_controller_slave.ino
✓ USB cable connected (Arduino to Raspberry Pi)
✓ Camera connected and enabled (raspi-config)
✓ AprilTags printed and positioned
✓ Serial port correct in camera_detection_master.py
✓ Stepper motors wired to correct Arduino pins
✓ Motors powered and enable pins connected
✓ All dependencies installed (pip3 check)
```

---

## 📊 Expected Output

When you run `python3 camera_detection_master.py`:

### Console Output:
```
============================================================
[MASTER] AGV Navigation - Master Mode
Python: Master (Raspberry Pi)
Arduino: Slave (Motor Controller)
============================================================

[SERIAL] Connecting to Arduino Slave...
<<< READY:AGV_Controller_Slave
[SERIAL] ✓ Connected to Arduino Slave

[CAMERA] Opening...
[CAMERA] ✓ Ready

[DETECTOR] Initializing...
[DETECTOR] ✓ Ready

[READY] All systems initialized

>>> CMD: 's'
<<< EVENT:STOPPED
```

### Video Window Shows:
- Live camera feed
- AprilTag detections (green/orange boxes)
- Yellow center line
- Real-time status text
- Motor speed and state

---

## 🚀 Common Commands During Operation

| Key | Action |
|-----|--------|
| `q` | Exit program gracefully |
| `Ctrl+C` | Emergency stop (may need to press 's' if stuck) |

---

## 🐛 Troubleshooting Quick Reference

| Problem | Solution |
|---------|----------|
| `Connection refused` | Check USB cable, verify Arduino uploaded |
| `No READY message` | Upload agv_controller_slave.ino to Mega 2560 |
| `Serial port /dev/ttyUSB0 not found` | Run `ls /dev/tty*`, update SERIAL_PORT |
| `Permission denied` | Run `sudo chmod 666 /dev/ttyUSB0` |
| `Camera not detected` | Enable camera in `sudo raspi-config` |
| `AprilTags not detected` | Check lighting, ensure tags are visible, correct IDs |

---

## 📋 Pin Configuration

### Arduino/Mega Motor Pins
```
Motor 1:
  STEP  = Pin 22
  DIR   = Pin 24  
  EN    = Pin 26

Motor 2:
  STEP  = Pin 23
  DIR   = Pin 25
  EN    = Pin 27
```

### Serial Communication
- **Baud Rate**: 115200 (critical!)
- **Port**: /dev/ttyUSB0 (or /dev/ttyACM0)

### Camera
- Standard Raspberry Pi Camera Module
- Resolution: 640×480
- FPS: 30

---

## 💡 Architecture at a Glance

```
Python (Master)              Arduino (Slave)
   │                            │
   ├─ Detects Tag 1            │
   ├─ Sends 'f' command ──────→ │
   │                    Motor accelerates
   │
   ├─ Detects Tag 2            │
   ├─ Sends 'p' command ──────→ │
   │                    Motor pivots
   │
   ├─ Waits for PIVOT_DONE ←── │
   │                    Returns EVENT message
   │
   └─ Navigation Complete      │
```

---

## 📚 File Reference

| File | Purpose | Edit? |
|------|---------|-------|
| `camera_detection_master.py` | Main Python program | ⚠️ Only SERIAL_PORT |
| `agv_controller_slave.ino` | Arduino firmware | ✗ Upload as-is |
| `test_serial.py` | Connection tester | ✗ Run to verify |
| `setup_rpi.sh` | Install dependencies | ✗ Run once |
| `README.md` | Full documentation | ℹ️ Reference |

---

## 🎯 Stage 1 Navigation Flow

```
START
  ↓
Seek → Find Tag 1 (START)
  ↓
Move Forward (send 'f' repeatedly)
  ↓
Seek → Find Tag 2 (TURN)
  ↓
Pivot 180° (send 'p', wait for PIVOT_DONE)
  ↓
COMPLETE
  ↓
Press 'q' to exit
```

---

## 📞 Need Help?

1. **Test connection first**: `python3 test_serial.py`
2. **Check Arduino Serial Monitor** for raw messages
3. **Verify all cables** and pin connections
4. **Read README.md** for detailed troubleshooting
5. **Check motor wiring** and enable pins

---

**Ready to run?** 🚀
```bash
python3 camera_detection_master.py
```

Good luck with your AGV project!
