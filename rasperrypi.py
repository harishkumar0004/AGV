USB_PORT = "/dev/ttyUSB0"   # Change if needed
import serial
def print_commands():
    print("Available commands:")
    print("f - Move Forward")
    print("r - Move Backward")
    print("s - Stop Motor")
try:
    usb = serial.Serial(USB_PORT, 9600, timeout=1)
except:
    print("Error - could not open serial port")
    print("Exiting program")
    exit()
print("Enter a command from keyboard to send to the Arduino.")
print_commands()
while True:
    command = input("Enter command: ")
    if command == "f":
        usb.write(b'f')
        print("Motor moving forward")
    elif command == "r":
        usb.write(b'r')
        print("Motor moving backward")
    elif command == "s":
        usb.write(b's')
        print("Motor is stopped")
    else:
        print("Unknown command:", command)
        print_commands()