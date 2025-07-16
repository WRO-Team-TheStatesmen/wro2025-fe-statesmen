import serial
import time

ser = serial.Serial('COM3', 115200, timeout=1)

def send_command(speed: int, direction: int, steer: int):
    """
    Sends 3 bytes to ESP32:
    - speed: 0-255
    - direction: 0 = forward, 1 = backward
    - steer: -90 to 90 (two's complement format)
    """
    speed = max(0, min(speed, 255))
    direction = 0 if direction == 0 else 1
    steer = max(-90, min(steer, 90))
    steer_byte = steer & 0xFF

    packet = bytes([speed, direction, steer_byte])
    ser.write(packet)
    print(f"Sent -> Speed: {speed}, Dir: {direction}, Steer: {steer}")

try:
    while True:
        send_command(0, 0, 0)
        time.sleep(2)

        send_command(0, 0, 70)
        time.sleep(2)

        send_command(0, 0, -70)
        time.sleep(2)

except KeyboardInterrupt:
    send_command(0, 0, 0)
    print("Stopped.")
