import serial
import time

ser = serial.Serial('COM3', 115200, timeout=1)

def send_command(speed: int, direction: int, steer: int):
    speed = max(0, min(speed, 255))
    direction = 0 if direction == 0 else 1
    steer = max(-90, min(steer, 90))
    packet = f"{speed},{direction},{steer},0,0\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent -> {packet.strip()}")

def read_response():
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            parts = line.split(',')
            if len(parts) == 4:
                heading = int(parts[0])
                left = int(parts[1])
                front = int(parts[2])
                right = int(parts[3])
                print(f"Received -> Heading: {heading}, Left: {left} cm, Front: {front} cm, Right: {right} cm")

try:
    while True:
        send_command(0, 0, 30)
        time.sleep(0.5)
        send_command(0, 0, -30)
        time.sleep(0.5)
        read_response()

except KeyboardInterrupt:
    send_command(0, 0, 0)
    print("Stopped.")
