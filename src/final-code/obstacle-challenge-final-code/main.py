## Precedence Order - Main Loop
"""
1. Detect Block - imshow if debug
2. Reverse (If block detected)
3. Block Matrix
4. Turn
5. PID (If no blocks at all, speed *= 1.5)
"""

"""
Comment Key
# = Comment
## = Note / ToDo
### = Failsafe
"""

# Importing Modules
import os
import cv2
import numpy as np
import serial
import time
import threading
from gpiozero import LED
from time import sleep

cv2.setNumThreads(2)

SPEED_K = 1
SPEED_K_PARKING = 1
PARKING_SPEED = 60
TURNING_SPEED = 80
SPEED = 100
PARKING_SPEED = int(PARKING_SPEED*SPEED_K_PARKING)
TURNING_SPEED = int(TURNING_SPEED*SPEED_K)
SPEED = int(SPEED*SPEED_K)
SPEED_NO_AURA_FARM = 90

# Debugging
DEBUG = False

# Serial Values
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

LIDAR_PORT = None

# Movement
DIRECTION = None
KP = 1.0
COUNTER = 0
COUNTER_MAX = 5
target_angle = 0
BLOCK_MULITPLIER_GREEN_ANTI = 1.5

# Logic
BACK_AFTER_TURN_TIME = 0.8
BACK_BEFORE_TURN_TIME = 1.5
last_turn = time.time()
last_cooldown = time.time()
last_block_pass = time.time()
COOLDOWN = 3
InnerStuckFailsafeFlag = False
OuterStuckFailsafeFlag = False

# Data
data = None
angle, left, front, right, ir = None, None, None, None, None

# Camera Settings
BRIGHTNESS = 0
CONTRAST = 3.0
GAMMA = 0.7
inv_gamma = 1.0 / GAMMA
table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")

# Image Processing Values
red_lower   = np.array([0, 167, 120])
red_upper   = np.array([255, 255, 255])
green_lower = np.array([0, 0, 0])
green_upper = np.array([255, 87, 255])
blue_lower  = np.array([0, 137, 0])
blue_upper  = np.array([255, 177, 92])
magenta_lower = np.array([0, 160, 0])
magenta_upper = np.array([255, 255, 130])

led = LED(3)
led.off()

colors = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "orange": (255, 165, 0)
}

masks = {
    "red": (red_lower, red_upper),
    "green": (green_lower, green_upper),
    "blue": (blue_lower, blue_upper)
}

COLOR_RANGES = {
    "red":    ([0, 184, 108], [255, 255, 255]),
    "green":  ([0, 0, 0],     [255, 101, 255]),
    "blue":   ([0, 153, 0],   [255, 186, 90]),
    "orange": ([0, 130, 150], [255, 255, 255])
}

MIN_AREA = {
    "red": 500,
    "green": 500,
    "blue": 300
}

class ThreadedVideoCapture:
    """
    Drop-in replacement for cv2.VideoCapture that:
    - sets V4L2 MJPG (lighter on Pi CPU)
    - shrinks internal buffer to 1 (no multi-frame lag)
    - runs a grab thread keeping only the most recent frame
    - exposes read()/isOpened()/release() just like OpenCV
    """
    def __init__(self, device="/dev/video1"):
        # Allow overriding via env without touching logic
        w = int(os.getenv("CAM_W", "1280"))
        h = int(os.getenv("CAM_H", "720"))
        fps = int(os.getenv("CAM_FPS", "30"))

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            raise SystemExit(1)

        # Use MJPG; many USB webcams support it and it slashes CPU usage on Pi
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Keep buffer tiny to avoid latency
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Optional exposure/brightness tweaks (preserve your BRIGHTNESS via postprocess)
        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

        # Warmup: wait until we have a frame
        t0 = time.time()
        while self.frame is None and (time.time() - t0) < 2.0:
            time.sleep(0.01)

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                continue
            with self.lock:
                self.frame = f

    def read(self):
        # returns (ret, frame) like cv2.VideoCapture
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        try:
            self.t.join(timeout=0.5)
        except Exception:
            pass
        self.cap.release()

cap = ThreadedVideoCapture("/dev/video0")

# Logic Constants
program_start_time = time.time()
red_turning_values = [0, 0, 0, 0, 0, 0, 0, 5, 10, 15, 0, 10, 15, 25, 30, 5, 10, 15, 25, 35, 0, 0, -1, -1, -1]
red_turning_values_4th_turn_anti = [-5, 0, 0, 0, 0, -5, 0, 5, 10, 15, -5, 10, 15, 25, 30, -5, 10, 15, 25, 35, -5, 0, -1, -1, -1]
red_turning_values_less_than_3000 = [0, 0, 0, 0, 0, 0, 0, 5, 20, 25, -10, 10, 15, 25, 30, -10, 10, 15, 25, 35, 0, 0, -1, -1, -1]
green_turning_values = [0, 0, 0, 0, 0, -15, -10, -5, 0, 0, -30, -25, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]
green_turning_values_4th_turn_clock = [0, 0, 0, 0, 5, -15, -10, -5, 0, 5, -30, -25, -15, -10, 5, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0] ## TODO: NEED TO BE UPDATED
green_turning_values_less_than_3000 = [0, 0, 0, 0, 0, -25, -20, -5, 0, 0, -35, -30, -15, -10, 0, -35, -25, -15, -10, -5, -1, -1, -1, 0, 0]

# Setting Up Serial
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2) # Let ESP32 Initialize

# Functions
# Logic Functions
def normalize_angle(angle):
    if angle > 180:
        angle -= 360
    return angle

def angle_diff(target, current):
    diff = target - current
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff

# Communication Functions
def get_front(port=None, baud=115200, timeout=0.1):
    global DIRECTION
    if DIRECTION == "anticlocwise":
        port = "/dev/ttyAMA3"
    else:
        port = "/dev/ttyAMA0"
    if not port:
        return None
    try:
        with serial.Serial(port, baud, timeout=timeout) as ser:
            # Read 9 bytes (full TF-LUNA frame)
            frame = ser.read(9)
            if len(frame) != 9:
                return None

            # Check header (0x59 0x59)
            if frame[0] != 0x59 or frame[1] != 0x59:
                return None

            # Extract distance (low byte + high byte)
            dist = frame[2] | (frame[3] << 8)
            return dist if dist >= 0 else None
    except serial.SerialException as e:
        print(f"[ERROR] Failed to read from {port}: {e}")
        return None

def read_data():
    global LIDAR_PORT
    try:
        line = ser.readline().decode().strip()
        if not line:
            return [None, None, None, None, None]
        parts = line.split(",")
        if len(parts) != 3:
            return [None, None, None, None, None]
        angle = normalize_angle(int(parts[0]))
        left = int(parts[1])
        if DIRECTION:
            front = get_front(port=LIDAR_PORT)
        else:
            front = 100
        right = int(parts[2])
        ir = 0
        return angle, left, front, right, ir
    except Exception as e:
        print(e)
        return [None, None, None, None, None]

def wait_until_and_read_data():
    d = [None, None, None, None, None]
    while not d[2]:
        d = read_data()
        # print(d)
        flush_serial()
    return d

def read_latest():
    data = None
    while ser.in_waiting:
        try:
            data=read_data()
        except:
            continue
    return data

def send_data(speed, direction, steer):
    cmd = f"{speed},{direction},{steer}\n"
    ser.write(cmd.encode())

def flush_serial():
    while ser.in_waiting:
        ser.readline()

# Movement Functions
def steer_until_angle(current_angle, new_target, speed, direction, steer):
    if new_target > current_angle:
        sign = ">"
    else:
        sign = "<"

    data = [None, None, None, None, None]
    while not data[0]:
        data = read_data()
        flush_serial()
    angle, _, _, _, _ = data

    if sign == "<":
        while angle > new_target:
            send_data(speed, direction, steer)
            time.sleep(0.05)
            data = None
            while not data:
                data = read_latest()
            angle, _, _, _, _ = data
    elif sign == ">":
        while angle < new_target:
            send_data(speed, direction, steer)
            time.sleep(0.05)
            data = None
            while not data:
                data = read_latest()
            angle, _, _, _, _ = data

    send_data(0, 0, 0)

# Image Processing Functions
def get_frame(cap):
    ret, frame = cap.read()
    while not ret:
        ret, frame = cap.read()
    return frame

def adjust_frame(frame):
    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)

    inv_gamma = 1.0 / GAMMA
    table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)

    return adjusted

def process_frame(frame, masks, colors, DIR, SHOW=False):
    results = []

    frame = adjust_frame(frame)

    height = frame.shape[0]

    if DIR == "clockwise":
        third_line = height // 3
    else:
        third_line = height // 2

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    for name, (lower, upper) in masks.items():
        mask = cv2.inRange(lab, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                if y + h > third_line:
                    results.append((name, area))

                    if SHOW:
                        # Draw bounding rectangle
                        cv2.rectangle(frame, (x, y), (x + w, y + h), colors[name], 2)
                        # Put label
                        cv2.putText(frame, f"{name} ({area})", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[name], 2)

    results.sort(key=lambda t: t[1], reverse=True)

    if SHOW:
        cv2.line(frame, (0, third_line), (frame.shape[1], third_line), (0, 255, 255), 2)

        # cv2.imshow("Processed Frame", frame)
        # cv2.waitKey(1)

    return results

def detect_biggest_block(frame, SHOW=False):
    global last_turn, DIRECTION, last_cooldown
    full_h, full_w, _ = frame.shape
    roi_start = int(full_h * 0.2)
    frame = frame[roi_start:full_h, 0:full_w]

    adjusted = cv2.convertScaleAbs(frame, alpha=CONTRAST, beta=BRIGHTNESS)
    adjusted = cv2.LUT(adjusted, table)

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l)
    lab_clahe = cv2.merge((l_clahe, a, b))
    adjusted_frame = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

    lab_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2LAB)
    lab_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    detections = {"red": [], "green": [], "blue": [], "orange": []}
    blue_y_values, orange_y_values = [], []

    for color, (lower, upper) in COLOR_RANGES.items():
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        source_lab = lab_raw if color == "orange" else lab_frame
        mask = cv2.inRange(source_lab, lower_bound, upper_bound)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA.get(color, 0):
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            detections[color].append((cx, cy, area, color))
            if color == "blue":
                blue_y_values.append(cy)
            elif color == "orange":
                orange_y_values.append(cy)
            if SHOW:
                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), colors[color], 2)
                cv2.circle(frame, (cx, cy), 4, colors[color], -1)
                cv2.putText(frame, f"{color} ({area})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[color], 2)

    max_blue_y = max(blue_y_values) if blue_y_values else None
    max_orange_y = max(orange_y_values) if orange_y_values else None

    valid_blocks = []
    for color in ["red", "green"]:
        for (cx, cy, area, col) in detections[color]:
            if time.time() - last_cooldown > COOLDOWN:
                if max_blue_y and cy <= max_blue_y:
                    continue
                if max_orange_y and cy <= max_orange_y:
                    continue

            ### CODENAME PENDING FAILSAFE
            cy_full = cy + roi_start
            cell_w = full_w // 5
            cell_h = full_h // 5
            col_idx = cx // cell_w
            row_idx = cy_full // cell_h
            cell_num = row_idx * 5 + col_idx + 1

            if DIRECTION == "anticlockwise" and cell_num in [1, 6, 11, 16, 21]:
                if time.time() - last_turn <= 1:
                    print("CODENAMING PENDING FAILSAFE")
                    continue

            valid_blocks.append((col, cy, cx, area))

    closest_block = None
    if valid_blocks:
        closest_block = max(valid_blocks, key=lambda b: b[1])
        color, cy, cx, area = closest_block
        cy_full = cy + roi_start
        x_percent = (cx / full_w) * 100
        y_percent = ((full_h - cy_full) / full_h) * 100
        cell_w = full_w // 5
        cell_h = full_h // 5
        col_idx = cx // cell_w
        row_idx = cy_full // cell_h
        cell_number = row_idx * 5 + col_idx + 1
    else:
        color, cy, cx, area, cell_number, x_percent, y_percent = (None, None, None, None, None, None, None)

    special_case = 0
    if DIRECTION == "anticlockwise" and detections["green"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        green_in_right = False
        green_in_left_front = False
        for (cx_g, cy_g, area_g, _) in detections["green"]:
            cy_full_g = cy_g + roi_start
            col_idx = cx_g // cell_w
            row_idx = cy_full_g // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            if cell_num in [5, 10, 15, 20, 25] and area_g > 30000:
                green_in_right = True
            if cell_num % 5 in [1, 2, 3, 4]:
                if ((max_blue_y and cy_g > max_blue_y) or (max_orange_y and cy_g > max_orange_y)):
                    if area_g > 7000:
                        green_in_left_front = True
        if green_in_right and green_in_left_front:
            special_case = 1
    elif DIRECTION == "clockwise" and detections["red"]:
        cell_w = full_w // 5
        cell_h = full_h // 5
        red_in_left = False
        red_in_right_front = False
        for (cx_r, cy_r, area_r, _) in detections["red"]:
            cy_full_r = cy_r + roi_start
            col_idx = cx_r // cell_w
            row_idx = cy_full_r // cell_h
            cell_num = row_idx * 5 + col_idx + 1
            if cell_num in [1, 6, 11, 16, 21] and area_r > 20000:
                red_in_left = True
            if cell_num % 5 in [4, 0]:
                if ((max_blue_y and cy_r > max_blue_y) or (max_orange_y and cy_r > max_orange_y)):
                    if area_r > 7000:
                        red_in_right_front = True
        if red_in_left and red_in_right_front:
            special_case = 1

    if SHOW:
        cell_w = full_w // 5
        cell_h = full_h // 5
        display = frame.copy()
        for i in range(1, 5):
            cv2.line(display, (i * cell_w, 0), (i * cell_w, frame.shape[0]), (200, 200, 200), 1)
        for j in range(1, 5):
            y_line = j * cell_h - roi_start
            if 0 <= y_line < frame.shape[0]:
                cv2.line(display, (0, y_line), (frame.shape[1], y_line), (200, 200, 200), 1)
        num = 1
        for r in range(5):
            for c in range(5):
                y_pos = r * cell_h + 20 - roi_start
                if 0 <= y_pos < frame.shape[0]:
                    x_pos = c * cell_w + 10
                    cv2.putText(display, str(num), (x_pos, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                num += 1
        if closest_block:
            cv2.rectangle(display, (cx - 20, cy - 20), (cx + 20, cy + 20), (0, 255, 255), 3)
            cv2.putText(display, f"Closest: {color} Cell:{cell_number}",
                        (cx, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # cv2.imshow("Block Detection", display)
        # cv2.waitKey(1)

    return (color if closest_block else None,
            y_percent if closest_block else None,
            x_percent if closest_block else None,
            area if closest_block else None,
            cell_number if closest_block else None,
            special_case)

# Running Functions
def exit_parking_lot():
    global DIRECTION
    data = [None, None, None, None, None]
    while not data[2]:
        data = read_data()
        flush_serial()
    angle, left, front, right, ir = data
    if left > right:
        DIRECTION = "anticlockwise"
    else:
        DIRECTION = "clockwise"
    print(f"DIRECTION: {DIRECTION}")

    if DIRECTION == "clockwise":
        steer_until_angle(0, 10, PARKING_SPEED, 0, 50)
        steer_until_angle(10, 25, PARKING_SPEED, 1, -50)
        steer_until_angle(25, 45, PARKING_SPEED, 0, 50)

    elif DIRECTION == "anticlockwise":
        steer_until_angle(0, -10, PARKING_SPEED, 0, -50)
        steer_until_angle(-10, -25, PARKING_SPEED, 1, 50)
        steer_until_angle(-25, -45, PARKING_SPEED, 0, -50)

def first_block_sequence():
    global DIRECTION
    detected_blocks = []
    itr = 0
    Flag = True
    while Flag:
        itr += 1

        if itr >= 20:
            break

        detected_blocks = process_frame(get_frame(cap), masks, colors, SHOW=True, DIR=DIRECTION)

        Flag2 = True
        while len(detected_blocks) > 0 and Flag2:
            for object in detected_blocks:
                if object[0] == "blue":
                    detected_blocks.remove(object)
                    continue
                else:
                    Flag = False
                    Flag2 = False

    if detected_blocks:
        if detected_blocks[0][0] == "red" and DIRECTION == "anticlockwise":
            steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
            steer_until_angle(-30, -0, PARKING_SPEED, 0, 20)

        elif detected_blocks[0][0] == "green" and DIRECTION == "anticlockwise":
            steer_until_angle(-45, -90, PARKING_SPEED, 0, -35)
            send_data(PARKING_SPEED, 0, 0)
            distance = 100
            while distance >= 25:
                send_data(PARKING_SPEED, 0, 0)
                _, _, distance, _, _ = read_data()
                while not distance:
                    _, _, distance, _, _ = read_data()
            steer_until_angle(-90, 0, PARKING_SPEED, 0, 50)

        elif detected_blocks[0][0] == "green" and DIRECTION == "clockwise":
            steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
            steer_until_angle(30, 0, PARKING_SPEED, 0, -30)

        elif detected_blocks[0][0] == "red" and DIRECTION == "clockwise":
            steer_until_angle(45, 70, PARKING_SPEED, 0, 35)
            send_data(PARKING_SPEED, 0, 0)
            distance = 100
            while distance >= 20:
                send_data(PARKING_SPEED, 0, 0)
                _, _, distance, _, _ = read_data()
                while not distance:
                    _, _, distance, _, _ = read_data()
            steer_until_angle(90, 0, PARKING_SPEED, 0, -50)

    elif DIRECTION == "anticlockwise":
        steer_until_angle(-45, -30, PARKING_SPEED, 0, 10)
        steer_until_angle(-30, -0, PARKING_SPEED, 0, 20)

    elif DIRECTION == "clockwise":
        steer_until_angle(45, 30, PARKING_SPEED, 0, -20)
        steer_until_angle(30, 0, PARKING_SPEED, 0, -30)

def main_logic():
    global DIRECTION, KP, SPEED, COUNTER
    global last_turn, target_angle, last_block_pass, last_cooldown
    global ir, left, right, front
    global InnerStuckFailsafeFlag, OuterStuckFailsafeFlag
    left = 1000
    right = 1000
    ir = 0

    if DIRECTION == "anticlockwise":
        angle = 0
        while COUNTER < COUNTER_MAX:
            ret, frame = cap.read()
            if not ret:
                continue

            block = detect_biggest_block(frame, SHOW=True)

            data = read_data()
            print(data)
            last_left = left
            if data and data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # print(angle_diff(target_angle, angle))
            if (angle_diff(target_angle, angle) > 50 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) > 60) or (angle_diff(target_angle, angle) > 50 and time.time() - last_turn > 10):
                print("GOING WRONG DIRECTION FAILSAFE")
                send_data(SPEED, 0, 40)
                time.sleep(0.5)
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                continue

            # print(block)

            if not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2: ### FAILSAFE --> It doesn't speed up after a turn for 5s
                SPEED = SPEED
            else:
                SPEED = SPEED_NO_AURA_FARM

            # REVERSE PRECEDENCE
            if block[1] and block[1] < 20 and block[2] > 40 and block[2] < 60:
                send_data(0, 0, 0)
                time.sleep(0.2)
                send_data(80, 1, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                ### IR REVERSE FAILSAFE
                print("IR REVERSE FAILSAFE")
                send_data(80, 1, 0)
                time.sleep(1)
                if COUNTER % 4 == 0:
                    time.sleep(1)
                    steer_until_angle(0, -30, 100, 0, -50)
                    steer_until_angle(-30, 0, 100, 0, 50)
                send_data(0, 0, 0)
                flush_serial()
                ir = 1
                while ir:
                    data = None
                    while not data[2]:
                        data = read_data()
                    angle, left, front, right, ir = data
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()

            ### INNER STUCK FAILSAFE
            if left < 5 and right > 60:
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif InnerStuckFailsafeFlag and time.time() - last_inner_stuck > 5:
                    send_data(80, 1, -10)
                    time.sleep(1)
                    send_data(80, 0, 10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    InnerStuckFailsafeFlag = False
                    continue

            ### OUTER STUCK FAILSAFE
            if right < 5 and left > 60:
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif OuterStuckFailsafeFlag and time.time() - last_outer_stuck > 5:
                    send_data(80, 1, 10)
                    time.sleep(1)
                    send_data(80, 0, -10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    OuterStuckFailsafeFlag = False
                    continue

            # BLOCK AVOID PRECEDENCE

            # Matrix
            ### DOUBLE TROUBLE FAILSAFE
            if block[5]:
                print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                send_data(80, 0, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()

            if block[0] and block[3] > 4000:
                # if block[4] in [] ### AFTERTURN EXTREME FAILSAFE
                color, y, x, area, cell, dt_failsafe_flag = block

                if color == "red" or color == "green":
                    last_block_pass = time.time()
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        send_data(SPEED, 0, int(red_turning_values[cell-1]))
                        if COUNTER % 4 == 0:
                            send_data(SPEED, 0, int(red_turning_values_4th_turn_anti[cell-1]))
                        elif area < 3000:
                            send_data(SPEED, 0, red_turning_values_less_than_3000[cell - 1])
                        continue
                    elif red_turning_values[cell-1] == -1:
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    else:
                        error = angle_diff(target_angle, angle)
                        send_data(SPEED, 0, error)
                elif color == "green":
                    if green_turning_values[cell-1] != -1: ### DOUBLE TROUBLE KINDA FAILSAFE
                        if abs(angle_diff(target_angle, angle)) < 5 and left > 100 and time.time() - last_turn > 5:
                            print("DOUBLE TROUBLE KINDA FAILSWAFE")
                            send_data(80, 0, 0)
                            time.sleep(1)
                            send_data(0, 0, 0)
                            continue
                        send_data(SPEED, 0, int(green_turning_values[cell-1] * BLOCK_MULITPLIER_GREEN_ANTI))
                        continue
                    elif green_turning_values[cell-1] == -1:
                        send_data(0, 0, 0)
                        data = wait_until_and_read_data()
                        angle, left, front, right, ir = data
                        if abs(angle_diff(target_angle, angle)) < 5:
                            if left > 100:
                                send_data(80, 0, 0)
                                time.sleep(1)
                                send_data(0, 0, 0)
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    # else:
                    #     error = angle_diff(target_angle, angle)
                    #     send_data(SPEED, 0, error)
            # Other
            # if block[0]:
            #     if area > 4000 and y < 85:
            #         if color == "red":
            #             send_data(SPEED, 0, 25)
            #             continue
            #         elif color == "green":
            #             send_data(SPEED, 0, -25)
            #             continue

            # TURN PRECEDENCE

            data = read_data()
            last_left = left
            if data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # PARKING PRECEDENCE
            if data[2] == 0 and COUNTER == COUNTER_MAX - 1 and time.time() - last_turn > 7.5 and left > 100 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return

            # print(left, front)
            # if COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0]:
            #     print("PARKING")
            #     parking()
            #     exit()

            if front < 15 and block[0] == None and (left > 100 or ((COUNTER + 1) % 4 == 0 and left > 50)) and COUNTER != COUNTER_MAX - 1:
                ### FALSE TURN FAILSAFE
                data = None
                while not data:
                    data = read_data()
                    if data[2]:
                        angle, left, front, right, ir = data
                    else:
                        continue
                if abs(angle_diff(normalize_angle(target_angle+5), angle)) > 20:
                    send_data(80, 1, 0)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    flush_serial()
                    continue
                target_angle = normalize_angle(target_angle - 90)
                COUNTER += 1
                print(f"COUNTER = {COUNTER}, TARGET={target_angle}")
                if right >= 25:
                    while True:
                        send_data(TURNING_SPEED, 1, 40)

                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle+10), angle)
                        if err >= -10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                elif right <= 25:
                    send_data(80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    send_data(0, 0, 0)
                    while True:
                        send_data(TURNING_SPEED, 0, -40)
                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle+10), angle)

                        if err >= -10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                flush_serial()
            # elif front < 15 and COUNTER % 4 == 0:
            #     print("PARKING LOT COLLISION FAILSAFE")
            #     send_data(80, 1, 0)
            #     time.sleep(1)
            #     send_data(0, 0, 0)
            #     steer_until_angle(0, -10, 80, 0, -50)
            #     steer_until_angle(-10, -25, 80, 1, 50)
            #     steer_until_angle(-25, -45, 80, 0, -50)


            # PID PRECEDENCE
            if COUNTER % 4 == 0:
                error = angle_diff(normalize_angle(target_angle - 4), angle)
            else:
                error = angle_diff(target_angle, angle)
            send_data(SPEED, 0, error)
            if COUNTER == COUNTER_MAX - 1:
                print("COUNTER MAX - 1")
            if COUNTER == COUNTER_MAX - 1 and time.time() - last_turn > 7.5 and left > 100 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return
    if DIRECTION == "clockwise":
        angle = 0
        while COUNTER < COUNTER_MAX:
            ret, frame = cap.read()
            if not ret:
                continue

            block = detect_biggest_block(frame, SHOW=True)

            data = read_data()
            print(data)
            last_left = left
            if data and data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # print(angle_diff(target_angle, angle))
            if (angle_diff(target_angle, angle) < -50 and time.time() - last_turn < 5) or (angle_diff(target_angle, angle) < -60) or (angle_diff(target_angle, angle) < -50 and time.time() - last_turn > 10):
                print("GOING WRONG DIRECTION FAILSAFE")
                send_data(SPEED, 0, -40)
                time.sleep(0.5)
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                continue

            # print(block)

            if not block[0] and time.time() - last_turn > 5 and time.time() - last_block_pass > 2: ### FAILSAFE --> It doesn't speed up after a turn for 5s
                SPEED = SPEED
            else:
                SPEED = SPEED_NO_AURA_FARM

            # REVERSE PRECEDENCE
            if block[1] and block[1] < 20 and block[2] > 40 and block[2] < 60:
                send_data(0, 0, 0)
                time.sleep(0.2)
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                ### IR REVERSE FAILSAFE
                print("IR REVERSE FAILSAFE")
                send_data(80, 1, 0)
                time.sleep(1)
                if COUNTER % 4 == 0:
                    time.sleep(1)
                    steer_until_angle(0, -30, 100, 0, -50)
                    steer_until_angle(-30, 0, 100, 0, 50)
                send_data(0, 0, 0)
                flush_serial()
                ir = 1
                while ir:
                    data = None
                    while not data[2]:
                        data = read_data()
                    angle, left, front, right, ir = data
                ret = None
                while not ret:
                    ret, frame = cap.read()
                continue

            if ir == 1:
                send_data(80, 1, 0)
                time.sleep(1)
                send_data(0, 0, 0)
                flush_serial()

            ### LEFT STUCK FAILSAFE
            if left < 5 and right > 60:
                if not InnerStuckFailsafeFlag:
                    InnerStuckFailsafeFlag = True
                    last_inner_stuck = time.time()
                elif InnerStuckFailsafeFlag and time.time() - last_inner_stuck > 5:
                    send_data(80, 1, -10)
                    time.sleep(1)
                    send_data(80, 0, 10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    InnerStuckFailsafeFlag = False
                    continue

            ### RIGHT STUCK FAILSAFE
            if right < 5 and left > 60:
                if not OuterStuckFailsafeFlag:
                    OuterStuckFailsafeFlag = True
                    last_outer_stuck = time.time()
                elif OuterStuckFailsafeFlag and time.time() - last_outer_stuck > 5:
                    send_data(80, 1, 10)
                    time.sleep(1)
                    send_data(80, 0, -10)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    data = wait_until_and_read_data()
                    OuterStuckFailsafeFlag = False
                    continue

            # PARKING PRECEDENCE FRFR

            if COUNTER == COUNTER_MAX - 1 and right < 80 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                angle, left, front, right, ir = data
                if right > 80:
                    continue
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                print("MAIN LOOP COMPLETE -- 24")
                send_data(0, 0, 0)
                return

            # BLOCK AVOID PRECEDENCE

            # Matrix
            ### DOUBLE TROUBLE FAILSAFE
            if block[5]:
                print("DOUBLE TROUBLE FAILSAFE ACTIVATED")
                send_data(80, 0, 0)
                time.sleep(2)
                send_data(0, 0, 0)
                flush_serial()

            if block[0] and block[3] > 4000:
                # if block[4] in [] ### AFTERTURN EXTREME FAILSAFE
                color, y, x, area, cell, dt_failsafe_flag = block

                if color == "red" or color == "green":
                    last_block_pass = time.time()
                if color == "red":
                    if red_turning_values[cell-1] != -1:
                        if abs(angle_diff(target_angle, angle)) < 5 and right > 100 and time.time() - last_turn > 5: ### DOUBLE TROUBLE KINDA FAILSAFE
                            print("DOUBLE TROUBLE KINDA FAILSWAFE")
                            send_data(80, 0, 0)
                            time.sleep(1)
                            send_data(0, 0, 0)
                            continue
                        send_data(SPEED, 0, int(red_turning_values[cell-1]))
                        continue
                    elif red_turning_values[cell-1] == -1:
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    else:
                        error = angle_diff(target_angle, angle)
                        send_data(SPEED, 0, error)
                elif color == "green":
                    if green_turning_values[cell-1] != -1: ### DOUBLE TROUBLE KINDA FAILSAFE
                        send_data(SPEED, 0, int(green_turning_values[cell-1] * BLOCK_MULITPLIER_GREEN_ANTI))
                        if COUNTER % 4 == 0:
                            send_data(SPEED, 0, int(green_turning_values_4th_turn_clock[cell-1])) ##### PENDING: 
                        continue
                    elif green_turning_values[cell-1] == -1:
                        send_data(0, 0, 0)
                        data = wait_until_and_read_data()
                        angle, left, front, right, ir = data
                        send_data(80, 1, 0)
                        time.sleep(1)
                        send_data(0, 0, 0)
                        flush_serial()
                        continue
                    # else:
                    #     error = angle_diff(target_angle, angle)
                    #     send_data(SPEED, 0, error)
            # Other
            # if block[0]:
            #     if area > 4000 and y < 85:
            #         if color == "red":
            #             send_data(SPEED, 0, 25)
            #             continue
            #         elif color == "green":
            #             send_data(SPEED, 0, -25)
            #             continue

            # TURN PRECEDENCE

            data = read_data()
            last_left = left
            if data[2]:
                angle, left, front, right, ir = data
                # print(data)

            # PARKING PRECEDENCE
            if data[2] == 0 and COUNTER == COUNTER_MAX - 1 and right < 80 and abs(angle_diff(0, angle)) < 15:
                send_data(0, 0, 0)
                data = wait_until_and_read_data()
                if abs(angle_diff(0, data[0])) > 10:
                    continue
                if data[1] < 100:
                    continue
                send_data(80, 0, 0)
                time.sleep(0.5)
                send_data(0, 0, 0)
                if right > 80:
                    continue
                print("MAIN LOOP COMPLETE")
                send_data(0, 0, 0)
                return

            # print(left, front)
            # if COUNTER == COUNTER_MAX - 1 and front < 15 and not block[0]:
            #     print("PARKING")
            #     parking()
            #     exit()

            if front < 15 and block[0] == None and (right > 100 or ((COUNTER + 1) % 4 == 0)) and COUNTER != COUNTER_MAX - 1:
                ### FALSE TURN FAILSAFE
                data = None
                while not data:
                    data = read_data()
                    if data[2]:
                        angle, left, front, right, ir = data
                    else:
                        continue
                if abs(angle_diff(normalize_angle(target_angle-5), angle)) > 20:
                    send_data(80, 1, 0)
                    time.sleep(1)
                    send_data(0, 0, 0)
                    flush_serial()
                    continue
                target_angle = normalize_angle(target_angle + 90)
                COUNTER += 1
                print(f"COUNTER = {COUNTER}, TARGET={target_angle}")
                if left >= 25:
                    while True:
                        send_data(TURNING_SPEED, 1, -40)

                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle-10), angle)
                        if err <= 10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                elif left <= 25:
                    send_data(80, 1, 0)
                    time.sleep(BACK_BEFORE_TURN_TIME)
                    send_data(0, 0, 0)
                    while True:
                        send_data(TURNING_SPEED, 0, 40)
                        data = read_data()
                        if not data[2]:
                            continue
                        angle, left, front, right, ir = data

                        err = angle_diff(normalize_angle(target_angle-10), angle)

                        if err <= 10:
                            send_data(80, 1, 0)
                            time.sleep(BACK_AFTER_TURN_TIME)
                            send_data(0, 0, 0)
                            last_turn = time.time()
                            last_cooldown = time.time()
                            break
                flush_serial()
            # elif front < 15 and COUNTER % 4 == 0:
            #     print("PARKING LOT COLLISION FAILSAFE")
            #     send_data(80, 1, 0)
            #     time.sleep(1)
            #     send_data(0, 0, 0)
            #     steer_until_angle(0, -10, 80, 0, -50)
            #     steer_until_angle(-10, -25, 80, 1, 50)
            #     steer_until_angle(-25, -45, 80, 0, -50)


            # PID PRECEDENCE
            if COUNTER % 4 == 0:
                error = angle_diff(normalize_angle(target_angle + 4), angle)
            else:
                error = angle_diff(target_angle, angle)
            send_data(SPEED, 0, error)
            if COUNTER == COUNTER_MAX - 1:
                print("COUNTER MAX - 1")

def parking(DEBUG = False):
    global DIRECTION
    send_data(0, 0, 0)
    if DIRECTION == "anticlockwise":
        data = wait_until_and_read_data()
        angle, left, front, right, ir = data
        if front < 25:
            send_data(75, 1, 0)
            time.sleep(1.5)
            send_data(0, 0, 0)
        elif front > 70:
            send_data(75, 0, 0)
            time.sleep(1)
            send_data(0, 0, 0)
        send_data(0, 0, 0)
        print("SET POSITION; STARTING TURN IN 1S")
        time.sleep(1)
        data = wait_until_and_read_data()
        data = wait_until_and_read_data()
        data = wait_until_and_read_data()
        angle, left, front, right, ir = data
        send_data(50, 1, 0)
        while left > 80 or (front and front < 80):
            data = read_data()
            if data:
                angle, left, front, right, ir = data
                send_data(50, 1, angle)
                print(data)
            flush_serial()
        send_data(0, 0, 0)
        steer_until_angle(0, -80, 75, 1, 55)
        send_data(60, 1, 0)
        time.sleep(3)
        send_data(0, 0, 0)
    elif DIRECTION == "clockwise":
        steer_until_angle(0, -90, 80, 0, -30)
        send_data(80, 0, 0)
        time.sleep(2)
        send_data(0, 0, 0)

def waitForOk():
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print("ESP32:", line)
            if line.upper() == "OK":
                print("Sending back OK...")
                ser.write(b"OK\n")
                break

if __name__ == "__main__":
    # MAIN
    try:
        print("Initialized")
        led.on()
        waitForOk()
        print("STARTING")
        flush_serial()
        exit_parking_lot()
        print(f"LIDAR PORT = {LIDAR_PORT}")
        first_block_sequence()
        main_logic()
        parking()
    except KeyboardInterrupt:
        send_data(0, 0, 0)
        exit()

    # DEBUG
    # while True:
    #     print(wait_until_and_read_data())
    #     flush_serial()