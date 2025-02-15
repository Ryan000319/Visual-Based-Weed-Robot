import sys
import threading
import cv2
import torch
import numpy as np
import time
import platform
import pathlib
import busio
from adafruit_servokit import ServoKit
from adafruit_blinka.microcontroller.bcm283x.pin import Pin
import json
import os
import logging
import argparse
from queue import Queue
from gpiozero import Motor, PWMOutputDevice, OutputDevice
from pynput import keyboard

# -------------------- Configuration and Initialization --------------------
from logging.handlers import RotatingFileHandler


# Initialize the pump GPIO pin 
PUMP_PIN = 16
pump = OutputDevice(PUMP_PIN)

# Initialize I2C and the servo kit
i2c = busio.I2C(scl=Pin(3), sda=Pin(2))
kit = ServoKit(channels=16, i2c=i2c)

# Servo channels
SPRAY_SERVO_CHANNEL = 0    
MIDDLE_SERVO_CHANNEL = 1    
BASE_SERVO_CHANNEL = 2      

spray_servo = kit.servo[SPRAY_SERVO_CHANNEL]
middle_servo = kit.servo[MIDDLE_SERVO_CHANNEL]
base_servo = kit.servo[BASE_SERVO_CHANNEL]

# -------------------- GPIOZero Motor Control --------------------

left_back_motor = Motor(forward=17, backward=18)
left_back_speed = PWMOutputDevice(13)

right_back_motor = Motor(forward=27, backward=22)
right_back_speed = PWMOutputDevice(19)

right_front_motor = Motor(forward=25, backward=9)
right_front_speed = PWMOutputDevice(10)

left_front_motor = Motor(forward=6, backward=5)
left_front_speed = PWMOutputDevice(12)

# Function to set motor speed 
def set_motor_speed(speed):
    speed_normalized = speed / 100  
    left_back_speed.value = speed_normalized
    right_back_speed.value = speed_normalized
    right_front_speed.value = speed_normalized
    left_front_speed.value = speed_normalized

# Movement functions
def move_forward(speed):
    set_motor_speed(speed)
    left_back_motor.forward()
    right_back_motor.forward()
    left_front_motor.forward()
    right_front_motor.forward()

def move_backward(speed):
    set_motor_speed(speed)
    left_back_motor.backward()
    right_back_motor.backward()
    left_front_motor.backward()
    right_front_motor.backward()

def stop_all_motors():
    left_back_motor.stop()
    right_back_motor.stop()
    left_front_motor.stop()
    right_front_motor.stop()
    set_motor_speed(0)

def move_forward_duration(speed, duration):
    """
    Moves the robot forward at the specified speed for the given duration (in seconds).
    """
    def move_thread():
        move_forward(speed)
        time.sleep(duration)
        stop_all_motors()
    threading.Thread(target=move_thread).start()

# -------------------- Grid and Coordinate Functions --------------------

NUM_ROWS = 11  
NUM_COLS = 11


GRID_COORDS = {
    # Row 0
    (1,1): {'B':129.64, 'M':152.48, 'S':180},
    (1,2): {'B':116.75, 'M':155.40, 'S':180},
    (1,3): {'B':108.56, 'M':154.82, 'S':180},
    (1,4): {'B':97.43,  'M':157.16, 'S':180},
    (1,5): {'B':85.13,  'M':152.48, 'S':180},
    (1,6): {'B':73.42,  'M':152.48, 'S':180},
    (1,7): {'B':58.78,  'M':150.72, 'S':180},

    # Row 1
    (2,1): {'B':134.90, 'M':165.95, 'S':180},
    (2,2): {'B':124.95, 'M':159.50, 'S':170.63},
    (2,3): {'B':110.90, 'M':159.50, 'S':164.77},
    (2,4): {'B':96.26,  'M':153.06, 'S':157.75},
    (2,5): {'B':82.79,  'M':153.65, 'S':148.96},
    (2,6): {'B':69.32,  'M':156.58, 'S':165.36},
    (2,7): {'B':54.10,  'M':161.85, 'S':176.49},

    # Row 2
    (3,1): {'B':137.84, 'M':167.12, 'S':180},
    (3,2): {'B':132.57, 'M':173.56, 'S':174.73},
    (3,3): {'B':115.58, 'M':158.92, 'S':147.21},
    (3,4): {'B':99.19, 'M':151.89, 'S':134.91},
    (3,5): {'B':85.72,  'M':147.21, 'S':123.20},
    (3,6): {'B':64.64,  'M':153.06, 'S':142.62},
    (3,7): {'B':52.92,  'M':158.92, 'S':151.31},

    # Row 3
    (4,1): {'B':146.62, 'M':145.45, 'S':125.54},
    (4,2): {'B':138.42, 'M':145.44, 'S':121.44},
    (4,3): {'B':120.85, 'M':139.59, 'S':101.53},
    (4,4): {'B':104.46, 'M':143.69, 'S':95.67},
    (4,5): {'B':84.55,  'M':143.69, 'S':95.67},
    (4,6): {'B':64.64,  'M':142.52, 'S':104.46},
    (4,7): {'B':45.90,  'M':155.99, 'S':131.39},

    # Row 4
    (5,1): {'B':160.68, 'M':147.79, 'S':121.44},
    (5,2): {'B':148.96, 'M':135.49, 'S':89.82},
    (5,3): {'B':130.00, 'M':131.98, 'S':73.42},
    (5,4): {'B':110.90, 'M':131.98, 'S':61.71},
    (5,5): {'B':82.79,  'M':131.98, 'S':61.71},
    (5,6): {'B':56.44,  'M':131.98, 'S':59.95},
    (5,7): {'B':36.52,  'M':131.98, 'S':71.66},

    # Row 5
    (6,1): {'B':170.63, 'M':135.49, 'S':95.67},
    (6,2): {'B':164.77, 'M':141.94, 'S':75.76},
    (6,3): {'B':148.96, 'M':127.88, 'S':53.51},
    (6,4): {'B':118.51, 'M':127.88, 'S':43.55},
    (6,5): {'B':84.55,  'M':127.88, 'S':33.60},
    (6,6): {'B':46.48,  'M':127.88, 'S':43.55},
    (6,7): {'B':22.47,  'M':127.88, 'S':53.51},
}


# Initialize locks for thread safety
servo_lock = threading.Lock()
data_lock = threading.Lock()

# Shared variables between threads
sprayed_grids = set()
detected_grids = set()
spray_durations = {} 

# -------------------- Servo Control Functions --------------------

def move_servos_smoothly(target_base, target_middle, target_spray, steps=40, delay=0.02):
    """
    Smoothly moves servos from their current positions to target angles.
    """
    def servo_thread():
        with servo_lock:
            current_base = base_servo.angle if base_servo.angle is not None else 0
            current_middle = middle_servo.angle if middle_servo.angle is not None else 0
            current_spray = spray_servo.angle if spray_servo.angle is not None else 0

            for step_count in range(steps + 1):
                t = step_count / steps

                new_base = linear_easing(t, current_base, target_base)
                new_middle = linear_easing(t, current_middle, target_middle)
                new_spray = linear_easing(t, current_spray, target_spray)

                base_servo.angle = max(0, min(180, new_base))
                middle_servo.angle = max(0, min(180, new_middle))
                spray_servo.angle = max(0, min(180, new_spray))

                time.sleep(delay)

            # Ensure final angles are set precisely
            base_servo.angle = max(0, min(180, target_base))
            middle_servo.angle = max(0, min(180, target_middle))
            spray_servo.angle = max(0, min(180, target_spray))
    threading.Thread(target=servo_thread).start()

def linear_easing(t, start, end):
    """
    Linear easing function for smooth servo movement.
    """
    return (1 - t) * start + t * end

def activate_pump(duration):
    """
    Activates the pump for a specified duration in seconds.
    """
    def pump_thread():
        with servo_lock:
            logging.info("Activating pump...")
            print("Activating pump...")
            pump.on()
            time.sleep(duration)
            pump.off()
            logging.info("Pump deactivated.")
            print("Pump deactivated.")
    threading.Thread(target=pump_thread).start()
    

def reset_arm_position():
    """
    Resets the robotic arm to the default or initial position.
    """
    logging.info("Resetting arm to initial position...")
    print("Resetting arm to initial position...")
    DEFAULT_BASE = 0    
    DEFAULT_MIDDLE = 25  
    DEFAULT_SPRAY = 0    
    move_servos_smoothly(DEFAULT_BASE, DEFAULT_MIDDLE, DEFAULT_SPRAY, steps=30, delay=0.01)
    logging.info("Arm reset complete.")
    print("Arm reset complete.")

def get_current_servo_angles():
    """
    Retrieves the current servo angles.
    """
    with servo_lock:
        current_base = base_servo.angle if base_servo.angle is not None else 0
        current_middle = middle_servo.angle if middle_servo.angle is not None else 0
        current_spray = spray_servo.angle if spray_servo.angle is not None else 0
    return {'Base': current_base, 'Middle': current_middle, 'Spray': current_spray}

# -------------------- Grid and Coordinate Functions --------------------

def get_grid_cell(x_pixel, y_pixel, frame_width, frame_height, num_rows, num_cols):
    """
    Determines the grid cell (row, col) based on pixel coordinates.
    """
    cell_width = frame_width / num_cols  
    cell_height = frame_height / num_rows

    col = int(x_pixel // cell_width)  
    row = int(y_pixel // cell_height)

    # Clamp values to grid boundaries
    col = min(max(col, 0), num_cols - 1)  
    row = min(max(row, 0), num_rows - 1)

    return (row, col)

def get_servo_angles(grid_row, grid_col, grid_coords):
    """
    Retrieves the servo angles (B, M, S) based on grid cell.
    """
    return grid_coords.get((grid_row, grid_col), None)

# Function to draw grid on the frame and label coordinates
def draw_grid_with_coordinates(frame, num_rows, num_cols, sprayed_grids=set(), detected_grids=set()):
    """
    Draws a grid on the frame and labels each cell with its (row, col) coordinates.
    Sprayed grids are highlighted in red, and detected grids are highlighted in green.
    Returns the width and height of each grid cell.
    """
    height, width, _ = frame.shape
    cell_width = width / num_cols  
    cell_height = height / num_rows

    for row in range(num_rows):
        for col in range(num_cols): 
            x1, y1 = int(col * cell_width), int(row * cell_height)
            x2, y2 = int((col + 1) * cell_width), int((row + 1) * cell_height)

            # Determine grid color based on status
            if (row, col) in sprayed_grids:
                grid_color = (0, 0, 255)  # Red for sprayed grids
            elif (row, col) in detected_grids:
                grid_color = (0, 255, 0)  # Green for detected grids
            else:
                grid_color = (255, 255, 255)  # White for unsprayed and undetected grids

            # Draw rectangle for grid cell
            cv2.rectangle(frame, (x1, y1), (x2, y2), grid_color, 1)

            # Label the cell with (row, col)
            coord_text = f"({row}, {col})"
            cv2.putText(frame, coord_text, (x1 + 5, y1 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, grid_color, 1)

    return cell_width, cell_height  # Return cell dimensions

# -------------------- VideoStream Class --------------------

class VideoStream:
    def __init__(self, src, queue_size=10):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            logging.error(f"Error: Could not open video source {src}")
            print(f"Error: Could not open video source {src}")
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()
        self.queue = Queue(maxsize=queue_size)

    def start(self):
        if self.started:
            logging.warning("[INFO] Asynchronous video capturing has already started.")
            print("[INFO] Asynchronous video capturing has already started.")
            return self
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            if not grabbed:
                logging.warning("Failed to grab frame from video source.")
                print("Failed to grab frame from video source.")
                time.sleep(0.1) 
                continue
            with self.read_lock:
                self.frame = frame

    def read(self):
        with self.read_lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.started = False
        self.thread.join()
        self.cap.release()

    def __exit__(self, exc_type, exc_value, traceback):
        self.cap.release()

# -------------------- YOLOv5 Detection Functions --------------------

def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    """
    Scales coordinates from img1_shape to img0_shape.
    """
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0] 
    coords[:, [1, 3]] -= pad[1]  
    coords[:, :4] /= gain
    coords[:, 0].clamp_(0, img0_shape[1])  
    coords[:, 1].clamp_(0, img0_shape[0])  
    coords[:, 2].clamp_(0, img0_shape[1]) 
    coords[:, 3].clamp_(0, img0_shape[0]) 
    return coords

# -------------------- Detection and Spraying Functions --------------------

if platform.system() == 'Linux':
    # Check if the system is a Raspberry Pi
    try:
        with open('/proc/device-tree/model') as f:
            model_str = f.read()
        if 'Raspberry Pi' in model_str:
            pathlib.WindowsPath = pathlib.PosixPath
    except FileNotFoundError:
        pass


YOLOV5_PATH = '/home/pi/yolov5'  
sys.path.append(YOLOV5_PATH)

try:
    from models.common import DetectMultiBackend
    from utils.general import check_img_size, non_max_suppression, check_imshow
    from utils.torch_utils import select_device
except ImportError as e:
    logging.error(f"YOLOv5 modules not found: {e}")
    print(f"Error: YOLOv5 modules not found. Ensure YOLOv5 is correctly installed at '{YOLOV5_PATH}'.")
    sys.exit(1)

# Define detection window duration and cycle delay 
DETECTION_WINDOW = 4
CYCLE_DELAY = 1      

def detect_and_spray(stream):
    """
    Detects weeds using YOLOv5 and controls the robotic arm to spray each detected weed sequentially.
    For each detected weed, selects the grid cell with the most green pixels within the bounding box.
    After spraying all detected weeds in the current batch, moves the robot forward and resets for the next cycle.
    """
    logging.info("Initializing YOLOv5 model...")
    print("Initializing YOLOv5 model...")

    # Load YOLOv5 model
    device = select_device('')
    MODEL_PATH = '/home/pi/weed_detection_models/exp11/weights/best.pt' 
    try:
        model = DetectMultiBackend(MODEL_PATH, device=device, dnn=False)
    except Exception as e:
        print(f"Error loading model: {e}")
        logging.error(f"Error loading model: {e}")
        return
    print("Model loaded successfully.")

    # Print model classes to identify class indices
    print("Model Classes:", model.names)
    logging.info(f"Model Classes: {model.names}")

    # Set model configuration parameters
    model.conf = 0.5  # Confidence threshold for detections
    model.iou = 0.5    # Intersection over Union (IoU) threshold for Non-Max Suppression (NMS)
    model.multi_label = False  
    model.max_det = 100  

    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(704, s=stride)  # Image size for model input
    print(f"Image size set to: {imgsz}")

    frame_skip = 2  # Process every 2nd frame to reduce computational load
    frame_count = 0

    while True:
        print("Moving forward for 1 second.")
        logging.info("Moving forward for 1 second.")
        # Move robot forward at speed 25 for 1 second
        move_forward_duration(25, duration=1)  

        # Wait for the robot to stop moving before starting the next detection cycle
        time.sleep(3)

        # Reset detection state for the new cycle
        detected_weeds = []  # List to store detected weeds and their spray durations
        with data_lock:
            # Clear previously detected and sprayed grid cells
            detected_grids.clear()  
            sprayed_grids.clear()  

        # Start time for the detection window
        window_start_time = time.time()
        print(f"Starting detection window of {DETECTION_WINDOW} seconds...")
        logging.info(f"Starting detection window of {DETECTION_WINDOW} seconds...")

        # Detection window loop
        while (time.time() - window_start_time) < DETECTION_WINDOW:
            frame = stream.read()  # Read the latest frame from the video stream
            if frame is None:
                logging.warning("No frame captured.")
                print("No frame captured.")
                time.sleep(0.01)  
                continue

            frame_count += 1
            if frame_count % frame_skip != 0:
                continue  

            # Preprocess frame for model input
            img = cv2.resize(frame, (imgsz, imgsz)) 
            img = img.transpose((2, 0, 1))[::-1]    
            img = np.ascontiguousarray(img)         
            im = torch.from_numpy(img).to(device)   
            im = im.half() if model.fp16 else im.float() 
            im /= 255.0  

            if len(im.shape) == 3:
                im = im[None]  

            # Perform inference using the YOLOv5 model
            try:
                with torch.no_grad():
                    pred = model(im, augment=False, visualize=False)
            except Exception as e:
                logging.error(f"Error during inference: {e}")
                print(f"Error during inference: {e}")
                continue

            # Apply Non-Max Suppression (NMS) to filter overlapping detections
            try:
                pred = non_max_suppression(pred, model.conf, model.iou, None, model.multi_label, max_det=model.max_det)
            except Exception as e:
                logging.error(f"Error during NMS: {e}")
                print(f"Error during NMS: {e}")
                continue

            # Get image dimensions for coordinate scaling
            img_height, img_width = frame.shape[:2]

            # Iterate through each detection in the predictions
            for det in pred:
                if len(det):
                    try:
                        # Scale bounding box coordinates back to original frame size
                        det[:, :4] = scale_coords(im.shape[2:], det[:, :4], frame.shape).round()
                    except Exception as e:
                        logging.error(f"Error scaling coordinates: {e}")
                        print(f"Error scaling coordinates: {e}")
                        continue

                    # Process each detection in reverse order (optional)
                    for detection in reversed(det):
                        try:
                            *xyxy, conf, cls = detection.tolist()  
                        except ValueError as e:
                            logging.error(f"Unpacking error for detection {detection}: {e}")
                            print(f"Unpacking error for detection {detection}: {e}")
                            continue

                        class_idx = int(cls)
                        class_name = model.names[class_idx]

                        # Filter detections to include only weeds
                        if class_name.lower() not in ('weed', 'weeds'):
                            continue

                        # Ensure bounding box has all four coordinates
                        if len(xyxy) != 4:
                            logging.warning(f"Incomplete bounding box coordinates: {xyxy}")
                            print(f"Incomplete bounding box coordinates: {xyxy}")
                            continue

                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = map(int, xyxy)
                        label = f"{class_name} {conf:.2f}"

                        # Calculate bounding box area to determine spray duration
                        bounding_box_area = (x2 - x1) * (y2 - y1)
                        # Threshold for small weeds
                        if bounding_box_area < 8000:  
                            spray_duration = 1.5
                        # Threshold for medium weeds
                        elif bounding_box_area < 14000: 
                            spray_duration = 3
                        # Threshold for large weeds
                        else:  
                            spray_duration = 5

                        # Print and log detection details
                        print(f"Weed detected with bounding box area: {bounding_box_area}px, Spray duration: {spray_duration}s")
                        logging.info(f"Weed detected with bounding box area: {bounding_box_area}px, Spray duration: {spray_duration}s")

                        # Create a unique identifier for the detected weed based on its bounding box
                        weed_id = (x1, y1, x2, y2)

                        # Check if this weed has already been processed in this cycle to avoid duplicates
                        is_duplicate = False
                        for existing_weed, _ in detected_weeds:
                            iou = calculate_iou(weed_id, existing_weed)
                            if iou > 0.5:
                                is_duplicate = True
                                break

                        if is_duplicate:
                             # Skip duplicate weed detections
                            continue 

                        # Add the weed's bounding box and spray duration to the detected_weeds list
                        detected_weeds.append((weed_id, spray_duration))

                        # Initialize variables to track the grid cell with the most green pixels
                        max_green_pixels = 0
                        selected_grid = None

                        # Iterate through all defined grid cells to find the one with the most green pixels within the bounding box
                        for (row, col), angles in GRID_COORDS.items():
                            cell_width_pix = frame.shape[1] / NUM_COLS  
                            cell_height_pix = frame.shape[0] / NUM_ROWS  

                            # Calculate pixel boundaries of the current grid cell
                            grid_x1 = int(col * cell_width_pix)
                            grid_y1 = int(row * cell_height_pix)
                            grid_x2 = int((col + 1) * cell_width_pix)
                            grid_y2 = int((row + 1) * cell_height_pix)

                            # Determine the intersection coordinates between the weed's bounding box and the grid cell
                            inter_x1 = max(x1, grid_x1)
                            inter_y1 = max(y1, grid_y1)
                            inter_x2 = min(x2, grid_x2)
                            inter_y2 = min(y2, grid_y2)

                            # Check if there is an actual overlap between the bounding box and the grid cell
                            if inter_x1 >= inter_x2 or inter_y1 >= inter_y2:
                                continue  

                            # Extract the intersecting region from the frame
                            intersect_region = frame[inter_y1:inter_y2, inter_x1:inter_x2]
                            # Count the number of green pixels in the intersecting region
                            green_pixels = np.sum(intersect_region[:, :, 1] > 128)  

                            # Update the selected grid cell if this cell has more green pixels than previous ones
                            if green_pixels > max_green_pixels:
                                max_green_pixels = green_pixels
                                selected_grid = (row, col)

                        # After iterating through all grid cells, check if a suitable grid cell was found
                        if selected_grid:
                            with data_lock:
                                # Add to detected grids for spraying
                                detected_grids.add(selected_grid)  
                                spray_durations[selected_grid] = spray_duration  
                            logging.info(f"Weed detected in grid cell {selected_grid} with {max_green_pixels} green pixels.")
                            print(f"Weed detected in grid cell {selected_grid} with {max_green_pixels} green pixels.")
                        else:
                            logging.info(f"Weed detected but no overlapping grid cell found in bounding box ({x1}, {y1}, {x2}, {y2}).")
                            print(f"Weed detected but no overlapping grid cell found in bounding box ({x1}, {y1}, {x2}, {y2}).")

        # After the detection window ends, report the number of unique grid cells detected
        print(f"Detection window ended. {len(detected_grids)} unique grid cells detected.")
        logging.info(f"Detection window ended. {len(detected_grids)} unique grid cells detected.")

        if detected_grids:
            # Iterate through each detected grid cell to perform spraying
            for target_grid in detected_grids:
                grid_row, grid_col = target_grid
                spray_duration = spray_durations.get(target_grid, 1.5) 
                logging.info(f"Grid cell {target_grid} detected.")
                print(f"Grid cell {target_grid} detected.")

                # Retrieve servo angles for the selected grid cell from GRID_COORDS
                servo_angles = get_servo_angles(grid_row, grid_col, GRID_COORDS)

                if servo_angles:
                    Servo_Base = servo_angles['B']
                    Servo_Middle = servo_angles['M']
                    Servo_Spray = servo_angles['S']

                    logging.info(f"Targeting grid cell {target_grid} with Servo Angles -> Base: {Servo_Base}°, Middle: {Servo_Middle}°, Spray: {Servo_Spray}°")
                    print(f"Targeting grid cell {target_grid} with Servo Angles -> Base: {Servo_Base}°, Middle: {Servo_Middle}°, Spray: {Servo_Spray}°")

                    # Move the robotic arm smoothly to the target servo angles
                    move_servos_smoothly(Servo_Base, Servo_Middle, Servo_Spray)
                    time.sleep(2)  

                    # Activate the pump to spray herbicide for the calculated duration
                    activate_pump(duration=spray_duration)
                    print(f"Spraying for {spray_duration} seconds")
                    time.sleep(spray_duration)  

                    # Mark the grid cell as sprayed to avoid re-spraying in the same cycle
                    with data_lock:
                        sprayed_grids.add(target_grid)

                else:
                    # Handle cases where servo angles for the grid cell are not defined
                    print(f"No servo angles found for grid cell ({grid_row}, {grid_col}).")
                    logging.warning(f"No servo angles found for grid cell ({grid_row}, {grid_col}).")

        else:
            # Inform if no weeds were detected in the current detection cycle
            print("No weeds detected in this cycle.")
            logging.info("No weeds detected in this cycle.")

        # After spraying all detected weeds, reset the robotic arm to its default position
        reset_arm_position()
        time.sleep(1)

        # Add a short delay before the next detection cycle begins
        time.sleep(CYCLE_DELAY)


def calculate_iou(weed_box, existing_weed_box):
    """Calculates IoU between two bounding boxes."""
    x1, y1, x2, y2 = weed_box
    ex1, ey1, ex2, ey2 = existing_weed_box

    inter_x1 = max(x1, ex1)
    inter_y1 = max(y1, ey1)
    inter_x2 = min(x2, ex2)
    inter_y2 = min(y2, ey2)

    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    weed_area = (x2 - x1) * (y2 - y1)
    existing_area = (ex2 - ex1) * (ey2 - ey1)

    iou = inter_area / float(weed_area + existing_area - inter_area)
    return iou



# -------------------- Video Display Thread --------------------

def video_display_thread(stream):
    """
    Continuously reads frames from the video stream and displays them.
    """
    while True:
        frame = stream.read()
        if frame is not None:
            frame_with_overlays = frame.copy()

            # Draw grid and detected/sprayed grids
            with data_lock:
                draw_grid_with_coordinates(
                    frame_with_overlays,
                    num_rows=NUM_ROWS,
                    num_cols=NUM_COLS,
                    sprayed_grids=sprayed_grids,
                    detected_grids=detected_grids
                )

            cv2.imshow('Weed Detection', frame_with_overlays)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.01)

    # When 'q' is pressed, ensure all resources are cleaned up
    stop_all_motors()
    stream.stop()
    cv2.destroyAllWindows()
    sys.exit(0)

# -------------------- Keyboard Control Functions --------------------

def on_press(key):
    """
    Handles key press events to adjust servo angles.
    """
    try:
        if key.char == 'w':  # Move middle servo up
            with servo_lock:
                current_middle = middle_servo.angle if middle_servo.angle is not None else 0
                target_middle = min(180, current_middle + 5)
            move_servos_smoothly(base_servo.angle, target_middle, spray_servo.angle)
            print_current_servo_angles()

        elif key.char == 's':  # Move middle servo down
            with servo_lock:
                current_middle = middle_servo.angle if middle_servo.angle is not None else 0
                target_middle = max(0, current_middle - 5)
            move_servos_smoothly(base_servo.angle, target_middle, spray_servo.angle)
            print_current_servo_angles()

        elif key.char == 'a':  # Move base servo left
            with servo_lock:
                current_base = base_servo.angle if base_servo.angle is not None else 0
                target_base = min(180, current_base + 5)
            move_servos_smoothly(target_base, middle_servo.angle, spray_servo.angle)
            print_current_servo_angles()

        elif key.char == 'd':  # Move base servo right
            with servo_lock:
                current_base = base_servo.angle if base_servo.angle is not None else 0
                target_base = max(0, current_base - 5)
            move_servos_smoothly(target_base, middle_servo.angle, spray_servo.angle)
            print_current_servo_angles()

        elif key.char == 'q':  # Move spray servo up
            with servo_lock:
                current_spray = spray_servo.angle if spray_servo.angle is not None else 0
                target_spray = min(180, current_spray + 5)
            move_servos_smoothly(base_servo.angle, middle_servo.angle, target_spray)
            print_current_servo_angles()

        elif key.char == 'e':  # Move spray servo down
            with servo_lock:
                current_spray = spray_servo.angle if spray_servo.angle is not None else 0
                target_spray = max(0, current_spray - 5)
            move_servos_smoothly(base_servo.angle, middle_servo.angle, target_spray)
            print_current_servo_angles()

    except AttributeError:
        pass  # Handle special keys that don't have 'char' attribute

def on_release(key):
    """
    Handles key release events.
    """
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def start_keyboard_listener():
    """
    Starts the keyboard listener in a separate thread.
    """
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    return listener

def print_current_servo_angles():
    """
    Prints the current servo angles.
    """
    angles = get_current_servo_angles()
    print(f"Current Servo Angles -> Base: {angles['Base']:.2f}°, Middle: {angles['Middle']:.2f}°, Spray: {angles['Spray']:.2f}°")
    logging.info(f"Current Servo Angles -> Base: {angles['Base']:.2f}°, Middle: {angles['Middle']:.2f}°, Spray: {angles['Spray']:.2f}°")

# -------------------- Main Functionality --------------------

def main():
    """
    Main function to handle command-line arguments and execute corresponding actions.
    """
    parser = argparse.ArgumentParser(description="3DOF Robotic Arm Grid Mapping Control")
    parser.add_argument('--detect', action='store_true', help='Run detection and spraying')
    args = parser.parse_args()

    if args.detect:
        # Start keyboard listener
        keyboard_listener = start_keyboard_listener()
        print("Keyboard controls enabled. Use W/S/A/D/Q/E to adjust servos. Press ESC to exit keyboard control.")
        logging.info("Keyboard controls enabled.")

        # Initialize video stream
        print("Starting video stream...")
        logging.info("Starting video stream...")
        stream_url = 'tcp://127.0.0.1:8888' 
        stream = VideoStream(stream_url).start()
        time.sleep(2.0)
        print("Video stream started.")
        logging.info("Video stream started.")

        # Start video display thread
        video_thread = threading.Thread(target=video_display_thread, args=(stream,))
        video_thread.start()

        # Run detection and spraying
        reset_arm_position()
        try:
            detect_and_spray(stream)
        except KeyboardInterrupt:
            print("\nDetection and spraying interrupted by user.")
            logging.info("Detection and spraying interrupted by user.")
            stop_all_motors()
            stream.stop()
            cv2.destroyAllWindows()
            sys.exit(0)

        # Wait for video thread to finish
        video_thread.join()

    else:
        # Interactive mode
        print("""
3DOF Robotic Arm Grid Mapping Control
--------------------------------------------
Instructions:
 - Enter grid coordinates in the format 'row,col' (e.g., '2,5') to move the arm.
 - Type 'detect' to start the detection and spraying program.
 - Type 'exit' to quit the program.
 - Use W/S/A/D/Q/E keys to adjust servos manually at any time.
   W/S: Move the middle servo (vertical) up/down
   A/D: Move the base servo (horizontal) left/right
   Q/E: Move the spray servo (spray nozzle) up/down
   ESC: Stop keyboard control
        """)

        # Start keyboard listener
        keyboard_listener = start_keyboard_listener()

        while True:
            try:
                input_str = input("Enter command (format: row,col | detect | exit): ").strip().lower()
                if input_str == 'detect':
                    reset_arm_position()

                    # Initialize video stream
                    print("Starting video stream...")
                    logging.info("Starting video stream...")
                    stream_url = 'tcp://127.0.0.1:8888'  
                    stream = VideoStream(stream_url).start()
                    time.sleep(2.0)
                    print("Video stream started.")
                    logging.info("Video stream started.")

                    # Start video display thread
                    video_thread = threading.Thread(target=video_display_thread, args=(stream,))
                    video_thread.start()

                    # Run detection and spraying
                    try:
                        detect_and_spray(stream)
                    except KeyboardInterrupt:
                        print("\nDetection and spraying interrupted by user.")
                        logging.info("Detection and spraying interrupted by user.")
                        stop_all_motors()
                        stream.stop()
                        cv2.destroyAllWindows()
                        sys.exit(0)

                   
                    video_thread.join()
                    

                elif input_str == 'exit':
                    print("Exiting program.")
                    logging.info("Program exited by user.")
                    break

                else:
                    # Parse grid coordinates
                    try:
                        row_str, col_str = input_str.split(',')
                        row = int(row_str.strip())
                        col = int(col_str.strip())

                        # Validate grid coordinates
                        if (row, col) not in GRID_COORDS:
                            print("Invalid grid coordinates. Please enter values within the operational grid range.")
                            logging.warning(f"Invalid grid coordinates entered: ({row}, {col})")
                            continue

                        # Retrieve servo angles
                        servo_angles = get_servo_angles(row, col, GRID_COORDS)

                        if servo_angles:
                            Servo_Base = servo_angles['B']
                            Servo_Middle = servo_angles['M']
                            Servo_Spray = servo_angles['S']

                            print(f"Moving to grid cell ({row}, {col}) with Servo Angles -> Base: {Servo_Base}°, Middle: {Servo_Middle}°, Spray: {Servo_Spray}°")
                            logging.info(f"Moving to grid cell ({row}, {col}) with Servo Angles -> Base: {Servo_Base}°, Middle: {Servo_Middle}°, Spray: {Servo_Spray}°")

                            # Move servos to the calculated angles smoothly
                            move_servos_smoothly(Servo_Base, Servo_Middle, Servo_Spray)

                            print_current_servo_angles()

                        else:
                            print(f"No servo angles found for grid cell ({row}, {col}).")
                            logging.warning(f"No servo angles found for grid cell ({row}, {col}).")

                    except ValueError:
                        print("Invalid input. Please enter two integer values separated by a comma (e.g., '2,5').")
                        logging.warning(f"Invalid input format: {input_str}")
                        continue

            except KeyboardInterrupt:
                print("\nExiting program.")
                logging.info("Program exited via keyboard interrupt.")
                break

        # Cleanup GPIO and video streams if necessary
        stop_all_motors()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()



