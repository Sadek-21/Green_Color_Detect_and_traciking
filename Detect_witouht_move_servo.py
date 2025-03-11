import cv2
import numpy as np
import serial
import time
import requests
from PIL import Image
from io import BytesIO

# Initialize serial communication with ESP32-CAM
ser = serial.Serial('COM6', 115200)  # Replace 'COM6' with the correct port
time.sleep(2)  # Wait for the serial connection to initialize

# ESP32-CAM video stream URL
esp32_cam_url = "http://192.168.3.14/capture"  # Replace with the correct URL

# Create a window for trackbars
cv2.namedWindow("Trackbars")

# Create trackbars for adjusting HSV range
def nothing(x):
    pass

cv2.createTrackbar("H Lower", "Trackbars", 35, 179, nothing)  # Hue lower bound
cv2.createTrackbar("H Upper", "Trackbars", 85, 179, nothing)  # Hue upper bound
cv2.createTrackbar("S Lower", "Trackbars", 50, 255, nothing)  # Saturation lower bound
cv2.createTrackbar("S Upper", "Trackbars", 255, 255, nothing)  # Saturation upper bound
cv2.createTrackbar("V Lower", "Trackbars", 50, 255, nothing)  # Value lower bound
cv2.createTrackbar("V Upper", "Trackbars", 255, 255, nothing)  # Value upper bound

# Servo control parameters
pan_angle = 90  # Initial pan angle (X-axis)
tilt_angle = 90  # Initial tilt angle (Y-axis)
servo_speed = 2  # Speed of servo movement (adjust as needed)
servo_delay = 0.02  # Delay between servo movements (in seconds)

# Define tilt servo limits (adjust these values as needed)
TILT_MIN_ANGLE = 60  # Minimum angle for tilt (down)
TILT_MAX_ANGLE = 100  # Maximum angle for tilt (up)

# Function to constrain servo angles to valid range
def constrain_angle(angle, min_angle, max_angle):
    return max(min_angle, min(max_angle, angle))

# Function to fetch and decode the ESP32-CAM stream
def fetch_esp32_cam_frame(url):
    try:
        response = requests.get(url, stream=True)
        print(f"Response status code: {response.status_code}")  # Debug print
        if response.status_code == 200:
            bytes_data = bytes()
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')  # Start of JPEG frame
                b = bytes_data.find(b'\xff\xd9')  # End of JPEG frame
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]  # Extract the JPEG frame
                    bytes_data = bytes_data[b+2:]  # Remove the processed frame
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    return frame
    except Exception as e:
        print(f"Error fetching frame: {e}")
    return None

while True:
    # Fetch frame from ESP32-CAM stream
    frame = fetch_esp32_cam_frame(esp32_cam_url)
    if frame is None:
        print("Error: Unable to fetch frame. Retrying...")
        time.sleep(1)  # Wait before retrying
        continue
    
    # Flip and resize the frame
    frame = cv2.flip(frame, 1)
    frame = cv2.resize(frame, (300, 300))  # Resize for better performance
    
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Get current trackbar positions
    h_lower = cv2.getTrackbarPos("H Lower", "Trackbars")
    h_upper = cv2.getTrackbarPos("H Upper", "Trackbars")
    s_lower = cv2.getTrackbarPos("S Lower", "Trackbars")
    s_upper = cv2.getTrackbarPos("S Upper", "Trackbars")
    v_lower = cv2.getTrackbarPos("V Lower", "Trackbars")
    v_upper = cv2.getTrackbarPos("V Upper", "Trackbars")
    
    # Define HSV range for green color
    green_lower = np.array([h_lower, s_lower, v_lower], np.uint8)
    green_upper = np.array([h_upper, s_upper, v_upper], np.uint8)
    
    # Create a mask for green color
    mask = cv2.inRange(hsv, green_lower, green_upper)
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill small holes
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sort contours by area (largest first)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    
    # Get the center of the frame
    rows, cols, _ = frame.shape
    center_x = int(cols / 2)
    center_y = int(rows / 2)
    
    # Draw a crosshair at the center of the frame (in red)
    cv2.line(frame, (center_x, 0), (center_x, rows), (0, 0, 255), 2)
    cv2.line(frame, (0, center_y), (cols, center_y), (0, 0, 255), 2)
    
    # Process the largest contour (if any)
    if contours:
        (x, y, w, h) = cv2.boundingRect(contours[0])
        
        # Draw a rectangle around the detected green object (in red)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        # Calculate the center of the detected object
        medium_x = int((x + x + w) / 2)
        medium_y = int((y + y + h) / 2)
        
        # Draw lines and text for the object's center (in red)
        cv2.line(frame, (medium_x, 0), (medium_x, rows), (0, 0, 255), 2)
        cv2.line(frame, (0, medium_y), (cols, medium_y), (0, 0, 255), 2)
        cv2.putText(frame, f"X: {medium_x}, Y: {medium_y}", (medium_x + 10, medium_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Calculate the error between the object center and the frame center
        error_x = medium_x - center_x
        error_y = medium_y - center_y
        
        # Pan servo (left/right)
        if error_x > 30:  # Object is to the right
            pan_angle += servo_speed  # Move right
        elif error_x < -30:  # Object is to the left
            pan_angle -= servo_speed  # Move left
        
        # Tilt servo (up/down)
        if error_y > 30:  # Object is below
            tilt_angle += servo_speed  # Move down
        elif error_y < -30:  # Object is above
            tilt_angle -= servo_speed  # Move up
        
        # Constrain servo angles to valid range
        pan_angle = constrain_angle(pan_angle, 0, 180)  # Pan servo range (0 to 180)
        tilt_angle = constrain_angle(tilt_angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE)  # Tilt servo range
        
        # Send target positions to ESP32-CAM
        command = f"{pan_angle},{tilt_angle}\n"
        ser.write(command.encode())
        print(f"Sent to ESP32-CAM: {command.strip()}")  # Debug print
        time.sleep(servo_delay)  # Add delay for smoother movement

    # Display the frame and mask
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    
    # Add a small delay to reduce CPU usage
    time.sleep(0.01)
    
    # Exit on 'Esc' key press
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release resources
cv2.destroyAllWindows()
ser.close()