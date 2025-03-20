import cv2
import numpy as np
import time
import requests
import socket

# ESP32-CAM video stream URL
esp32_cam_url = "http://192.168.24.56/capture"  # Replace with the correct URL

#http://192.168.3.14/   http://192.168.158.56    192.168.218.56  http://192.168.24.56

# TCP Server Configuration for ESP32-CAM -fjl
ESP32_IP = "192.168.24.56"  # Replace with your ESP32-CAM IP address
ESP32_PORT = 82  # Use the same port as in the ESP32-CAM code

# Servo control parameters
pan_angle = 90  # Initial pan angle (X-axis)
tilt_angle = 40  # Initial tilt angle (Y-axis)
servo_speed = 2  # Speed of servo movement (adjust as needed)

# Define tilt servo limits (adjust these values as needed)
TILT_MIN_ANGLE = 20  # Minimum angle for tilt (down)
TILT_MAX_ANGLE = 80  # Maximum angle for tilt(up)

# Function to constrain servo angles to valid range
def constrain_angle(angle, min_angle, max_angle):
    return max(min_angle, min(max_angle, angle))



# Function to fetch and decode the ESP32-CAM stream
def fetch_esp32_cam_frame(url):
    try:
        response = requests.get(url, stream=True)
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



# Function to send servo angles via TCP
def send_servo_angles(pan_angle, tilt_angle, detected):
    try:
        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to {ESP32_IP}:{ESP32_PORT}...")  # Debug: Print connection attempt
        sock.connect((ESP32_IP, ESP32_PORT))
        print("Connection successful!")  # Debug: Confirm connection

        # Send the servo angles (format: "pan_angle,tilt_angle\n")
        data = f"{pan_angle},{tilt_angle},{int(detected)}\n"
        print(f"Sending: {data.strip()}")  # Debug: Print the data being sent
        sock.send(data.encode())

        # Wait for a response from the ESP32-CAM
        response = sock.recv(1024).decode().strip()
        print(f"Response: {response}")  # Debug: Print the response

        # Close the socket
        sock.close()
    except Exception as e:
        print(f"Error sending servo angles: {e}")

#we
# Main function for object detection and servo control
def main():
    global pan_angle, tilt_angle  # Declare pan_angle and tilt_angle as global variables

    while True:
        # Fetch frame from ESP32-CAM stream
        frame = fetch_esp32_cam_frame(esp32_cam_url)
        if frame is None:
            print("Error: Unable to fetch frame. Retrying...")
            time.sleep(1)
            continue

        # Flip and resize the frame
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (300, 300))  # Resize for better performance

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for red color
        # Red has two ranges in HSV due to the hue wrap-around
        red_lower1 = np.array([0, 50, 50])    # Lower range for red
        red_upper1 = np.array([10, 255, 255]) # Upper range for red
        red_lower2 = np.array([170, 50, 50])  # Lower range for red (wrap-around)
        red_upper2 = np.array([180, 255, 255])# Upper range for red (wrap-around)

        # Create masks for red color
        mask1 = cv2.inRange(hsv, red_lower1, red_upper1)  # Mask for the first red range
        mask2 = cv2.inRange(hsv, red_lower2, red_upper2)  # Mask for the second red range
        mask = cv2.bitwise_or(mask1, mask2)  # Combine the two masks

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
        detected = False
        if contours:
            (x, y, w, h) = cv2.boundingRect(contours[0])

            # Draw a rectangle around the detected red object (in red)
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

            # Set detection status to true
            detected = True

        # Send servo angles and detection status via T.C.P
        send_servo_angles(pan_angle, tilt_angle, detected)

        # Display the frame and mask
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        # Add a small delay to reduce CPU usage
        time.sleep(0.01)

        # Exit on 'Esc' key press
        key = cv2.waitKey(1)
        if key == 27:
            break

if __name__ == "__main__":
    main()