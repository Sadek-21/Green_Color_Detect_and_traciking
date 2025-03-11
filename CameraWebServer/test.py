import socket

# ESP32-CAM IP address and port
ESP32_IP = "192.168.3.14"  # Replace with your ESP32-CAM IP address
ESP32_PORT = 81  # Use the same port as in the ESP32-CAM code

# Function to send servo angles via TCP
def send_servo_angles(pan_angle, tilt_angle):
    try:
        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ESP32_IP, ESP32_PORT))

        # Send the servo angles (format: "pan_angle,tilt_angle")
        data = f"{pan_angle},{tilt_angle}\n"
        print(f"Sending: {data.strip()}")  # Debug: Print the data being sent
        sock.send(data.encode())

        # Receive a response from the ESP32-CAM
        response = sock.recv(1024).decode().strip()
        print(f"Sent: Pan={pan_angle}, Tilt={tilt_angle}, Response: {response}")

        # Close the socket
        sock.close()
    except Exception as e:
        print(f"Error sending servo angles: {e}")

# Main function
def main():
    while True:
        print("Enter pan angle (0-180) and tilt angle (60-100), separated by a comma (e.g., 90,90):")
        input_data = input().strip()

        try:
            pan_angle, tilt_angle = map(int, input_data.split(','))
            send_servo_angles(pan_angle, tilt_angle)
        except ValueError:
            print("Invalid input. Please enter two integers separated by a comma.")

if __name__ == "__main__":
    main()