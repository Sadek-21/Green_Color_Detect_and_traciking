import socket

# ESP32-CAM IP address and port
ESP32_IP = "192.168.3.14"  # Replace with your ESP32-CAM IP address
ESP32_PORT = 81  # Use the same port as in the ESP32-CAM code

# Function to send commands via TCP
def send_command(command):
    try:
        # Create a TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ESP32_IP, ESP32_PORT))

        # Send the command
        sock.send(f"{command}\n".encode())

        # Receive a response from the ESP32-CAM
        response = sock.recv(1024).decode().strip()
        print(f"Sent: {command}, Response: {response}")

        # Close the socket
        sock.close()
    except Exception as e:
        print(f"Error sending command: {e}")

# Main function
def main():
    while True:
        print("Enter command (UP, DOWN, LEFT, RIGHT, or EXIT to quit):")
        command = input().strip().upper()

        if command == "EXIT":
            break

        if command in ["UP", "DOWN", "LEFT", "RIGHT"]:
            send_command(command)
        else:
            print("Invalid command. Please try again.")

if __name__ == "__main__":
    main()