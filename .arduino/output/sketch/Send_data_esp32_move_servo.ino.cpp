#include <Arduino.h>
#line 1 "C:\\Users\\mohsa\\OneDrive\\Desktop\\Green_Color_Detect\\Send_data_esp32_move_servo\\Send_data_esp32_move_servo.ino"
#include <WiFi.h>
#include <ESP32Servo.h>

// Servo Configuration
#define PAN_PIN 14   // GPIO pin for the pan servo
#define TILT_PIN 15  // GPIO pin for the tilt servo

Servo panServo;  // Servo for X-axis (pan)
Servo tiltServo; // Servo for Y-axis (tilt)

// Initial servo positions
int pan_angle = 90;  // Center position for pan servo
int tilt_angle = 90; // Center position for tilt servo

// Wi-Fi credentials
const char* ssid = "HUAWEI-1CFEJ9";       // Replace with your Wi-Fi SSID
const char* password = "20242024";  // Replace with your Wi-Fi password

// TCP Server Configuration
WiFiServer tcpServer(81);  // Use port 81 for TCP communication

void setup() {
    Serial.begin(115200);

    // Initialize servos
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    panServo.write(pan_angle);
    tiltServo.write(tilt_angle);
    Serial.println("Servos initialized to 90 degrees");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());

    // Start the TCP server
    tcpServer.begin();
    Serial.println("TCP server started on port 81");
}

void loop() {
    // Check for a new TCP client
    WiFiClient client = tcpServer.available();
    if (client) {
        Serial.println("New client connected");

        while (client.connected()) {
            if (client.available()) {
                String command = client.readStringUntil('\n');
                command.trim(); // Remove any extra whitespace or newline characters

                // Process the command
                if (command == "UP") {
                    tilt_angle = constrain(tilt_angle + 10, 60, 100); // Move tilt servo up
                    tiltServo.write(tilt_angle);
                    Serial.println("Tilt servo moved UP");
                } else if (command == "DOWN") {
                    tilt_angle = constrain(tilt_angle - 10, 60, 100); // Move tilt servo down
                    tiltServo.write(tilt_angle);
                    Serial.println("Tilt servo moved DOWN");
                } else if (command == "LEFT") {
                    pan_angle = constrain(pan_angle + 10, 0, 180); // Move pan servo left
                    panServo.write(pan_angle);
                    Serial.println("Pan servo moved LEFT");
                } else if (command == "RIGHT") {
                    pan_angle = constrain(pan_angle - 10, 0, 180); // Move pan servo right
                    panServo.write(pan_angle);
                    Serial.println("Pan servo moved RIGHT");
                } else {
                    Serial.println("Invalid command");
                }

                // Send a response back to the client
                client.println("OK");
            }
        }

        // Close the connection
        client.stop();
        Serial.println("Client disconnected");
    }
}
