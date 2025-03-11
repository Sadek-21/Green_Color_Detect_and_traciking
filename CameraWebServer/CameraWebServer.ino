#include <esp_camera.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

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

    // Configure the camera
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA;
    config.pixel_format = PIXFORMAT_JPEG;  // for streaming
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    // Adjust camera settings
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);        // flip it back
        s->set_brightness(s, 1);   // up the brightness just a bit
        s->set_saturation(s, -2);  // lower the saturation
    }
    s->set_framesize(s, FRAMESIZE_QVGA);

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
                String data = client.readStringUntil('\n');
                data.trim(); // Remove any extra whitespace or newline characters

                // Parse the data (format: "pan_angle,tilt_angle")
                int commaIndex = data.indexOf(',');
                if (commaIndex > 0) {
                    int pan_angle = data.substring(0, commaIndex).toInt();
                    int tilt_angle = data.substring(commaIndex + 1).toInt();

                    // Constrain servo angles to valid range
                    pan_angle = constrain(pan_angle, 0, 180);
                    tilt_angle = constrain(tilt_angle, 60, 100);

                    // Debug: Print received angles
                    Serial.printf("Received: Pan=%d, Tilt=%d\n", pan_angle, tilt_angle);

                    // Move servos
                    panServo.write(pan_angle);
                    tiltServo.write(tilt_angle);

                    // Debug: Print actual servo angles
                    Serial.printf("Actual: Pan=%d, Tilt=%d\n", panServo.read(), tiltServo.read());

                    // Send a response back to the client
                    client.println("OK");
                } else {
                    client.println("Invalid data format");
                }
            }
        }

        // Close the connection
        client.stop();
        Serial.println("Client disconnected");
    }
}