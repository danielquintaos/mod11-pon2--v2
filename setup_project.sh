#!/bin/sh

# Create the project root directory
mkdir -p project

# Navigate into the project directory
cd project

# --- esp32-cam ---
mkdir -p esp32-cam/src esp32-cam/include esp32-cam/lib

# Create main.cpp with content
cat << 'EOF' > esp32-cam/src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"

// Camera configuration
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// Serial communication settings
#define BAUD_RATE 115200

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Configure camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  // Capture image
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Send image size
  Serial.write((uint8_t*)&fb->len, sizeof(fb->len));

  // Send image buffer
  Serial.write(fb->buf, fb->len);

  // Return the frame buffer back to be reused
  esp_camera_fb_return(fb);

  // Small delay
  delay(1000);
}
EOF

# Create platformio.ini with content
cat << 'EOF' > esp32-cam/platformio.ini
[env:esp32cam]
platform = espressif32
board = esp32cam
framework = arduino
monitor_speed = 115200

lib_deps =
    esp32-camera

upload_port = /dev/cu.SLAB_USBtoUART
monitor_port = /dev/cu.SLAB_USBtoUART
EOF

# --- computer ---
mkdir -p computer/src

# Create face_detection.py with content
cat << 'EOF' > computer/src/face_detection.py
import cv2
import numpy as np
import serial
import struct

# Serial communication settings
SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your serial port
BAUD_RATE = 115200

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Initialize face detector
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def receive_image():
    # Read image size
    size_data = ser.read(4)
    img_size = struct.unpack('<I', size_data)[0]

    # Read image data
    img_data = b''
    while len(img_data) < img_size:
        img_data += ser.read(img_size - len(img_data))

    # Decode image
    img_array = np.frombuffer(img_data, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    return img

def main():
    while True:
        # Receive image from ESP32-CAM
        frame = receive_image()

        if frame is not None:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)

            # Draw rectangles around faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Display the output
            cv2.imshow('Face Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to receive frame.")

    # Cleanup
    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
EOF

# Create requirements.txt with content
cat << 'EOF' > computer/requirements.txt
opencv-python
numpy
pyserial
EOF

# --- nix ---
mkdir -p nix

# Create flake.nix with content
cat << 'EOF' > nix/flake.nix
{
  description = "ESP32-CAM Real-Time Face Detection Project";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {
      devShells.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          python3
          python3Packages.numpy
          python3Packages.opencv4
          python3Packages.pyserial
          platformio
        ];
      };
    };
}
EOF

# Create shell.nix with content
cat << 'EOF' > nix/shell.nix
{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    python3
    python3Packages.numpy
    python3Packages.opencv4
    python3Packages.pyserial
    platformio
  ];
}
EOF

# --- Root files ---

# Create README.md with content
cat << 'EOF' > README.md
# ESP32-CAM Real-Time Face Detection Project

## Overview

This project implements a real-time face detection system using an ESP32-CAM and a computer. The ESP32-CAM captures images and sends them via USB to a computer, which processes the images to detect human faces and displays the results with detection rectangles.

## Folder Structure

- \`esp32-cam/\`: Contains code and configuration for the ESP32-CAM module.
  - \`src/main.cpp\`: The main program for the ESP32-CAM.
  - \`platformio.ini\`: Configuration file for building and uploading code using PlatformIO.
- \`computer/\`: Contains the computer-side application.
  - \`src/face_detection.py\`: Python script for receiving images and performing face detection.
  - \`requirements.txt\`: Lists Python dependencies.
- \`nix/\`: Contains NixOS configuration files for dependency management.
  - \`flake.nix\`: Defines the Nix flake for the project.
  - \`shell.nix\`: Sets up the development shell environment.
- \`README.md\`: Documentation and setup instructions.

## Prerequisites

- **Hardware**:
  - ESP32-CAM module
  - USB to TTL Serial adapter (e.g., FTDI module)
  - USB cable
  - Computer running NixOS with flakes enabled

- **Software**:
  - PlatformIO Core (CLI)
  - Nix package manager
  - Python 3.x

## Setup Instructions

### ESP32-CAM

1. **Wiring**:
   - Connect the ESP32-CAM to the USB to TTL Serial adapter.
   - Ensure RX/TX pins are cross-connected (RX to TX, TX to RX).
   - Connect GPIO0 to GND to enable flash mode.

2. **Flashing the ESP32-CAM**:
   - Navigate to the \`esp32-cam/\` directory.
   - Modify \`platformio.ini\` to set the correct \`upload_port\` and \`monitor_port\`.
   - Run \`platformio run --target upload\` to build and upload the firmware.
   - Disconnect GPIO0 from GND after flashing.

### Computer Application

1. **Enter Nix Shell**:
   - From the project root, run \`nix develop\` to enter the development environment.

2. **Install Python Dependencies** (if not using Nix):
   - Navigate to the \`computer/\` directory.
   - Run \`pip install -r requirements.txt\`.

3. **Modify Serial Port**:
   - In \`face_detection.py\`, set \`SERIAL_PORT\` to the correct serial port connected to the ESP32-CAM.

## Running the Application

1. **Start the Face Detection Script**:
   - Run \`python src/face_detection.py\` from the \`computer/\` directory.

2. **View the Output**:
   - A window titled "Face Detection" should display the video feed with rectangles around detected faces.

3. **Exit**:
   - Press 'q' in the display window to exit the application.

## Troubleshooting

- **Serial Communication Errors**:
  - Ensure the serial port is correct in both \`platformio.ini\` and \`face_detection.py\`.
  - Check that the baud rate matches on both the ESP32-CAM and the computer script.

- **No Video Feed or Faces Detected**:
  - Verify the ESP32-CAM is properly connected and functioning.
  - Check for errors in the console output.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
EOF

# Create LICENSE with content
cat << 'EOF' > LICENSE
MIT License

Copyright (c) 2023

Permission is hereby granted, free of charge, to any person obtaining a copy of this software...
[Include the full text of the MIT License here]
EOF

# Optionally, create esp32-cam/include/camera_pins.h if needed
cat << 'EOF' > esp32-cam/include/camera_pins.h
#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// Camera pin definitions
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#endif // CAMERA_PINS_H
EOF

# Navigate back to the parent directory
cd ..

echo "Project files and folder structure have been created successfully."

