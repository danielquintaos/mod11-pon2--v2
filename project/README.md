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
