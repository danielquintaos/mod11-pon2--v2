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
