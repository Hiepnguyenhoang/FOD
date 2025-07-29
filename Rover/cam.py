import cv2
import serial
import time
import os

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=3280,
    capture_height=2464,
    display_width=640,
    display_height=640,
    framerate=21,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def camera_control(ser, filename, status_file):
    global camera_running
    window_title = "CSI Camera"
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return None
    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
    print("Camera opened. Press button to capture and close.")

    distance = None  # default distance

    while camera_running:
        ret_val, frame = video_capture.read()
        if not ret_val:
            print("Error: Unable to read frame from camera")
            break

        cv2.imshow(window_title, frame)

        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Signal received: {message}")
            if "dist=" in message:
                try:
                    distance = int(message.split("dist=")[1].strip())
                    print(f"Distance updated: {distance}")
                except ValueError:
                    print("Invalid distance format.")

            if message == "CLOSE_CAM":
                print("Closing camera...")
                cv2.imwrite(filename, frame)
                print(f"Image captured and saved as {filename}")

                # Ghi trạng thái và khoảng cách vào file
                with open(status_file, "w") as f:
                    f.write(f"ready;distance={distance}")

                camera_running = False

        if cv2.waitKey(1) & 0xFF == 27:
            print("Closing camera manually...")
            camera_running = False

    video_capture.release()
    cv2.destroyAllWindows()

def listen_to_arduino():
    global camera_running
    arduino_port = "/dev/ttyACM0"
    baud_rate = 115200
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    camera_running = False
    filename = "hiep003.jpg"
    status_file = "status.txt"  # File trạng thái

    # Đảm bảo file trạng thái luôn tồn tại
    if not os.path.exists(status_file):
        with open(status_file, "w") as f:
            f.write("idle")

    print("Listening for Arduino signal...")
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Signal received: {message}")
            if "OPEN_CAM" in message and not camera_running:
                print("Opening camera...")
                camera_running = True
                camera_control(ser, filename, status_file)
            elif "EXIT" in message:
                print("Exiting program...")
                break

if __name__ == "__main__":
    listen_to_arduino()

