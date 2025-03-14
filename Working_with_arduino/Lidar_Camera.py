import cv2
import serial
import time

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
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

def camera_control(ser):
    global camera_running
    window_title = "CSI Camera"
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return None
    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
    print("Camera opened. Press button to capture and close.")
    captured_frame = None
    while camera_running:
        ret_val, frame = video_capture.read()
        if not ret_val:
            print("Error: Unable to read frame from camera")
            break
        cv2.imshow(window_title, frame)
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(f"Signal received: {message}")
            if message == "CLOSE_CAM":
                print("Closing camera...")
                captured_frame = frame
                camera_running = False
        if cv2.waitKey(1) & 0xFF == 27:
            print("Closing camera manually...")
            camera_running = False
    video_capture.release()
    cv2.destroyAllWindows()
    return captured_frame

def listen_to_arduino():
    global camera_running
    arduino_port = "/dev/ttyUSB0"
    baud_rate = 9600
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    camera_running = False
    print("Listening for Arduino signal...")
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(f"Signal received: {message}")
            if message == "OPEN_CAM" and not camera_running:
                print("Opening camera...")
                camera_running = True
                frame = camera_control(ser)
                if frame is not None:
                    filename = "hiep003.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Image saved as {filename}")
            elif message == "EXIT":
                print("Exiting program...")
                break

if __name__ == "__main__":
    listen_to_arduino()

