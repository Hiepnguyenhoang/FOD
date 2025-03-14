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

def zoom_frame(frame, zoom_factor):
    """Zoom frame to the specified zoom factor."""
    height, width = frame.shape[:2]
    new_height = int(height / zoom_factor)
    new_width = int(width / zoom_factor)
    start_y = (height - new_height) // 2
    start_x = (width - new_width) // 2
    cropped_frame = frame[start_y:start_y + new_height, start_x:start_x + new_width]
    return cv2.resize(cropped_frame, (width, height))

def camera_control(ser, filename):
    global camera_running
    window_title = "CSI Camera"
    zoom_factor = 1  # Default zoom factor
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return None
    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
    print("Camera opened. Press button to capture and close.")
    while camera_running:
        ret_val, frame = video_capture.read()
        if not ret_val:
            print("Error: Unable to read frame from camera")
            break
        
        zoomed_frame = zoom_frame(frame, zoom_factor)
        cv2.imshow(window_title, zoomed_frame)
        
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(f"Signal received: {message}")
            if message == "CLOSE_CAM":
                print("Closing camera...")
                cv2.imwrite(filename, zoomed_frame)
                print(f"Image captured and saved as {filename}")
                camera_running = False
            elif message == "zoom x2":
                zoom_factor = 2
                print("Zoom factor set to 2")
            elif message == "zoom x3":
                zoom_factor = 3
                print("Zoom factor set to 3")
        if cv2.waitKey(1) & 0xFF == 27:
            print("Closing camera manually...")
            camera_running = False
    video_capture.release()
    cv2.destroyAllWindows()

def listen_to_arduino():
    global camera_running
    arduino_port = "/dev/ttyUSB0"
    baud_rate = 9600
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    camera_running = False
    filename = "hiep003.jpg"
    print("Listening for Arduino signal...")
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(f"Signal received: {message}")
            if message == "OPEN_CAM" and not camera_running:
                print("Opening camera...")
                camera_running = True
                camera_control(ser, filename)
            elif message == "EXIT":
                print("Exiting program...")
                break

if __name__ == "__main__":
    listen_to_arduino()
