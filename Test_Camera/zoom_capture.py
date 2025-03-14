import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=21,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

def zoom_frame(frame, zoom_factor):
    height, width = frame.shape[:2]
    new_height, new_width = int(height / zoom_factor), int(width / zoom_factor)
    start_y = (height - new_height) // 2
    start_x = (width - new_width) // 2
    cropped_frame = frame[start_y:start_y + new_height, start_x:start_x + new_width]
    return cv2.resize(cropped_frame, (width, height))

def camera_with_digital_zoom():
    zoom_factor = 1.0
    max_zoom = 10.0
    min_zoom = 1.0
    window_title = "Camera Pi V2 - Digital Zoom"

    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open Camera Pi V2")
        return

    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
    print("Camera opened. Use '+' to zoom in, '-' to zoom out, and 'q' to quit.")

    while True:
        ret_val, frame = video_capture.read()
        if not ret_val:
            print("Error: Unable to read frame from camera")
            break

        zoomed_frame = zoom_frame(frame, zoom_factor)
        cv2.imshow(window_title, zoomed_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting program...")
            break
        elif key == ord('+') and zoom_factor < max_zoom:
            zoom_factor += 0.1
            print(f"Zooming in: {zoom_factor:.1f}x")
        elif key == ord('-') and zoom_factor > min_zoom:
            zoom_factor -= 0.1
            print(f"Zooming out: {zoom_factor:.1f}x")
        elif keyCode == ord('s'):
                    filename = "hiep003.jpg"
                    cv2.imwrite(filename, frame)
                    print("Image saved ")


    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_with_digital_zoom()

