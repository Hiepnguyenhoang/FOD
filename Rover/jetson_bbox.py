import cv2
import pickle
import serial
from ultralytics import YOLO
import numpy as np
import time


def compute_Z(u, v, fx, fy, cx, cy, d):
    return d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)

def pixel_to_world(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def calculate_bbox_size_from_xyxy(xyxy, camera_matrix_path, d):
    with open(camera_matrix_path, "rb") as f:
        camera_matrix = pickle.load(f)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    x1, y1, x2, y2 = xyxy
    center_u = (x1 + x2) / 2
    center_v = (y1 + y2) / 2

    Z_box = compute_Z(center_u, center_v, fx, fy, cx, cy, d)

    X1, Y1 = pixel_to_world(x1, y1, Z_box, fx, fy, cx, cy)
    X2, Y2 = pixel_to_world(x2, y2, Z_box, fx, fy, cx, cy)

    width_bbox = abs(X2 - X1) * 1000  # mm
    height_bbox = abs(Y2 - Y1) * 1000  # mm

    return width_bbox, height_bbox

def process_detected_classes(model, results, image_path, camera_matrix_file, ser, d):
    orig_img = cv2.imread(image_path)
    detected = False

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)
            class_name = model.names[class_id]
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy

            width_mm, height_mm = calculate_bbox_size_from_xyxy(xyxy, camera_matrix_file, d)

            print(f"Detected class: {class_name}")
            print(f"Width: {width_mm:.1f} mm, Height: {height_mm:.1f} mm")

            # Vẽ bbox lên ảnh gốc
            cv2.rectangle(orig_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imwrite("bbox_result.jpg", orig_img)
            print("Bounding box saved as bbox_result.jpg")

            print("Sending COLLECT_FOD to Arduino...")
            ser.write(b"@COLLECT_FOD\n")
            detected = True
            break

        if detected:
            break

    if not detected:
        print("No relevant objects detected. Sending OPEN_LIDAR to Arduino...")
        ser.write(b"@OPEN_LIDAR\n")
        time.sleep(0.5)

def run_yolo_on_image(image_path, model_path, camera_matrix_file, serial_port, status_file):
    try:
        ser = serial.Serial(serial_port, 115200, timeout=10)
        print("Connected to Arduino on", serial_port)
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return

    # Đọc khoảng cách từ status.txt
    try:
        with open(status_file, "r") as f:
            content = f.read().strip()
        d_line = [s for s in content.split(";") if "distance=" in s][0]
        d = float(d_line.split("=")[1]) / 100  # convert cm to meters
    except Exception as e:
        print(f"Failed to read distance: {e}")
        return

    model = YOLO(model_path).to("cuda")
    results = model(image_path)
    if results is None or len(results) == 0:
        print("No results returned by YOLO.")
        ser.write(b"@OPEN_LIDAR\n")
        return

    result_image = results[0].plot()
    if result_image is not None:
        cv2.imwrite("result_image.jpg", result_image)
        print("Detection result saved as result_image.jpg")

        # Hiển thị ảnh phát hiện
        cv2.imshow("YOLO Detection Result", result_image)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    process_detected_classes(model, results, image_path, camera_matrix_file, ser, d)


if __name__ == "__main__":
    status_file = "status.txt"
    image_path = "/home/long/hiep003.jpg"
    model_path = "/home/long/test_box_ab/best.pt"
    camera_matrix_file = "/home/long/test_box_ab/cameraMatrix.pkl"
    serial_port = "/dev/ttyACM0"

    print("Waiting for new image...")
    while True:
        with open(status_file, "r") as f:
            content = f.read().strip()

        if content.startswith("ready"):
            print("New image detected. Running YOLO...")
            run_yolo_on_image(image_path, model_path, camera_matrix_file, serial_port, status_file)
            with open(status_file, "w") as f:
                f.write("idle")

        time.sleep(1)

