import cv2
import pickle
import serial
from ultralytics import YOLO
from imutils import perspective
import numpy as np
import imutils
import time


def compute_Z(u, v, fx, fy, cx, cy, d):
    return d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)

def pixel_to_world(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def calculate_bbox_size(bbox, camera_matrix_path, d):
    with open(camera_matrix_path, "rb") as f:
        camera_matrix = pickle.load(f)
    
    tl, tr, br, bl = bbox
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    Z_tl = compute_Z(tl[0], tl[1], fx, fy, cx, cy, d)
    Z_tr = compute_Z(tr[0], tr[1], fx, fy, cx, cy, d)
    Z_bl = compute_Z(bl[0], bl[1], fx, fy, cx, cy, d)

    X_tl, Y_tl = pixel_to_world(tl[0], tl[1], Z_tl, fx, fy, cx, cy)
    X_tr, Y_tr = pixel_to_world(tr[0], tr[1], Z_tr, fx, fy, cx, cy)
    X_bl, Y_bl = pixel_to_world(bl[0], bl[1], Z_bl, fx, fy, cx, cy)

    width = np.sqrt((X_tr - X_tl)**2 + (Y_tr - Y_tl)**2) * 1000
    height = np.sqrt((X_bl - X_tl)**2 + (Y_bl - Y_tl)**2) * 1000
    return width, height

def process_detected_classes(model, results, image_path, camera_matrix_file, ser, d):
    orig_img = cv2.imread(image_path)
    all_contours_img = orig_img.copy()
    all_contours_list = []
    detected = False

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)
            class_name = model.names[class_id]
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x_min, y_min, x_max, y_max = xyxy

            crop_img = orig_img[y_min:y_max, x_min:x_max]
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)
            edged = cv2.Canny(gray, 50, 100)
            edged = cv2.dilate(edged, None, iterations=1)
            edged = cv2.erode(edged, None, iterations=1)
            cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            if not cnts:
                continue

            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            box_pts = cv2.boxPoints(rect)
            box_pts = np.array(box_pts, dtype="float32")
            box_pts[:, 0] += x_min
            box_pts[:, 1] += y_min
            box_pts = perspective.order_points(box_pts)

            width_mm, height_mm = calculate_bbox_size(box_pts, camera_matrix_file, d)

            print(f"Detected class: {class_name}")
            print(f"Width: {width_mm:.1f} mm, Height: {height_mm:.1f} mm")
            all_contours_list.append(box_pts.astype(np.int32))

            print("Sending COLLECT_FOD to Arduino...")
            ser.write(b"COLLECT_FOD\n")
            detected = True
            break

        if detected:
            break

    if not detected:
        print("No relevant objects detected. Sending OPEN_LIDAR to Arduino...")
        ser.write(b"OPEN_LIDAR\n")

    if all_contours_list:
        cv2.drawContours(all_contours_img, all_contours_list, -1, (0, 255, 0), 2)
        cv2.imwrite("all_contours_result.jpg", all_contours_img)
        print("All contours result saved as all_contours_result.jpg")
        # Hiển thị ảnh phát hiện
        cv2.imshow("YOLO Detection Result", all_contours_img)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()


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
        d = float(d_line.split("=")[1]) / 100
    except Exception as e:
        print(f"Failed to read distance: {e}")

    model = YOLO(model_path).to("cuda")
    results = model(image_path)
    if results is None or len(results) == 0:
        print("No results returned by YOLO.")
        ser.write(b"OPEN_LIDAR\n")
        return

    result_image = results[0].plot()
    if result_image is not None:
        cv2.imwrite("result_image.jpg", result_image)
        print("Detection result saved as result_image.jpg")

    process_detected_classes(model, results, image_path, camera_matrix_file, ser, 0.85)

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

