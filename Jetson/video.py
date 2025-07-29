import cv2
import pickle
from ultralytics import YOLO
import numpy as np
import torch

def load_camera_params(camera_matrix_file, dist_coeffs_file):
    with open(camera_matrix_file, "rb") as f:
        camera_matrix = pickle.load(f)
    with open(dist_coeffs_file, "rb") as f:
        dist_coeffs = pickle.load(f)
    return camera_matrix, dist_coeffs

def undistort_frame(frame, camera_matrix, dist_coeffs):
    return cv2.undistort(frame, camera_matrix, dist_coeffs)

def compute_Z(u, v, fx, fy, cx, cy, d):
    Z = d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)
    return Z

def pixel_to_world(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def calculate_bbox_size(bbox, camera_matrix):
    x_min, y_min, x_max, y_max = bbox
    d = 2.3  # Estimated distance from camera to object

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    Z_min = compute_Z(x_min, y_min, fx, fy, cx, cy, d)
    Z_max = compute_Z(x_max, y_max, fx, fy, cx, cy, d)

    X_min, Y_min = pixel_to_world(x_min, y_min, Z_min, fx, fy, cx, cy)
    X_max, Y_max = pixel_to_world(x_max, y_max, Z_max, fx, fy, cx, cy)

    width = abs(X_max - X_min) * 1000
    height = abs(Y_max - Y_min) * 1000

    return width, height

def process_detected_classes(model, results, camera_matrix):
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)
            class_name = model.names[class_id]
            bbox = box.xyxy[0].tolist()
            width_mm, height_mm = calculate_bbox_size(bbox, camera_matrix)

            print(f"Detected class: {class_name}")
            print(f"Width: {width_mm:.1f} mm, Height: {height_mm:.1f} mm")

            if class_name == "person":
                print("Detected a person!")

def run_yolo_on_video(video_path, model_path, camera_matrix_file, dist_coeffs_file, output_video="output_result.avi"):
    # Load model and camera parameters
    model = YOLO(model_path).to("cuda")
    camera_matrix, dist_coeffs = load_camera_params(camera_matrix_file, dist_coeffs_file)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Failed to open video.")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Video writer
    out = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'XVID'), fps, (width, height))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted_frame = undistort_frame(frame, camera_matrix, dist_coeffs)
        with torch.no_grad():
    	    results = model(undistorted_frame)

        # Draw detection on frame
        result_frame = results[0].plot()
        out.write(result_frame)

        # Show result
        cv2.imshow("YOLO Detection", result_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Optionally process classes (comment out to improve performance)
        # process_detected_classes(model, results, camera_matrix)

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Detection results saved to {output_video}")

if __name__ == "__main__":
    video_path = "/home/long/test_box_ab/vd3.webm"
    model_path = "/home/long/test_box_ab/best.pt"
    camera_matrix_file = "/home/long/test_box_ab/cameraMatrix.pkl"
    dist_coeffs_file = "/home/long/test_box_ab/dist.pkl"

    print("Running YOLO on the video...")
    run_yolo_on_video(video_path, model_path, camera_matrix_file, dist_coeffs_file)

