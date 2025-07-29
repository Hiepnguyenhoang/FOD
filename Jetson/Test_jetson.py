import cv2
import pickle
import torch
from ultralytics import YOLO
import time
import numpy as np

def undistort_image(image_path, camera_matrix_file="cameraMatrix.pkl", dist_coeffs_file="dist.pkl"):
    # Load camera matrix and distortion coefficients
    with open(camera_matrix_file, "rb") as f:
        camera_matrix = pickle.load(f)
    with open(dist_coeffs_file, "rb") as f:
        dist_coeffs = pickle.load(f)

    # Read the input image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return None

    # Undistort the image
    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)
    return undistorted_image

def run_yolo_on_image(image_path, model_path="best.pt", camera_matrix_file="cameraMatrix.pkl", dist_coeffs_file="dist.pkl"):
    # Undistort the input image
    undistorted_image = undistort_image(image_path, camera_matrix_file, dist_coeffs_file)
    if undistorted_image is None:
        return

    # Save the undistorted image temporarily
    temp_image_path = "undistorted_image.jpg"
    cv2.imwrite(temp_image_path, undistorted_image)

    # Load YOLO model on GPU
    device = "cuda"
    model = YOLO(model_path).to(device)

    # Run YOLO detection on the undistorted image
    with torch.no_grad():
    	results = model(temp_image_path)
    if results is None or len(results) == 0:
        print("No results returned by YOLO.")
        print("No objects detected.")
        return

    # Get result image with plotted detections
    result_image = results[0].plot()
    if result_image is None:
        print("Failed to generate result image.")
        return

    # Display the result image
    cv2.imshow("Result", result_image)
    # cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

    # Save the result image
    output_path = "result_image.jpg"
    if not cv2.imwrite(output_path, result_image):
        print("Failed to save the image.")
        return
    print(f"Detection result saved as {output_path}")

    # Process the results to extract class names
    process_detected_classes(model, results)

def compute_Z(u, v, fx, fy, cx, cy, d):
    Z = d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)
    return Z

def pixel_to_world(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def calculate_bbox_size(bbox, camera_matrix_path):
    # Load camera matrix from file
    with open(camera_matrix_path, "rb") as f:
        camera_matrix = pickle.load(f)

    # Extract bounding box pixel coordinates
    x_min, y_min, x_max, y_max = bbox
    
    # Distance  from camera to object (in meters)
    d = 0.5
    
    # Camera intrinsic parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    Z_min = compute_Z(x_min, y_min, fx, fy, cx, cy, d)
    Z_max = compute_Z(x_max, y_max, fx, fy, cx, cy, d)

    # Convert each corner of the bounding box
    X_min, Y_min = pixel_to_world(x_min, y_min, Z_min, fx, fy, cx, cy)
    X_max, Y_max = pixel_to_world(x_max, y_max, Z_max, fx, fy, cx, cy)

    # Calculate width (distance between X_min and X_max)
    width = abs(X_max - X_min) * 1000  # Convert to millimeters

    # Calculate height (distance between Y_min and Y_max)
    height = abs(Y_max - Y_min) * 1000  # Convert to millimeters

    return width, height


def process_detected_classes(model, results):
    detected = False  # Flag to check if any object is detected

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)  # Class ID as an integer
            class_name = model.names[class_id]  # Class name

            # Get bounding box coordinates
            bbox = box.xyxy[0].tolist()  # [x_min, y_min, x_max, y_max]
            width_mm, height_mm = calculate_bbox_size(bbox, camera_matrix_file)

            print(f"Detected class: {class_name}")
            print(f"Width: {width_mm:.1f} mm, Height: {height_mm:.1f} mm")

            # If "person" is detected, print a message
            if class_name == "person":
                print("Detected a person!")
                detected = True
                break
        if detected:
            break

    # If no "person" or any relevant object is detected
    if not detected:
        print("No relevant objects detected.")

if __name__ == "__main__":
    image_path = "/home/long/test_box_ab/hiep_20250624_115340.jpg"  # Path to your input image
    model_path = "/home/long/test_box_ab/best.pt"  # Path to your YOLO model
    camera_matrix_file = "/home/long/test_box_ab/cameraMatrix.pkl"  # Path to camera matrix file
    dist_coeffs_file = "/home/long/test_box_ab/dist.pkl"  # Path to distortion coefficients file

    print("Running YOLO on the specified image...")
    run_yolo_on_image(image_path, model_path, camera_matrix_file, dist_coeffs_file)
