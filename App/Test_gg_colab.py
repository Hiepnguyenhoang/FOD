import cv2
import pickle
from ultralytics import YOLO
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
from google.colab.patches import cv2_imshow  

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

    # Load YOLO model
    model = YOLO(model_path)

    # Run YOLO detection on the undistorted image
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
    cv2_imshow(result_image)
    cv2.waitKey(0)  # Wait for a key press to close the window
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

    # Distance from camera to object (in meters)
    d = 1.0

    # Unpack corners
    tl, tr, br, bl = bbox

    # Camera intrinsic parameters
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Compute Z for each corner
    Z_tl = compute_Z(tl[0], tl[1], fx, fy, cx, cy, d)
    Z_tr = compute_Z(tr[0], tr[1], fx, fy, cx, cy, d)
    Z_br = compute_Z(br[0], br[1], fx, fy, cx, cy, d)
    Z_bl = compute_Z(bl[0], bl[1], fx, fy, cx, cy, d)

    # Convert each corner of the bounding box to world coordinates
    X_tl, Y_tl = pixel_to_world(tl[0], tl[1], Z_tl, fx, fy, cx, cy)
    X_tr, Y_tr = pixel_to_world(tr[0], tr[1], Z_tr, fx, fy, cx, cy)
    X_br, Y_br = pixel_to_world(br[0], br[1], Z_br, fx, fy, cx, cy)
    X_bl, Y_bl = pixel_to_world(bl[0], bl[1], Z_bl, fx, fy, cx, cy)

    # Calculate width (distance between top-left and top-right)
    width = np.sqrt((X_tr - X_tl) ** 2 + (Y_tr - Y_tl) ** 2) * 1000  # mm

    # Calculate height (distance between top-left and bottom-left)
    height = np.sqrt((X_bl - X_tl) ** 2 + (Y_bl - Y_tl) ** 2) * 1000  # mm

    return width, height

def get_bbox_from_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged = cv2.Canny(gray, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    # find contours in the edge map
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    (cnts, _) = contours.sort_contours(cnts)

    # loop over the contours individually
    for c in cnts:
	# if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 100: # Liên quan tới khả năng phát hiện vật thể (giữ lại vật thể > 100 pixel)
            continue

        # compute the rotated bounding box of the contour
        orig = image.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

        (tl, tr, br, bl) = box
        output_image_path = "contour_result.jpg"
        cv2.imwrite(output_image_path, orig)
        print(f"Contour result saved as {output_image_path}")
        return box

def process_detected_classes(model, results):
    detected = False  # Flag to check if any object is detected

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)  # Class ID as an integer
            class_name = model.names[class_id]  # Class name

            # Get bounding box coordinates using helper function
            bbox = get_bbox_from_image(image_path)
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
    image_path = "/content/018efae6-e977-4f9b-b443-ebb457600fe4.jpg"  # Path to your input image
    model_path = "/content/best.pt"  # Path to your YOLO model
    camera_matrix_file = "/content/cameraMatrix.pkl"  # Path to camera matrix file
    dist_coeffs_file = "/content/dist.pkl"  # Path to distortion coefficients file

    print("Running YOLO on the specified image...")
    run_yolo_on_image(image_path, model_path, camera_matrix_file, dist_coeffs_file)