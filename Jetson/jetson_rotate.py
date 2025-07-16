import cv2
import pickle
from ultralytics import YOLO
from imutils import perspective
from imutils import contours
import numpy as np
import imutils

# def undistort_image(image_path, camera_matrix_file="cameraMatrix.pkl", dist_coeffs_file="dist.pkl"):
#    with open(camera_matrix_file, "rb") as f:
#        camera_matrix = pickle.load(f)
#    with open(dist_coeffs_file, "rb") as f:
#        dist_coeffs = pickle.load(f)

#    image = cv2.imread(image_path)
#    if image is None:
#        print(f"Failed to load image: {image_path}")
#        return None

#    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)
#    return undistorted_image

def run_yolo_on_image(image_path, model_path="best.pt", camera_matrix_file="cameraMatrix.pkl", dist_coeffs_file="dist.pkl"):
#    undistorted_image = undistort_image(image_path, camera_matrix_file, dist_coeffs_file)
#    if undistorted_image is None:
#        return

#    temp_image_path = "undistorted_image.jpg"
#    cv2.imwrite(temp_image_path, undistorted_image)

    model = YOLO(model_path).to('cuda')
    results = model(image_path)
    if results is None or len(results) == 0:
        print("No results returned by YOLO.")
        return

    result_image = results[0].plot()
    if result_image is None:
        print("Failed to generate result image.")
        return

    # Show result using OpenCV
    cv2.imshow("YOLO Result", result_image)
#    cv2.waitKey(0)
    cv2.destroyAllWindows()

    output_path = "result_image.jpg"
    if not cv2.imwrite(output_path, result_image):
        print("Failed to save the image.")
        return
    print(f"Detection result saved as {output_path}")

    process_detected_classes(model, results, image_path, camera_matrix_file)

def compute_Z(u, v, fx, fy, cx, cy, d):
    Z = d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)
    return Z

def pixel_to_world(u, v, Z, fx, fy, cx, cy):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return X, Y

def calculate_bbox_size(bbox, camera_matrix_path):
    with open(camera_matrix_path, "rb") as f:
        camera_matrix = pickle.load(f)

    d = 1.64
    tl, tr, br, bl = bbox

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    Z_tl = compute_Z(tl[0], tl[1], fx, fy, cx, cy, d)
    Z_tr = compute_Z(tr[0], tr[1], fx, fy, cx, cy, d)
    Z_br = compute_Z(br[0], br[1], fx, fy, cx, cy, d)
    Z_bl = compute_Z(bl[0], bl[1], fx, fy, cx, cy, d)

    X_tl, Y_tl = pixel_to_world(tl[0], tl[1], Z_tl, fx, fy, cx, cy)
    X_tr, Y_tr = pixel_to_world(tr[0], tr[1], Z_tr, fx, fy, cx, cy)
    X_br, Y_br = pixel_to_world(br[0], br[1], Z_br, fx, fy, cx, cy)
    X_bl, Y_bl = pixel_to_world(bl[0], bl[1], Z_bl, fx, fy, cx, cy)

    width = np.sqrt((X_tr - X_tl) ** 2 + (Y_tr - Y_tl) ** 2) * 1000
    height = np.sqrt((X_bl - X_tl) ** 2 + (Y_bl - Y_tl) ** 2) * 1000

    return width, height

def process_detected_classes(model, results, image_path, camera_matrix_file):
    detected = False
    orig_img = cv2.imread(image_path)

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
                print("No contour found in crop.")
                continue
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            box_pts = cv2.boxPoints(rect)
            box_pts = np.array(box_pts, dtype="float32")
            box_pts[:, 0] += x_min
            box_pts[:, 1] += y_min
            box_pts = perspective.order_points(box_pts)

            crop_box_pts = box_pts.copy()
            crop_box_pts[:, 0] -= x_min
            crop_box_pts[:, 1] -= y_min
            cv2.drawContours(crop_img, [crop_box_pts.astype(np.int32)], -1, (0, 255, 0), 2)

            width_mm, height_mm = calculate_bbox_size(box_pts, camera_matrix_file)

            print(f"Detected class: {class_name}")
            print(f"Width: {width_mm:.1f} mm, Height: {height_mm:.1f} mm")

            if 'all_contours_img' not in locals():
                all_contours_img = orig_img.copy()
                all_contours_list = []
            all_contours_list.append(box_pts.astype(np.int32))

        if detected:
            break
    if 'all_contours_img' in locals() and all_contours_list:
            cv2.drawContours(all_contours_img, all_contours_list, -1, (0, 255, 0), 2)
            output_image_path = "all_contours_result.jpg"
            cv2.imwrite(output_image_path, all_contours_img)
            print(f"All contours result saved as {output_image_path}")
    if not detected:
        print("No relevant objects detected.")

if __name__ == "__main__":
    image_path = "/home/long/test_box_ab/hiep_20250605_221226.jpg"  # Path to your input image
    model_path = "/home/long/test_box_ab/best.pt"  # Path to your YOLO model
    camera_matrix_file = "/home/long/test_box_ab/cameraMatrix.pkl"  # Path to camera matrix file
    dist_coeffs_file = "/home/long/test_box_ab/dist.pkl"  # Path to distortion coefficients file

    print("Running YOLO on the specified image...")
    run_yolo_on_image(image_path, model_path, camera_matrix_file, dist_coeffs_file)
