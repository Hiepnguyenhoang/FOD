# yolo_utils.py

import cv2
import numpy as np
import pickle
from ultralytics import YOLO
from imutils import perspective
import imutils
from PIL import Image

class_names = {
    0: "Battery",
    2: "Bottle", 
    3: "Lighter", 
    4: "Can", 
    5: "Piler",
    6: "Cutter", 
    7: "Rock", 
    8: "Metal",
    9: "Plastic", 
    1: "Other"
}

colors = {
    0: (255, 0, 0),
    1: (128, 0, 0),
    2: (0, 255, 0),
    3: (0, 0, 255),
    4: (255, 255, 0),
    5: (255, 0, 255),
    6: (0, 255, 255),
    7: (128, 0, 128),
    8: (128, 128, 0),
    9: (0, 128, 128),
}

def process_full_output(image_pil: Image.Image, model_path="best.pt", camera_matrix_file="cameraMatrix.pkl", d=0.7):
    model = YOLO(model_path)
    img_cv = np.array(image_pil)
    img_cv = cv2.cvtColor(img_cv, cv2.COLOR_RGB2BGR)
    bbox_img = img_cv.copy()
    rotated_img = img_cv.copy()

    with open(camera_matrix_file, "rb") as f:
        camera_matrix = pickle.load(f)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    def compute_Z(u, v):
        return d / np.sqrt(((u - cx)/fx)**2 + ((v - cy)/fy)**2 + 1)

    def pixel_to_world(u, v, Z):
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return X, Y

    results = model(img_cv)[0]
    log_result = ""

    for box in results.boxes.data.cpu().numpy():
        x1, y1, x2, y2 = map(int, box[:4])
        conf = float(box[4])
        class_id = int(box[5])
        label = f"{class_names.get(class_id, str(class_id))} ({conf*100:.1f}%)"
        color = colors.get(class_id, (200, 200, 200))

        # --- Bounding box mặc định ---
        cv2.rectangle(bbox_img, (x1, y1), (x2, y2), color, 3)
        cv2.putText(bbox_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Tính kích thước từ bounding box thường
        center_u = (x1 + x2) / 2
        center_v = (y1 + y2) / 2
        Z_box = compute_Z(center_u, center_v)
        X1, Y1 = pixel_to_world(x1, y1, Z_box)
        X2, Y2 = pixel_to_world(x2, y2, Z_box)
        width_bbox = abs(X2 - X1) * 1000  # mm
        height_bbox = abs(Y2 - Y1) * 1000  # mm

        cv2.putText(bbox_img, f"{width_bbox:.1f} x {height_bbox:.1f} mm", (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # --- Rotate bounding box ---
        
        pad = 20  # mở rộng box để crop
        h_img, w_img = img_cv.shape[:2]
        cx1 = max(x1 - pad, 0)
        cy1 = max(y1 - pad, 0)
        cx2 = min(x2 + pad, w_img)
        cy2 = min(y2 + pad, h_img)
        crop_img = img_cv[cy1:cy2, cx1:cx2]
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        width_rotated = height_rotated = None
        box_pts = None

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            box_pts = cv2.boxPoints(rect)
            box_pts = np.array(box_pts, dtype="float32")
            # Dịch các điểm box về tọa độ gốc ảnh
            box_pts[:, 0] += cx1
            box_pts[:, 1] += cy1
            box_pts = perspective.order_points(box_pts)

            cv2.drawContours(rotated_img, [box_pts.astype(np.int32)], -1, color, 2)

            pts_world = []
            for pt in box_pts:
                Z = compute_Z(pt[0], pt[1])
                X, Y = pixel_to_world(pt[0], pt[1], Z)
                pts_world.append((X, Y))

            width_rotated = np.linalg.norm(np.array(pts_world[1]) - np.array(pts_world[0])) * 1000
            height_rotated = np.linalg.norm(np.array(pts_world[3]) - np.array(pts_world[0])) * 1000

            cv2.putText(rotated_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(rotated_img, f"{width_rotated:.1f} x {height_rotated:.1f} mm", (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Log kết quả
        if width_rotated is not None and height_rotated is not None:
            rotated_str = f"{width_rotated:.1f} x {height_rotated:.1f} mm"
        else:
            rotated_str = "N/A"

        log_result += (
            f"- {label} ➤ [BBox] {width_bbox:.1f} x {height_bbox:.1f} mm | "
            f"[Rotated] {rotated_str}\n"
        )

    original_pil = image_pil
    bbox_pil = Image.fromarray(cv2.cvtColor(bbox_img, cv2.COLOR_BGR2RGB))
    rotated_pil = Image.fromarray(cv2.cvtColor(rotated_img, cv2.COLOR_BGR2RGB))
    return original_pil, bbox_pil, rotated_pil, log_result
