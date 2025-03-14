import cv2
from ultralytics import YOLO

def run_yolo_on_image(image_path, model_path="yolov8n.pt"):
    # Load YOLO model
    model = YOLO(model_path)
    
    # Run YOLO detection
    results = model(image_path)
    if results is None or len(results) == 0:
        print("No results returned by YOLO.")
        return
    
    # Get result image with plotted detections
    result_image = results[0].plot()
    if result_image is None:
        print("Failed to generate result image.")
        return
    
    # Display the result image
    cv2.imshow("YOLOv8 Detection", result_image)
    # Uncomment the line below if you want to wait for a key press to close the window
    # cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save the result image
    output_path = "result_image.jpg"
    if not cv2.imwrite(output_path, result_image):
        print("Failed to save the image.")
        return
    print(f"Detection result saved as {output_path}")
    
    # Process the results to extract class names
    extract_detected_classes(model, "result_image.jpg")

def extract_detected_classes(model, image_path):
    # Perform YOLO detection again on the saved result image
    results = model(image_path)
    for result in results:
        for box in result.boxes:
            # Ensure this block is properly indented
            class_id = int(box.cls)  # Class ID as an integer
            class_name = model.names[class_id]  # Class name
            x = class_name
            print(f"Detected class: {x}")
            print(x)

if __name__ == "__main__":
    image_path = "/home/long/hiep003.jpg"  # Path to your input image
    model_path = "yolov8n.pt"             # Path to your YOLO model
    
    run_yolo_on_image(image_path, model_path)
