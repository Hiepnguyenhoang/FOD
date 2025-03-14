import cv2
import serial
from ultralytics import YOLO

def run_yolo_on_image(image_path, model_path="yolov8n.pt", serial_port="/dev/ttyUSB0"):
    # Kết nối với Arduino qua Serial
    try:
        ser = serial.Serial(serial_port, 9600, timeout=10)
        print("Connected to Arduino on", serial_port)
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return

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
    
    cv2.destroyAllWindows()
    
    # Save the result image
    output_path = "result_image.jpg"
    if not cv2.imwrite(output_path, result_image):
        print("Failed to save the image.")
        return
    print(f"Detection result saved as {output_path}")
    
    # Process the results to extract class names and trigger LiDAR
    process_detected_classes(model, results, ser)

def process_detected_classes(model, results, ser):
	
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)  # Class ID as an integer
            class_name = model.names[class_id]  # Class name
            
            x = class_name
            print(f"Detected class: {x}")
            print(x)

            # Nếu phát hiện "person", gửi tín hiệu đến Arduino
            if x == "person":
                print("Sending OPEN_LIDAR to Arduino...")
                ser.write(b"OPEN_LIDAR\n")
                return  # Chỉ cần gửi một lần khi phát hiện "person"

if __name__ == "__main__":
    image_path = "/home/long/hiep003.jpg"  # Path to your input image
    model_path = "yolov8n.pt"             # Path to your YOLO model
    serial_port = "/dev/ttyUSB0"          # Arduino Serial Port
    
    run_yolo_on_image(image_path, model_path, serial_port)
