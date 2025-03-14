from ultralytics import YOLO

# Load mô hình YOLOv8
model = YOLO('yolov8n.pt')  # Hoặc model bạn đã huấn luyện

# Dự đoán trên ảnh
results = model("/home/long/result_image.jpg")  # Thay "image.jpg" bằng đường dẫn tới ảnh của bạn

# Truy xuất kết quả classification
for result in results:
    for box in result.boxes:
        # Lấy tên lớp (class name) từ box
        class_id = int(box.cls)  # ID lớp (số nguyên)
        class_name = model.names[class_id]  # Tên lớp
        
        # Gắn giá trị cho biến x
        x = class_name
        print(f"Detected class: {x}")
