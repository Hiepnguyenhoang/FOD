import serial
import time

try:
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=2)
    time.sleep(2)  # chờ Arduino khởi động nếu vừa reset
    print("Serial connected. Sending @OPEN_LIDAR...")
    ser.write(b"@OPEN_LIDAR\n")
    print("The @OPEN_LIDAR command was successfully sent")
    ser.close()
except Exception as e:
    print(f"Lỗi: {e}")

