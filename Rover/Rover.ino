#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float initialYaw = 0;
float currentYaw = 0;
Servo servo1;  // Vai (D7)
Servo servo2;  // Khuỷu tay (D8)
Servo servo3;  // Cổ tay (D9)
Servo servo4;  // Kẹp (D10)
bool dangTimLine = true;
// Mảng lưu góc hiện tại của từng servo
int angles[4] = {90, 70, 160, 30}; // servo1, servo2, servo3, servo4
char selectedServo;
char selectedDovat = 'f';
uint8_t stop_distance_ultrasonic = 35; // Khoảng cách phát hiện vật cản

// Kết nối SRF-05 hoặc SRF-04
#define trigPin 8
#define echoPin 9

// L298N kết nối Arduino
#define motorA1 13
#define motorA2 12
#define motorAspeed 11
#define motorB1 6
#define motorB2 7
#define motorBspeed 5

// Kết nối cảm biến dò line analog
#define L_S 2 // trái
#define S_S 3 // giữa
#define R_S 4 // phải

long duration_ultrasonic; //
float distance_ultrasonic;  // biến khoảng cách
bool isReceivingCommand = false;
String jetsonCommand = ""; 
bool lidarThreshold = true; // OK
//lidar
int dist_lidar;      // LiDAR actually measured distance value
int strength_lidar;  // LiDAR signal strength
int check;     // check numerical value storage
uint8_t uart[9];   // store data measured by LiDAR
const int HEADER = 0x59; // data package frame header
bool lidarActive = true;
bool check_lidar = false;
char huongQuay[10] = "";
bool  timVatBangHC_SR04();
long doKhoangCanhTrungBinh();
bool quetVatGiua();
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAspeed, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBspeed, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  analogWrite(motorAspeed, 130);
  analogWrite(motorBspeed, 130);
  servo1.attach(A1);
  servo2.attach(A0);
  servo3.attach(A2);
  servo4.attach(A3);
  servo1.write(angles[0]);
  servo2.write(angles[1]);
  servo3.write(angles[2]);
  servo4.write(angles[3]);
  if (!mpu.testConnection()) {
    while (1);
  }
  delay(1000);
}

void loop() {
  checkSerialCommand();  // xử lý Jetson trước

  // chỉ đọc Lidar khi không có dữ liệu lệnh
  if (Serial.available() == 0) {
    lidarSensor();
  }

  if (lidarActive) {
    doLinePhatHienVatCan();
  }
}
void lidarSensor() {
    if (!check_lidar &&lidarActive && Serial.available()>=9) {
    if (Serial.read() == HEADER) {
      uart[0] = HEADER;
      if (Serial.read() == HEADER) {
        uart[1] = HEADER;
        for (int i = 2; i < 9; i++) {
          uart[i] = Serial.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {
          dist_lidar = uart[2] + uart[3] * 256;
          strength_lidar = uart[4] + uart[5] * 256;
          Serial.println("lidarSensor");
          Serial.print("dist=");
          Serial.println(dist_lidar);
          if (dist_lidar > 20 && dist_lidar < 110) {
            Stop();
            check_lidar = true;
            delay(200);
            lidarActive = false;
            lidarThreshold = false;
            Serial.println("OPEN_CAM");
            Serial.println("lidarSensor");
            Serial.print("dist=");
            Serial.println(dist_lidar); 
            Serial.println("CLOSE_CAM");
            flushSerialBuffer();
            while (Serial.available() > 0) {
              Serial.read();
            }
          }
        }
      }
    }
  }
}

void checkSerialCommand() {
  if (lidarThreshold) return;
  while (Serial.available()) {
    char c = Serial.read();

    // Bỏ qua ký tự không in được, trừ \n
    if (!isPrintable(c) && c != '\n') continue;

    // Nếu chưa bắt đầu nhận lệnh, chỉ cho phép bắt đầu bằng @
    if (!isReceivingCommand) {
      if (c == '@') {
        isReceivingCommand = true;
        jetsonCommand = "@";
      }
      continue;
    }

    // Đang nhận lệnh: nếu gặp newline thì xử lý
    if (c == '\n') {
      jetsonCommand.trim();

      // Chỉ xử lý nếu chuỗi có độ dài > 1 và bắt đầu bằng @
      if (jetsonCommand.length() <= 1 || jetsonCommand.charAt(0) != '@') {
        jetsonCommand = "";
        isReceivingCommand = false;
        return;
      }

      // Cắt phần lệnh (bỏ dấu @)
      String cmd = jetsonCommand.substring(1);


      // So sánh lệnh
      if (cmd.equalsIgnoreCase("COLLECT_FOD")) {
        Serial.println(">> Thực hiện nhặt vật thể");
        pickUpObjects();
        check_lidar = false;
        lidarActive = true;
        lidarThreshold = true;
        Serial.println("CLOSE_CAM");
      } 
      else if (cmd.equalsIgnoreCase("OPEN_LIDAR")) {
        Serial.println(">> Tiếp tục quét lidar");
        lidarActive = true;
        check_lidar = false;
        lidarThreshold = true;
      } 
      // Reset trạng thái
      jetsonCommand = "";
      isReceivingCommand = false;
    } 
    else {
      // Ghép ký tự vào chuỗi
      jetsonCommand += c;

      // Nếu quá dài thì reset
      if (jetsonCommand.length() > 64) {
        jetsonCommand = "";
        isReceivingCommand = false;
      }
    }
  }
}

void flushSerialBuffer() {
  for (int i = 0; i < 5; i++) {
    while (Serial.available()) {
      Serial.read();
    }
    delay(10);
  }
}


void doLinePhatHienVatCan() {
  uint8_t left_sensor_state  = digitalRead(L_S);  // Trái
  uint8_t s_sensor_state     = digitalRead(S_S);  // Giữa
  uint8_t right_sensor_state = digitalRead(R_S);  // Phải
  if (dangTimLine) lidarSensor();
  // In giá trị để debug

  // Logic dò line (0 = phát hiện vạch đen, 1 = nền trắng)
  if ((left_sensor_state == 0) && (s_sensor_state == 1) && (right_sensor_state == 0)) {
    forword();  
    if (dangTimLine) lidarSensor();
  }
  else if ((left_sensor_state == 1) || (s_sensor_state == 1 && right_sensor_state == 1)) {
    turnLeft(); 
    if (dangTimLine) lidarSensor();
  }
  else if ((right_sensor_state == 1) || (s_sensor_state == 1 && left_sensor_state == 1)) {
    turnRight(); 
    if (dangTimLine) lidarSensor();
  }
  
  else {
    Stop(); // Không thấy gì → dừng lại
  }
  if (dangTimLine) lidarSensor();
  ultrasonic();
}

//void ultrasonic() {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  duration_ultrasonic = pulseIn(echoPin, HIGH, 30000);
//  distance_ultrasonic = (duration_ultrasonic * 0.034) / 2;
// // if (distance_ultrasonic < 25) {
//   // xuLyVatCan();
// // }
//  Serial.print("HCSR04: \n");
//  Serial.print("Distance: ");
//  Serial.println(distance_ultrasonic);
//
//}
void ultrasonic() {
  static unsigned long lastUltrasonicTime = 0;
  unsigned long now = millis();

  if (now - lastUltrasonicTime < 60) {
    // Nếu chưa đủ thời gian giữa hai lần đo, bỏ qua lần này
    return;
  }

  lastUltrasonicTime = now;  // Cập nhật thời gian đo cuối

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration_ultrasonic = pulseIn(echoPin, HIGH, 30000);
  distance_ultrasonic = (duration_ultrasonic * 0.034) / 2;

//  if (distance_ultrasonic < 35) {
//     xuLyVatCan();
//     }
}


float readYaw() {
  static float yaw = 0;
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float gyroZ = mpu.getRotationZ() / 131.0;

  if (lastTime != 0) {
    float deltaTime = (now - lastTime) / 1000.0;
    yaw += gyroZ * deltaTime;
  }

  lastTime = now;
  return yaw;
}

void quayTrai90Do() {
  initialYaw = readYaw();
  float targetYaw = initialYaw + 90.0;

  quayTraiTaiCho();

  while (readYaw() < targetYaw - 2) {  // Dung sai 2 độ
    delay(50);
  }

  Stop();
}
void quayTrai180Do() {
  initialYaw = readYaw();
  float targetYaw = initialYaw + 180.0;

  quayTraiTaiCho();

  while (readYaw() < targetYaw - 2) {  // Dung sai 2 độ
    delay(50);
  }

  Stop();
}

// chua dung
void xuLyVatCan() {
    // Xử lý khi gặp vật cản
  if (distance_ultrasonic < stop_distance_ultrasonic) {
    Stop();
    delay(500);
      analogWrite(motorAspeed, 200);
  analogWrite(motorBspeed, 200);
    // Lùi
    backward();
    delay(300);
    Stop(); delay(300);

    turnLeft();
    delay(900);
    Stop(); delay(400);

    // Tiến
    forword();
    delay(800);
    Stop(); delay(250);

    turnRight();
    delay(700);
    Stop(); delay(200);

    // Tiến
    forword();
    delay(300);
    Stop(); delay(200);

    turnRight();
    delay(800);
    Stop(); delay(200);

    forword();
    while (digitalRead(L_S) == LOW) {
      Serial.println("Đang tìm lại line...");
    }
    
    backward();
    delay(100);
    Stop();
    delay(200);
    analogWrite(motorAspeed, 130);
    analogWrite(motorBspeed, 130);

  }
}
void armRobot(char selectedDovat){
        char cmd = Serial.read(); // Đọc lệnh từ Serial
        cmd = selectedDovat;
        switch (selectedDovat) {
            case 'a': // Chọn đồ vật A
                diChuyenServo(2, 70, 120);
               delay(2000);
               diChuyenServo(3, 160, 170);
                //diChuyenServo(1, 110, 85);
                diChuyenServo(4, angles[3], 65);
                delay(1000);
                diChuyenServo(2, 120, 70);
                delay(1000);
                break;
            case 'x': 
                diChuyenServo(4, 70, 20);
            default:
                Serial.println("Lenh khong hop le. Vui long nhap lai!");
                break;
        }
}
Servo& chonServo(int soServo) {
    switch (soServo) {
        case 1: return servo1;
        case 2: return servo2;
        case 3: return servo3;
        case 4: return servo4;
        default: return servo1;
    }
}

void canhTayHaTuTu(Servo &servo, int gocHienTai, int gocThayDoi) {
    if (gocHienTai > gocThayDoi) {
        for (int goc = gocHienTai; goc >= gocThayDoi; goc -= 5) {
            servo.write(goc);
            delay(400);
        }
    } else {
        for (int goc = gocHienTai; goc <= gocThayDoi; goc += 5) {
            servo.write(goc);
            delay(400);
        }
    }
    servo.write(gocThayDoi); // Đảm bảo kết thúc đúng vị trí
}

// Hàm chọn servo và di chuyển từ từ
void diChuyenServo(int soServo, int gocHienTai, int gocThayDoi) {
    Servo &servo = chonServo(soServo);
    canhTayHaTuTu(servo, gocHienTai, gocThayDoi);
}
long doKhoangCachTrungBinh() {
    long tong = 0;
    for (int i = 0; i < 3; i++) {
        ultrasonic();
        tong += distance_ultrasonic;
        delay(30);  // delay nhỏ để ổn định
    }
    return tong / 3;
}
void pickUpObjects () {
    int a;
    analogWrite(motorAspeed, 255);
    analogWrite(motorBspeed, 255);
    Stop();
    delay(300);
    quayTrai90Do();
    delay(200);
    Stop();
    delay(500);
    forword();
    if (dist_lidar > 40) {
    a = 22 * (dist_lidar);
    } else if (dist_lidar < 80 && dist_lidar > 40) {
    a = 22 * (dist_lidar - 18 - 40);
    } else if (dist_lidar > 80 && dist_lidar < 130) {
    a = 22 * (dist_lidar - 18 - 40);
    }
    delay(a);
    timVatBangHC_SR04();
    Stop();
    delay(2000);
    // Quay đầu xe
    quayTrai180Do();
    Stop();
    delay(400);
    // Đi thẳng đến khi phát hiện line
    forword();
    bool lineFound = false;
    while (!lineFound) {
        uint8_t left_sensor_state  = digitalRead(L_S);  // Trái
        uint8_t s_sensor_state     = digitalRead(S_S);  // Giữa
        uint8_t right_sensor_state = digitalRead(R_S);  // Phải

        int lineValue = left_sensor_state * 100 + s_sensor_state * 10 + right_sensor_state;

        if (lineValue == 111) {
            Stop();
            delay(500);
            selectedDovat = 'x';
            armRobot(selectedDovat);
            Stop();
            delay(500);
            backward();
            delay(200);
            turnLeft();
            delay(800);
            lineFound = true;
            dangTimLine = false;
            while (dangTimLine) {

            doLinePhatHienVatCan();
            lineValue = left_sensor_state * 100 + s_sensor_state * 10 + right_sensor_state;
            
            if (lineValue == 010) {
              dangTimLine = true; 
            }
            }
        }
        delay(10); // kiểm tra liên tục
        
    }

    // Dừng xe lại sau khi rẽ trái
    Stop();
}

void forword() {

  digitalWrite(motorA1, HIGH);digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);digitalWrite(motorB2, HIGH);
}

void backward() {
  digitalWrite(motorA1, LOW);digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);digitalWrite(motorB2, LOW);
}

void turnLeft() {
  digitalWrite(motorA1, HIGH);digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);digitalWrite(motorB2, LOW);
}

void turnRight() {
  digitalWrite(motorA1, LOW);digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);digitalWrite(motorB2, HIGH);
}

void Stop() {
  digitalWrite(motorA1, LOW);digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);digitalWrite(motorB2, LOW);
}
void quayTraiTaiCho(){
  digitalWrite(motorA1, HIGH);digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);digitalWrite(motorB2, LOW);
}
void quayPhaiTaiCho(){
  digitalWrite(motorA1, LOW);digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);digitalWrite(motorB2, HIGH);
}
bool luiRaVaKiemTraLai() {
    for (int i = 0; i < 10; i++) {
        backward();
        delay(70);
        Stop();

        delay(400);

        long kc = doKhoangCachTrungBinh();
        if (kc >= 15 && kc <= 17) {
            Stop();
            quetVatGiua();
            return true;
        if (huongQuay == "trai") {
            quayTraiTaiCho();
            delay(60);
            Stop();
            delay(200);// lệch trái → canh phải
        } else if (huongQuay == "phai") {
            quayPhaiTaiCho();
            delay(60);
            Stop();
            delay(200);  // lệch phải → canh trái
        }       
            selectedDovat = 'a';
            armRobot(selectedDovat);
            delay(1000);
            Stop();
            delay(200);
            return true;
    }       if (kc > 25) {
            break;
        }
    }
    forword();
    delay(300);
    Stop();
    delay(400);

    long kc = doKhoangCachTrungBinh();
    if (kc >= 15 && kc <= 16) {
            selectedDovat = 'a';
            armRobot(selectedDovat);
            delay(1000);
            Stop();
            return true;
        }
    return false;
}

bool tienLaiVaKiemTraLai() {
  long kc;
    for (int i = 0; i < 10; i++) {
        forword();
        delay(70);
        Stop();
        delay(400);
        long kc = doKhoangCachTrungBinh();
        Stop();
        delay(400);
        if (kc >= 15 && kc <= 16) {
            quetVatGiua();
        if (huongQuay == "trai") {
            quayTraiTaiCho();
            delay(60);
            Stop();
            delay(200);// lệch trái → canh phải
        } else if (huongQuay == "phai") {
            quayPhaiTaiCho();
            delay(60);
            Stop();
            delay(200);  // lệch phải → canh trái
        }       
            selectedDovat = 'a';
            armRobot(selectedDovat);
            delay(1000);
            Stop();
            return true;
        } else if (kc < 5) {
            break;
        }
    }
    backward();
    delay(300);
    Stop();
    delay(400);

    kc = doKhoangCachTrungBinh();
    if (kc >= 14 && kc <= 16) {
            selectedDovat = 'a';
            armRobot(selectedDovat);
            delay(1000);
            Stop();
            return true;
        }
    return false;
}

bool timVatBangHC_SR04() {
    const int GAN_GAP_MIN = 14;
    const int GAN_GAP_MAX = 16;
    const int PHAM_VI_TIEN_LAI = 25;
    long kc;
    delay(300);
    kc = doKhoangCachTrungBinh();

    if (kc >= GAN_GAP_MIN && kc <= GAN_GAP_MAX) {
        selectedDovat = 'a';
        armRobot(selectedDovat);
        delay(1000);
        Stop();
        return true;
    } else if (kc < GAN_GAP_MIN) {
        if (luiRaVaKiemTraLai()) return true;
    } else if (kc < PHAM_VI_TIEN_LAI) {
        if (tienLaiVaKiemTraLai()) return true;
    }

    for (int j = 0; j < 5; j++) {
        quayTraiTaiCho();
        delay(100);
        Stop();
        delay(300);

        kc = doKhoangCachTrungBinh();

        if (kc >= GAN_GAP_MIN && kc <= GAN_GAP_MAX) {
            quetVatGiua();
            return true;
        } else if (kc < GAN_GAP_MIN) {
          strcpy(huongQuay, "trai");

            return luiRaVaKiemTraLai();
        } else if (kc < PHAM_VI_TIEN_LAI) {
          strcpy(huongQuay, "trai");

            return tienLaiVaKiemTraLai();
        }
    }

    quayPhaiTaiCho(); delay(400); Stop(); delay(200);
    for (int j = 0; j < 5; j++) {
        quayPhaiTaiCho();
        delay(100);
        Stop();
        delay(300);

    kc = doKhoangCachTrungBinh();

        if (kc >= GAN_GAP_MIN && kc <= GAN_GAP_MAX) {
            quetVatGiua();
            return true;
        } else if (kc < GAN_GAP_MIN) {
            strcpy(huongQuay, "phai");

            return luiRaVaKiemTraLai();
        } else if (kc < PHAM_VI_TIEN_LAI) {
            strcpy(huongQuay, "phai");

            return tienLaiVaKiemTraLai();
        }
    }
    return false;
}

bool quetVatGiua() {
    int step_left = 0;
    int step_right = 0;
    int step = 0;
    long kc;

    while (true) {
        kc = doKhoangCachTrungBinh();

        if (kc < 16) {
            step_left = step;
        } else {
            break;  // Ra khỏi vật
        }

        quayTraiTaiCho();
        delay(40);
        Stop();
        delay(150);
        step++;
    }

    int step_back = step;  // lưu số bước quay trái

    // 2. Quay phải về vị trí ban đầu
    for (int i = 0; i < step_back; i++) {
        quayPhaiTaiCho();
        delay(40);
        Stop();
        delay(150);
    }

    // 3. Quay phải để tìm rìa bên phải
    step = 0;
    while (true) {
        kc = doKhoangCachTrungBinh();

        if (kc < 16) {
            step_right = step;
        } else {
            break;  // Ra khỏi vật
        }

        quayPhaiTaiCho();
        delay(40);
        Stop();
        delay(150);
        step++;
    }
    int step_center;
    if (step_left > step_right) {
        // Vật lệch trái → quay trái canh tâm
        step_center = (step_left + step_right) / 2;
        for (int i = 0; i < step_center; i++) {
            quayTraiTaiCho();
            delay(40);
            Stop();
            delay(150);
        }
    Stop();
    delay(100);
    kc = doKhoangCachTrungBinh();
                Stop();
    delay(100);
    if (kc >= 15.5 && kc <= 16){
     backward();
     delay(50);
     Stop();
     delay(100);
    selectedDovat = 'a';
    armRobot(selectedDovat);
    delay(100);
    Stop();
    return true;
    } else {
      backward();
      delay(80);
      Stop();
      delay(100);
      selectedDovat = 'a';
    armRobot(selectedDovat);
    delay(100);
    Stop();
    return true;}
    } else {
        // Vật lệch phải → quay phải canh tâm
        step_center = (step_left + step_right) / 2;
        for (int i = 0; i < step_center; i++) {
            quayTraiTaiCho();
            delay(40);
            Stop();
            delay(150);
            
        }
    Stop();
    delay(150);
    kc = doKhoangCachTrungBinh();
                Stop();
    delay(100);
    if (kc >= 15.5&& kc <= 16){
    backward();
    delay(50);
    Stop();
    delay(100);
    selectedDovat = 'a';
    armRobot(selectedDovat);
    delay(100);
    Stop();
    return true;
    }
    else {
      backward(); delay(80);
      Stop(); delay(100);
      selectedDovat = 'a';
    armRobot(selectedDovat);
    delay(100);
    Stop();
    return true;
    }
    // 6. Gắp vật

}
}
