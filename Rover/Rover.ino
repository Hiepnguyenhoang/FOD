#include <AFMotor.h>
#include <SoftwareSerial.h>

SoftwareSerial Serial5(50,51);
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

#define SENSOR_LEFTMOST 25
#define SENSOR_LEFT 24
#define SENSOR_CENTER 26
#define SENSOR_RIGHT 52
#define SENSOR_RIGHTMOST 53

#define trigPin 40
#define echoPin 42

int rePhaiMotLan = 0;
int quayVeTruBanDau = 0;
int quayVeTru = 0;
int dist;      // LiDAR actually measured distance value
int strength;  // LiDAR signal strength
int check;     // check numerical value storage
int i;
int uart[9];   // store data measured by LiDAR
const int HEADER = 0x59; // data package frame header
bool lidarActive = true;

int checkBattery = 0;

const float detectionThreshold = 10.0; 
bool objectDetected = false;

void setup() {
    Serial.begin(9600);
    Serial5.begin(115200); 
    Serial.println("Hello my racing car");


    // Motor setup
    motor1.setSpeed(150);
    motor2.setSpeed(150);
    motor3.setSpeed(150);
    motor4.setSpeed(150);

    pinMode(SENSOR_LEFTMOST, INPUT);
    pinMode(SENSOR_LEFT, INPUT);
    pinMode(SENSOR_CENTER, INPUT);
    pinMode(SENSOR_RIGHT, INPUT);
    pinMode(SENSOR_RIGHTMOST, INPUT);

    // HC-SR04 setup
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);


}

void loop() {
  if (checkBattery == 0) {
   // moveForward();
    delay(2000);
  } 
  delay(5);
  lidarSensor();

}

void lidarSensor() {
  if (lidarActive && Serial5.available()) { // check whether the serial port has data input
        if (Serial5.read() == HEADER) { // determine data package frame header 0x59
            uart[0] = HEADER;
            if (Serial5.read() == HEADER) { // determine data package frame header 0x59
                uart[1] = HEADER;
                for (i = 2; i < 9; i++) { // store data to array
                    uart[i] = Serial5.read();
                }
                check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
                if (uart[8] == (check & 0xff)) { // check the received data as per protocols
                    
                    dist = uart[2] + uart[3] * 256; // calculate distance value
                    strength = uart[4] + uart[5] * 256; // calculate signal strength value
                    checkBattery = 1;
                    int x = 0;
                    Serial.print("dist = ");
                    Serial.print(dist);//output measure distance value of LiDAR
                    Serial.print('\t');
                    Serial.print("strength = ");
                    Serial.print(strength);//output signal strength value
                    Serial.print('\n');
                    scanForObject();
                    if (dist > 20 && dist < 120) {
                        x = 1;
                        
                    }
                    if (x) {
                        stopMotors();
                        Serial.println("OPEN_CAM"); // Tín hiệu bật camera
                        Serial.println(dist);
                        if(dist>100 && dist <200){
                          Serial.println("zoom x2");
                        }
/*                        if(dist>=200 && dist <300){
                          Serial.println("zoom x3");
                        }
                        if(dist>=300 && dist <400){
                          Serial.println("zoom x4");
                        }
                        if(dist>=400 && dist <500){
                          Serial.println("zoom x5");
                        }*/
                        Serial.println("CLOSE_CAM"); // Tín hiệu tắt camera
                        lidarActive = false;
                        while (!lidarActive) {
                          stopMotors();
                          if (Serial.available()) { // Check for input from serial
                            
                            String command = Serial.readStringUntil('\n'); // Read command until newline
                            command.trim(); // Remove extra spaces
                            if (command.equalsIgnoreCase("OPEN_LIDAR")) { // If "person" command is received
            
                              lidarActive = true; // Activate LiDAR
                            }
                            if (command.equalsIgnoreCase("COLLECT_FOD")) { // If "person" command is received
                              pickUpObjects(dist);
                              lidarActive = true;
                            }

                          }

                        }
                                     
                        // Clear the Serial5 buffer to avoid residual data
                        while (Serial5.available() > 0) {
                            Serial5.read();
                        }
                    } 
                }
            }
        }
    }

}

void sensor5mat() {
  checkBattery = 1;
    int sensorValue = (digitalRead(SENSOR_LEFTMOST) << 4) |
                      (digitalRead(SENSOR_LEFT) << 3) |
                      (digitalRead(SENSOR_CENTER) << 2) |
                      (digitalRead(SENSOR_RIGHT) << 1) |
                      (digitalRead(SENSOR_RIGHTMOST));

    switch (sensorValue) {
      case 0b10101:
      case 0b10001:
          moveForward();
          break;

      case 0b01111: 
      case 0b10111: 
      case 0b00111: 
      case 0b00101: 
          turnLeft();
          break;
      case 0b11100: 
      case 0b10100: 
      case 0b11101: 
      case 0b11110: 
          turnRight();
          break;
      case 0b00000:
      case 0b00100:
          if (quayVeTruBanDau == 1) break;
          else{
          turnRight();
          break;
          }
     case 0b11111:
         if (quayVeTruBanDau == 1) {
          turnLeft();
          delay(1820);
          moveForward();
          delay(5000);
          stopMotors();
          checkBattery = 0;
          delay(100000);
         }
         if (quayVeTru == 0) {
         //moDuong();
         break;
         } else {
                if (rePhaiMotLan == 0) {
                turnRight();
                delay(3040);
                moveForward();
                delay(600);
               // Serial.println("Quay lai r");
                rePhaiMotLan = 1;
                }
                
                quayVeTruBanDau = 1;
                //Serial.println("Di ve sac");
                quayVeTru = 0;
                scanForObject();
                checkBattery = 1;
         }

      default:
          stopMotors();
          break;
    }
}
void moDuong() {
    while (true) {
        bool lineDetected = false;
        turnLeft();
        unsigned long startTime = millis();
        while (millis() - startTime < 200) { 
        checkObject();
               if (quayVeTru == 1 ) break;
            int sensorValue = (digitalRead(SENSOR_LEFTMOST) << 4) |
                              (digitalRead(SENSOR_LEFT) << 3) |
                              (digitalRead(SENSOR_CENTER) << 2) |
                              (digitalRead(SENSOR_RIGHT) << 1) |
                              (digitalRead(SENSOR_RIGHTMOST));
            if (sensorValue == 0b00111 || sensorValue == 0b10111 || sensorValue == 0b01111) {
                lineDetected = true;
                break;
            }
        }
        if (lineDetected) break;

        turnRight();
        startTime = millis();
        while (millis() - startTime < 200) { 
        checkObject();
        if (quayVeTru == 1 ) break;
            int sensorValue = (digitalRead(SENSOR_LEFTMOST) << 4) |
                              (digitalRead(SENSOR_LEFT) << 3) |
                              (digitalRead(SENSOR_CENTER) << 2) |
                              (digitalRead(SENSOR_RIGHT) << 1) |
                              (digitalRead(SENSOR_RIGHTMOST));
            if (sensorValue == 0b11110 || sensorValue == 0b11100 || sensorValue == 0b11101) {
                lineDetected = true;
                break;
            }
        }
        if (lineDetected) break;
        if (quayVeTru == 1 ) break;
    }

    stopMotors();
}

// Hàm điều khiển động cơ
void moveForward() {
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    motor1.run(FORWARD); 
    motor2.run(FORWARD); 
    motor3.run(FORWARD); 
    motor4.run(FORWARD); 
}

void moveBackward() {   
   motor1.setSpeed(150);
    motor2.setSpeed(150);
    motor3.setSpeed(150);
    motor4.setSpeed(150);
    motor1.run(BACKWARD); 
    motor2.run(BACKWARD); 
    motor3.run(BACKWARD); 
    motor4.run(BACKWARD); 
}

void turnLeft() {
    motor1.setSpeed(50);
    motor2.setSpeed(150);
    motor3.setSpeed(150);
    motor4.setSpeed(50);
    motor1.run(BACKWARD); 
    motor2.run(FORWARD); 
    motor3.run(FORWARD); 
    motor4.run(BACKWARD); 
}

void turnRight() {
    motor1.setSpeed(150);
    motor2.setSpeed(50);
    motor3.setSpeed(50);
    motor4.setSpeed(150);
    motor1.run(FORWARD); 
    motor2.run(BACKWARD); 
    motor3.run(BACKWARD); 
    motor4.run(FORWARD); 
}

void stopMotors() {
    motor1.run(RELEASE); 
    motor2.run(RELEASE); 
    motor3.run(RELEASE); 
    motor4.run(RELEASE); 
}

float getDistance() {
    long duration;
    float distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration * 0.034) / 2;

    return distance;
}
void scanForObject() {
    sensor5mat();
    checkObject();
}

void checkObject() {
  quayVeTru = 0;
    int count = 0;
    float distance = getDistance();
    if (distance <= detectionThreshold) {
       // Serial.println("Vật thể được phát hiện! Dừng chương trình.");
        objectDetected = true;
        stopMotors();
        while (objectDetected) {
            float recheckDistance = getDistance();
            Serial.print("Kiểm tra lại - Khoảng cách: ");
            Serial.print(recheckDistance);
            Serial.println(" cm");

            if (recheckDistance <= detectionThreshold) {
                Serial.println("Vật thể vẫn còn! Gửi tín hiệu...");
                stopMotors();
                delay(1000);
                count++;
                if (count >= 5) {
                  quayVeTru = 1;
                  break;
                }
                  
            } else {
                Serial.println("Vật thể không còn. Tiếp tục chương trình.");
                objectDetected = false;
            }
        }
    }
}

void pickUpObjects (int d) {
    // Nhận tín hiệu hút vật thể
    unsigned long time = 0;
    time = d*40; // 4s đi được 1m
    //Rẽ phải 90
    turnRight(); 
    delay(2000);
    // Đi đến nơi chỉ định
    moveForwardByDistance(time); 
    // Hút vật thể
    stopMotors();
    delay(2000);
    // Quay đầu xe 
    turnRight();
    delay(4000);
    moveForwardByDistance(time);
}

void moveForwardByDistance (unsigned long time) {
    motor1.setSpeed(150);
    motor2.setSpeed(150);
    motor3.setSpeed(150);
    motor4.setSpeed(150);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    delay(time);
}