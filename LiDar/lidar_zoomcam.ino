#include <SoftwareSerial.h> // soft serial port header file

SoftwareSerial Serial1(2, 3); // define the soft serial port as Serial1, pin2 as RX, and pin3 as TX
/*For Arduino board with multiple serial ports such as DUE board, comment out the above two codes, and directly use
Serial1 port*/

int dist;      // LiDAR actually measured distance value
int strength;  // LiDAR signal strength
int check;     // check numerical value storage
int i;
int uart[9];   // store data measured by LiDAR
const int HEADER = 0x59; // data package frame header
bool lidarActive = true;

void setup() {
    Serial.begin(9600);       // set the Baud rate of Arduino and computer serial port
    Serial1.begin(115200);    // set the Baud rate of LiDAR and Arduino serial port
}

void loop() {
    if (lidarActive && Serial1.available()) { // check whether the serial port has data input
        if (Serial1.read() == HEADER) { // determine data package frame header 0x59
            uart[0] = HEADER;
            if (Serial1.read() == HEADER) { // determine data package frame header 0x59
                uart[1] = HEADER;
                for (i = 2; i < 9; i++) { // store data to array
                    uart[i] = Serial1.read();
                }
                check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
                if (uart[8] == (check & 0xff)) { // check the received data as per protocols
                    
                    dist = uart[2] + uart[3] * 256; // calculate distance value
                    strength = uart[4] + uart[5] * 256; // calculate signal strength value
                    
                    int x = 0;
                    if (dist > 5 && dist < 200) {
                        x = 1;
                    }
                    if (x) {
                        Serial.println("OPEN_CAM"); // Tín hiệu bật camera
                        Serial.println(dist);
                        if(dist>20 && dist <50){
                          Serial.println("zoom x2");
                        }
                        if(dist>=50 && dist <100){
                          Serial.println("zoom x3");
                        }
                        lidarActive = false;
                        delay(1000);
                        Serial.println("CLOSE_CAM"); // Tín hiệu tắt camera
                        
                        // Clear the serial1 buffer to avoid residual data
                        while (Serial1.available() > 0) {
                            Serial1.read();
                        }
                    } 
                }
            }
        }
    }
    if (Serial.available()) { // check if there is any input from the keyboard
        char inputChar = Serial.read(); // read the character
        if (inputChar != '\n' && inputChar != '\r') { // ignore line feed and carriage return
            lidarActive = true; // activate LiDAR
        }
    }
}
