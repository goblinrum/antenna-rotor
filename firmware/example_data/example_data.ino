// #include <WiFi.h>
// #include <HTTPClient.h>

unsigned char gps[64];
int count = 0; 

void setup() {
    // setup gps
    Serial1.begin(9600); // Serial for the GPS module, will need to define TX and RX for ESP32
    // setup motors here
    // set up serial connection to computer here
    Serial.begin(9600); // Starts the serial communication to the computer

}

void loop() {
    // Step 1: read the gps data
    if (Serial1.available()) {
      while (Serial1.available()) {
        gps[count++] = Serial1.read();
        if (count == 64) {
          break;
        }
      }
      Serial.write(gps, count);
      clearBuffer();
      count = 0;
    }
    // Step 2: 
    //  - Option 1: use the gps data to directly request API
    //  - Option 2: send GPS data to computer, and have computer request API
    // Step 3: use the API data to calculate the position needed
    // uses library like this: https://github.com/Hopperpop/Sgp4-Library
    // this enters tracking mode
    // Step 4: send the motor data to the computer every 10 seconds
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      if (input == "get_coords") {
        Serial.println("1,1,1,1"); // Send actual coordinates here
      }
    }
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}