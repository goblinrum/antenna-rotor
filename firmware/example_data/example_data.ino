// #include <WiFi.h>
// #include <HTTPClient.h>

unsigned char gps[64];
int count = 0; 

String sat_name = "ISS (ZARYA)";
char tle_1[70];
char tle_2[70];

int state = 2;

void setup() {
    // setup gps
    Serial1.begin(9600); // Serial for the GPS module, will need to define TX and RX for ESP32
    // setup motors here
    // set up serial connection to computer here
    Serial.begin(9600); // Starts the serial communication to the computer

}

void loop() {
    // Step 1: read the gps data (leave in acquring data mode until fix is found)
    // read_gps();
    // Step 2: 
    //  - Option 1: use the gps data to directly request API
    //  - Option 2: send GPS data to computer, and have computer request API
    if (state == 2)  process_tle();
    // Step 3: use the API data to calculate the position needed
    // uses library like this: https://github.com/Hopperpop/Sgp4-Library
    // this enters tracking mode
    // Step 4: send the motor data to the computer every 10 seconds
    if (state == 4) update_server();
}

void read_gps() {
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
}

void process_tle() {
  if (Serial.available()) {
    // read in a \n separated string and store it into 2 char arrays
    String input = Serial.readString();
    bool crossed = false; // if it is on the 2nd line
    for (int i = 0; i < input.length(); i++) {
      if (input[i] == "\n") crossed = true;
      if (!crossed) { // if it is on first line
        tle_1[i] = input[i];
      } else { // is on second line
        tle_2[i] = input[i];
      }
    }
    state = 4;
  }
}

void update_server() {
  if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      if (input == "get_coords") {
        Serial.println(gps[63] + "\n" + sat_name + "\n" + tle_1 + "\n" + tle_2); // Send actual coordinates here
      }
    }
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}