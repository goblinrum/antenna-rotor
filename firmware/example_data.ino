#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = ""; // Replace with your Wi-Fi network name
const char* password =  ""; // Replace with your Wi-Fi network password

const char* apiEndpoint = ""; // API endpoint

void setup() {
    // setup gps
    // setup motors here
    // set up serial connection to computer here
    Serial.begin(115200); // Starts the serial communication
    WiFi.begin(ssid, password); // Connect to Wi-Fi

    while (WiFi.status() != WL_CONNECTED) { // Wait for the connection to establish
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    void query_api();
}

void loop() {
    // Step 1: read the gps data
    // Step 2: 
    //  - Option 1: use the gps data to directly request API
    //  - Option 2: send GPS data to computer, and have computer request API
    // Step 3: use the API data to control the motors
    // this enters tracking mode
    // Step 4: send the motor data to the computer every 10 seconds
}

void query_api() {
    // option to query API here
    HTTPClient http;  // Object of class HTTP

    http.begin(apiEndpoint); // Specify request destination
    int httpCode = http.GET(); // Send the request

    if (httpCode > 0) { // Check the returning code
        String payload = http.getString(); // Get the request response payload
        Serial.println(httpCode); // Print HTTP return code
        Serial.println(payload); // Print request response payload
    } else {
        Serial.println("Error on HTTP request");
    }

    http.end(); // Free resources
}