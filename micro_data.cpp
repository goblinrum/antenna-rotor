#include <WebSocketsServer.h>

// declares a WebSocketServer object and initializes it on desired port
int port = 81; 
WebSocketsServer webSocket = WebSocket(port);

// Initializes and confiugres WebSocket server
void setup() {
    webSocket.begin();
}

// Continuously collect and send data
void loop() {
    // Wrote this as x, y values but will change to collect data from encoders
    int xValue = 1
    int yValue = 2
   
    // Create a JSON object containing the data
    String json = "{\"x\":" + String(xValue) + ",\"y\":" + String(yValue) + "}";

    // Send the data to connected clients
    webSocket.broadcastTXT(json);

    // Delay the data to prevent overload
    int delay = 1000;
    delay(delay);
}