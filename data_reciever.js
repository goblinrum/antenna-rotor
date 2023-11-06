// WebSocket connection with the microcontroller
var socket = new WebSocket("ws://esp32_ip_addresss:81/");

// Logic that commences when connection is established
socket.onopen = function(event) {
    // Data handling 
    var data = JSON.parse(event.data);
    var x = data.x;
    var y = data.y;

    // add some stuff to update rendering
};

// Error message if connection isn't established
socket.onerror = function(event) {
    console.error("Websocket errort:", event);
};