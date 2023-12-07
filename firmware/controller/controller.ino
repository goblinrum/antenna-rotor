#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Time.h>
#include <Sgp4.h>
#include <SPI.h>
#include <Ticker.h>
#include <Wire.h>
#include <string.h>
#include "AS5600.h"
#include "time.h"
#include "esp_sntp.h"
#include <PID_v1.h>

// interactive UI
#define LED 2
#define BUTTON_F 23 // button to move vertical arm
#define BUTTON_R 34 // button to move horizontal arm
#define BUTTON_SEL 35 // button to select

// user Serial 2 for GPS
#define RXD2 16
#define TXD2 17

// motor definitions
#define IN1 32
#define IN2 33
#define PWM_V 25

#define IN3 5
#define IN4 19
#define PWM_H 18

// encoder definitions
AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

// encoder readings
double offsetH = 0;
double offsetV = 0;
double adjustedAngleH = 0;
double desiredAngleH = 0;
double motorOutputH = 0;
double adjustedAngleV = 0;
double desiredAngleV = 0;
double motorOutputV = 0;

// PID coefficients
double kp = 5;     // Proportional coefficient
double ki = 0.1;   // Integral coefficient
double kd = 0.05;  // Derivative coefficient
PID PIDControllerH(&adjustedAngleH, &motorOutputH, &desiredAngleH, kp, ki, kd, DIRECT);
PID PIDControllerV(&adjustedAngleV, &motorOutputV, &desiredAngleV, kp, ki, kd, DIRECT);

// screen definitions
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS definitons
char gps[64];
int count = 0;

// processing gps data
char *token;
const char delimiter[2] = ",";

// positional arguments
float lat = 37.8716;        // latitude
float lon = -122.2728;        // longitude
float alt = 140;        // altutide
float azi = 0;        // azimuth
float ele = 0;        // elevation
float satLat = 0;     // satellite lat
float satLon = 0;     // satellite long
float satAlt = 0;     // satellite altitude
bool satVis = false;  // satellite visibility

// TLE data
char satname[70];
char tle_1[70];
char tle_2[70];

// SGP4 information
Sgp4 sat;
Ticker tkSecond;
unsigned long unixtime = 0;
int timezone = -8;  //utc - 8
int framerate;
int year;
int mon;
int day;
int hr;
int minute;
double sec;
bool done_setup = false;

//ESP32Time rtc;
ESP32Time rtc(-3600 * 8);

// current state machine state enum
enum { PROCESS_TLE,
       PREDICT,
       TRACK_MOTOR,
       UPDATE_SERVER };
unsigned char state;

// Define an enumeration for motor identification
enum Motor { MOTOR_V,
             MOTOR_H };

void setup() {
  // set up serial connection to computer here
  Serial.begin(9600);
  // setup gps serial
  Serial2.begin(9600);
  // setup screen
  setup_screen();
  // setup buttons
  pinMode(BUTTON_F, INPUT);
  pinMode(BUTTON_R, INPUT);
  pinMode(BUTTON_SEL, INPUT);
  // setup LED
  pinMode(LED, OUTPUT);
  // setup GPS
  // setup_gps();
  // setup encoders
  setup_encoders();
  // calibrate encoders
  calibrate_encoders();
}

/*
State machine management on loop 
*/
void loop() {
  // if button is pressed, go back to process_tle state (interrupts other states and updates the db)
  if (digitalRead(BUTTON_SEL) == HIGH) {
    state = PROCESS_TLE;
  }

  switch (state) {
    // case 1: wait to receive TLE data from computer
    case PROCESS_TLE: process_tle(); break;

    // Step 2: use the API data to calculate the position needed
    // uses library like this: https://github.com/Hopperpop/Sgp4-Library
    // this enters tracking mode
    case PREDICT: predict(); break;

    // Step 3: Actually move the motors to the desired azimuth/elevation
    case TRACK_MOTOR: track_motor(); break;

    // Step 5: send the motor data to the computer
    case UPDATE_SERVER: update_server(); break;

    default: break;
  }
}

/* 
Read from GPS until a fix is acquired. 
Transitions into read_gps()
*/
void setup_gps() {
  // read from GPS
  bool fix = false;
  if (Serial2.available()) {
    while (Serial2.available()) {
      // read one line at a time until a valid GPRMC line is found and "V" is not in the line
      String line = Serial2.readStringUntil('\n');
      delay(1000);
      if (!fix && line.indexOf("GPRMC") != -1 && line.indexOf("V") == -1) {
        fix = true;
      } else if ((fix && line.indexOf("GPGGA") != -1) || (line.indexOf("GPRMC") != -1 && line.indexOf(",,,,") == -1) || (line.indexOf("GPGGA") != -1 && line.indexOf(",,,") == -1)) {
        clearBuffer();
        line.toCharArray(gps, 64);
        read_gps();
        state = PROCESS_TLE;
        break;
      }
    }
  }
}

/* 
Process the TLE from the computer and saves it
Transitons to predict()
*/
void process_tle() {
  if (Serial.available()) {
    String line1 = Serial.readStringUntil('\n');
    String line2 = Serial.readStringUntil('\n');
    String line3 = Serial.readStringUntil('\n');
    String line4 = Serial.readStringUntil('\n');
    line1.toCharArray(satname, 70);
    line2.toCharArray(tle_1, 70);
    line3.toCharArray(tle_2, 70);
    // line 4 is a number with current unix time
    unixtime = line4.toInt();

    // set the RTC
    rtc.setTime(unixtime);

    displayPrint(String(unixtime));
    state = PREDICT;
  }
}

/* 
Performs first time setup if not done yet, then loops to do the actual prediction
*/
void predict() {
  if (done_setup) {  // predict based on new unixtime
    sat.findsat(rtc.getEpoch());
    setDesiredAngle(azi, ele);
    framerate += 1;
    state = TRACK_MOTOR;
  } else {  // first time setup
    sat.site(lat, lon, alt);
    sat.init(satname, tle_1, tle_2);
    double jdC = sat.satrec.jdsatepoch;
    invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
    tkSecond.attach(1, Second_Tick);
    done_setup = true;
  }
}

/* 
Moves the motor to input azimuth and elevation
1. Gets the current angle
2. Calculates the delta between the current angle and the desired angle
3. Moves the motor to the desired angle
*/
void track_motor() {
  displayPrint("Tracking...");

  getAngle();  // Update the current angle of the motors

  // Adjusting Horizontal Motor
  adjustMotor(MOTOR_V, desiredAngleH, adjustedAngleH, motorOutputH);

  // Adjusting Vertical Motor
  adjustMotor(MOTOR_H, desiredAngleV, adjustedAngleV, motorOutputV);

  // state = UPDATE_SERVER;
  state = PREDICT;
}

/*
Updates the server on the new position and coordinates 
*/
void update_server() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "get_coords") {
      // send satellite coordinates
      Serial.println(String(satLat) + "," + String(satLon) + "," + String(satAlt) + "," + String(azi) + "," + String(ele));
    }
  }
  state = PREDICT;
}

////////////////////// Utility Functions //////////////////////

/* 
Example prediction function taken from: 
https://github.com/Hopperpop/Sgp4-Library/blob/master/examples/Sgp4Tracker/Sgp4Tracker.ino
*/
void Second_Tick() {
  unixtime += 1;

  invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec);

  azi = sat.satAz;
  ele = sat.satEl;
  satLat = sat.satLat;
  satLon = sat.satLon;
  satAlt = sat.satAlt;
  satVis = sat.satVis;

  framerate = 0;
}

/* 
Uses the GPS data and parses the $GPGGA field into
latitude, longitude, and altidude

Transitions into process_tle()
*/
void read_gps() {
  token = strtok(gps, delimiter);  // Initial tokenization
  char *end;
  for (int i = 0; i < 12; i++) {  // Loop to reach the required tokens
    token = strtok(NULL, delimiter);
    if (i == 1 || i == 2) {                                            // Latitude
      if (strstr(token, "S") == NULL && strstr(token, "N") == NULL) {  // if it's a number
        lat = strtof(token, &end);
        lat /= 100;                             // convert to degrees
      } else if (strstr(token, "S") != NULL) {  // if it's in the Southern Hemisphere
        lat *= -1;
      }
    } else if (i == 3 || i == 4) {                                     // Longitude
      if (strstr(token, "W") == NULL && strstr(token, "E") == NULL) {  // if it's a number
        lon = strtof(token, &end);
        lon /= 100;                             // convert to degrees
      } else if (strstr(token, "W") != NULL) {  // if it's in the West
        lon *= -1;
      }
    } else if (i == 8 || i == 9) {
      if (strstr(token, "M") == NULL) {  // if it's a number
        alt = strtof(token, &end);
      }
    }
  }
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}

void setup_screen() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println("Welcome");
  display.display();
}

void calibrate_encoders() {
  // stay in this loop until the button is pressed
  // use BUTTON_F and BUTTON_R to move the motor forward and backward
  digitalWrite(LED, HIGH);
  displayPrint("Calibrating elevation motor...");
  while (digitalRead(BUTTON_SEL) == LOW) {
    if (digitalRead(BUTTON_F) == HIGH) {
      move_motor(MOTOR_V, 45);
    } else if (digitalRead(BUTTON_R) == HIGH) {
      move_motor(MOTOR_V, -45);
    } else {
      move_motor(MOTOR_V, 0);
    }
  }
  delay(500);
  displayPrint("Calibrating azimuth motor...");
  while (digitalRead(BUTTON_SEL) == LOW) {
    if (digitalRead(BUTTON_F) == HIGH) {
      move_motor(MOTOR_H, 40);
    } else if (digitalRead(BUTTON_R) == HIGH) {
      move_motor(MOTOR_H, -40);
    } else {
      move_motor(MOTOR_H, 0);
    }
  }
  offsetH = as5600_0.rawAngle() * AS5600_RAW_TO_DEGREES;
  offsetV = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
  digitalWrite(LED, LOW);
  displayPrint("Calibration complete!");
}

void setup_encoders() {  
  // 2 I2C buses
  Wire.begin(14, 13);
  Wire1.begin(26, 27);

  as5600_0.begin(4);  // (SDA, SCL)
  as5600_0.setDirection(AS5600_CLOCK_WISE);
  as5600_1.begin(5);  // (SDA, SCL)
  as5600_1.setDirection(AS5600_CLOCK_WISE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_V, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_H, OUTPUT);
}

void move_motor(Motor motor, int speed) {
  if (motor == MOTOR_V) {
    if (speed > 0) {
      Motor_Forward(IN1, IN2, PWM_V, speed);
    } else if (speed < 0) {
      Motor_Backward(IN1, IN2, PWM_V, -speed);
    } else {
      Motor_Brake(IN1, IN2);
    }
  } else if (motor == MOTOR_H) {
    if (speed > 0) {
      Motor_Forward(IN3, IN4, PWM_H, speed);
    } else if (speed < 0) {
      Motor_Backward(IN3, IN4, PWM_H, -speed);
    } else {
      Motor_Brake(IN3, IN4);
    }
  }
}

void Motor_Forward(int pinIN1, int pinIN2, int pinPWM, int Speed) {
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  analogWrite(pinPWM, Speed);
  delay(200);
}

void Motor_Backward(int pinIN1, int pinIN2, int pinPWM, int Speed) {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  analogWrite(pinPWM, Speed);
  delay(200);
}

void Motor_Brake(int pinIN1, int pinIN2) {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  delay(200);
}

void getAngle() {
  adjustedAngleH = as5600_0.rawAngle() * AS5600_RAW_TO_DEGREES - offsetH;
  adjustedAngleV = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES - offsetV;
  if (adjustedAngleH < 0) {
    adjustedAngleH += 360;
  } else if (adjustedAngleH >= 360) {
    adjustedAngleH -= 360;
  }
  if (adjustedAngleV < 0) {
    adjustedAngleV += 360;
  } else if (adjustedAngleV >= 360) {
    adjustedAngleV -= 360;
  }
}

void adjustMotor(Motor motor, double &desiredAngle, double &adjustedAngle, double &motorOutput) {
  // Calculate the shortest path to the desired angle
  double delta = desiredAngle - adjustedAngle;
  // Adjust delta for the shortest path considering 270 degrees range
  delta = fmod(delta + 405, 270) - 135;

  // If delta is within the leeway, stop the motor
  if (abs(delta) <= 5) {
    move_motor(motor, 0);
    return;
  }

  // Update the PID Controller
  if (motor == MOTOR_V) {
    PIDControllerH.SetTunings(kp, ki, kd);    // Make sure the PID tunings are set
    PIDControllerH.SetOutputLimits(-50, 50);  // Constrain the output to -60/60
    PIDControllerH.SetMode(AUTOMATIC);        // Set the PID to automatic mode

    // Update the PID controller with the current angle and desired angle
    PIDControllerH.Compute();

    // Use the output from the PID to move the motor
    move_motor(motor, motorOutputH);
  } else {
    PIDControllerV.SetTunings(kp, ki, kd);    // Make sure the PID tunings are set
    PIDControllerV.SetOutputLimits(-50, 50);  // Constrain the output to -60/60
    PIDControllerV.SetMode(AUTOMATIC);        // Set the PID to automatic mode

    // Update the PID controller with the current angle and desired angle
    PIDControllerV.Compute();

    // Use the output from the PID to move the motor
    move_motor(motor, motorOutputV);
  }
}

void displayPrint(String text) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(text);
  display.display();
}

void setDesiredAngle(double azimuth, double elevation) {
  // sets azimuth and elevation to desired angle
  desiredAngleH = azimuth;
  desiredAngleV = elevation;
}
