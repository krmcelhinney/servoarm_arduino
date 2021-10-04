#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Wire.h"
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "Arduino.h"

const char * ssid = "TelstraCD9D6D";
const char * password = "62k6h9ycd73m";
const char * mqtt_server = "10.0.0.1";
const char * MQTT_USER = "ken";
const char * MQTT_PASSWORD = "jessica1";
String payload_str_old = "xxx";
float joint[7];
int servoDegree[6];
int initServos = 99;

// Define Motor Outputs on PCA9685 board
int motorBase = 0;
int motorSholder = 1;
int motorElbow = 2;
int motorWrist = 3;
int motorPivot = 4;
int motorJaws = 5;

// Define servo positions
double mtrDegreeBase;
double mtrDegreeSholder;
double mtrDegreeElbow;
double mtrDegreeWrist;
double mtrDegreePivot;
double mtrDegreeJaws;

WiFiClient espClient;
PubSubClient client(espClient);

//Setup wifi connection
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//MQTT message callback
void callback(char * topic, byte * payload, unsigned int length) {
  int i = 0;
  String payload_str = "";
  for (int i = 0; i < length; i++) {
    payload_str += (char) payload[i];
  }
  if (payload_str != payload_str_old) {
    int payload_len = payload_str.length() + 1;
    char payload_array[payload_len];
    payload_str.toCharArray(payload_array, payload_len);
    sscanf(payload_array, "%f, %f, %f, %f, %f, %f, %f", &joint[0], &joint[1], &joint[2], &joint[3], &joint[4], &joint[5], &joint[6]);
    mtrDegreeBase = trimLimits(radiansToDegrees(joint[0]) - 90);
    mtrDegreeSholder = trimLimits(radiansToDegrees(joint[1]));
    mtrDegreeElbow = trimLimits(radiansToDegrees(joint[2]) + 90);
    mtrDegreeWrist = trimLimits(radiansToDegrees(joint[3]) + 90);
    mtrDegreePivot = trimLimits(radiansToDegrees(joint[4]) + 180);
    mtrDegreeJaws = trimLimits(radiansToDegrees(joint[5]));

    Serial.print(mtrDegreeBase);
    Serial.print(", ");
    Serial.print(mtrDegreeSholder);
    Serial.print(", ");
    Serial.print(mtrDegreeElbow);
    Serial.print(", ");
    Serial.print(mtrDegreeWrist);
    Serial.print(", ");
    Serial.print(mtrDegreePivot);
    Serial.print(", ");
    Serial.println(mtrDegreeJaws);

    moveMotorDeg(mtrDegreeBase, motorBase);
    moveMotorDeg(mtrDegreeSholder, motorSholder);
    moveMotorDeg(mtrDegreeElbow, motorElbow);
    moveMotorDeg(mtrDegreeWrist, motorWrist);
    moveMotorDeg(mtrDegreePivot, motorPivot);
    moveMotorDeg(mtrDegreeJaws, motorJaws);
  }
  payload_str_old = payload_str;
}

//Maintain connection to MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...  ");
    // Create a random client ID
    String clientId = "Servoarm-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("servoarm/move");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(50);
}

// Function to move motor to specific position
void moveMotorDeg(int moveDegree, int motorOut) {
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}

// Convert radians to degreees
double radiansToDegrees(float position_radians) {
  position_radians = position_radians * 57.2958;
  return position_radians;
}

// Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
double trimLimits(double mtr_pos) {
  if (mtr_pos > 180) {
    mtr_pos = 180;
  }
  if (mtr_pos < 0) {
    mtr_pos = 0;
  }
  return mtr_pos;
}
