#include "Wire.h"
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "Arduino.h"
#include "ArduinoHardware.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;
std_msgs::Int16MultiArray str_msg2;
ros::Publisher chatter("servoarm", &str_msg2);
int servoDegree[6];

// Define Motor Outputs on PCA9685 board
int motorBase = 0;
int motorSholder = 1;
int motorElbow = 2;
int motorWrist = 3;
int motorPivot = 4;
int motorJaws = 5;

// Define Motor position variables
double mtrDegreeBase;
double mtrDegreeSholder;
double mtrDegreeElbow;
double mtrDegreeWrist;
double mtrDegreePivot;
double mtrDegreeJaws;

// Function move motor to ROS angle
void servo_cb(const sensor_msgs::JointState& cmd_msg)
{
  mtrDegreeBase = radiansToDegrees(cmd_msg.position[0]) - 180;
  mtrDegreeSholder = radiansToDegrees(cmd_msg.position[1]) - 90;
  mtrDegreeElbow = radiansToDegrees(cmd_msg.position[2]) + 5;
  mtrDegreeWrist = radiansToDegrees(cmd_msg.position[3]);
  mtrDegreePivot = radiansToDegrees(cmd_msg.position[4]) +90;
  mtrDegreeJaws = radiansToDegrees(cmd_msg.position[5]);

  servoDegree[0] = mtrDegreeBase;
  servoDegree[1] = mtrDegreeSholder;
  servoDegree[2] = mtrDegreeElbow;
  servoDegree[3] = mtrDegreeWrist;
  servoDegree[4] = mtrDegreePivot;
  servoDegree[5] = mtrDegreeJaws;
  
  moveMotorDeg(mtrDegreeBase, motorBase);
  moveMotorDeg(mtrDegreeSholder, motorSholder);
  moveMotorDeg(mtrDegreeElbow, motorElbow);
  moveMotorDeg(mtrDegreeWrist, motorWrist);
  moveMotorDeg(mtrDegreePivot, motorPivot);
  moveMotorDeg(mtrDegreeJaws, motorJaws);
  }

  ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

  void setup()
  {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(chatter);
    // Setup PWM Controller object
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
  }

  // Function to move motor to specific position
  void moveMotorDeg(int moveDegree, int motorOut)
  {
    int pulse_wide, pulse_width;

    // Convert to pulse width
    pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

    //Control Motor
    pwm.setPWM(motorOut, 0, pulse_width);
  }

  void loop()
  {
    str_msg2.data = servoDegree;
    str_msg2.data_length = 6;
    chatter.publish( &str_msg2 );
    nh.spinOnce();
  }

  // Function to convert radians to degrees
  double radiansToDegrees(float position_radians)
  {
    position_radians = position_radians + 1.6;
    position_radians * 57.2958;
    if(position_radians < 0){
      position_radians = 0;
    }
    if(position_radians > 180){
      position_radians = 180;
    }
    return position_radians;
  }
