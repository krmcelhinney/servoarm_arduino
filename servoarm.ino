
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define Potentiometer Inputs

int controlBase = A0;
int controlSholder = A1;
int controlElbow = A2;
int controlWrist = A3;
int controlPivot = A6;
int controlJaws = A7;

int potSmooth = 1;

// Define Motor Outputs on PCA9685 board
int motorBase = 0;
int motorSholder = 1;
int motorElbow = 2;
int motorWrist = 3;
int motorPivot = 4;
int motorJaws = 5;

// Define Motor position variables
int mtrDegreeBase;
int mtrDegreeSholder;
int mtrDegreeElbow;
int mtrDegreeWrist;
int mtrDegreePivot;
int mtrDegreeJaws;

int oldMtrDegreeBase = 0;
int oldMtrDegreeSholder = 0;
int oldMtrDegreeElbow = 0;
int oldMtrDegreeWrist = 0;
int oldMtrDegreePivot = 0;
int oldMtrDegreeJaws = 0;

void setup() 
{
  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
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

// Function to convert potentiometer position into servo angle
int getDegree(int controlIn)
{
  int potVal,srvDegree;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Calculate angle in degrees
  srvDegree = map(potVal, 0, 1023, 0, 180);
  
  // Return angle in degrees
  return srvDegree;
  
}

void loop() {

  //Control Base Motor
  // Get desired position
  mtrDegreeBase = getDegree(controlBase);
  if(mtrDegreeBase>(oldMtrDegreeBase + potSmooth) || mtrDegreeBase<(oldMtrDegreeBase - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreeBase,motorBase);
  }
  oldMtrDegreeBase = mtrDegreeBase;

  //Control Sholder Motor 
  // Get desired position
  mtrDegreeSholder = getDegree(controlSholder);
  if(mtrDegreeSholder>(oldMtrDegreeSholder + potSmooth) || mtrDegreeSholder<(oldMtrDegreeSholder - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreeSholder,motorSholder);
  }
  oldMtrDegreeSholder = mtrDegreeSholder;

  //Control Elbow Motor
  // Get desired position
  mtrDegreeElbow = getDegree(controlElbow);
  if(mtrDegreeElbow>(oldMtrDegreeElbow + potSmooth) || mtrDegreeElbow<(oldMtrDegreeElbow - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreeElbow,motorElbow);
  }
  oldMtrDegreeElbow = mtrDegreeElbow;
  
  //Control Wrist Motor
  // Get desired position
  mtrDegreeWrist = getDegree(controlWrist);
  if(mtrDegreeWrist>(oldMtrDegreeElbow + potSmooth) || mtrDegreeElbow<(oldMtrDegreeElbow - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreeWrist,motorWrist);
  }
  oldMtrDegreeWrist = mtrDegreeWrist;    
  
  //Control Pivot Motor
  // Get desired position
  mtrDegreePivot = getDegree(controlPivot);
  if(mtrDegreePivot>(oldMtrDegreePivot + potSmooth) || mtrDegreePivot<(oldMtrDegreePivot - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreePivot,motorPivot);
  }
  
  //Control Jaws Motor
  // Get desired position
  mtrDegreeJaws = getDegree(controlJaws);
  if(mtrDegreeJaws>(oldMtrDegreeJaws + potSmooth) || mtrDegreeJaws<(oldMtrDegreeJaws - potSmooth)){
    // Move motor
    moveMotorDeg(mtrDegreeJaws,motorJaws);
  }
  oldMtrDegreeJaws = mtrDegreeJaws;
  
  // Add short delay
  delay(20);
}
