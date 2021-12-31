//Teensy Flight Controller- Avian Tail Program- Science Fair Project
//Author- Kevin Shen
//Project Start- 10/26/2021

//Instructions for use

//1: Wipe SD card
//2: Place MPU6050 upright (plane upside down, to calibrate. when plane is level, pitch and roll are 0, when rolling right roll is positive,
//pitch up when pitch is positive, when yaw right yaw is positive)
//3: plug in SD card
//4: turn on & reset Teensy
//5: fly
//6: power off teensy
//7: unplug SD card from MAV
//8: plug in SD card to computer

//Credits

//Parts of code structure inspried by Nicholas Rehm
//IMU Filter designed by Nicholas Rehm
//Github: https://github.com/nickrehm/dRehmFlight

/*
 * Nicholas Rehm
 * Department of Aerospace Engineering
 * University of Maryland
 * College Park 20742
 * Email: nrehm@umd.edu
 */

//Used MPU6050 Library and I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//https://github.com/jrowberg/i2cdevlib

//Used standard Arduino SD Library
//https://www.arduino.cc/en/reference/SD

//PWM signal code structure inspired by
//https://www.camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino/

//libraries
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

//SD and MPU classes
File file;
MPU6050 mpu;

//pin definition
#define INTERRUPT_PIN 9
#define RX1 2
#define RX2 3
#define RX3 4
#define RX4 5
#define RX5 6
#define RX6 7

#define SERVO1PIN 23
#define SERVO2PIN 22
#define SERVO3PIN 1
#define SERVO4PIN 8
#define SERVO5PIN 15
#define SERVO6PIN 14

//servo class
Servo elevatorServo;
Servo rotatorServo;
Servo rightElevonServo;
Servo leftElevonServo;

//configurations
const bool flyingWing = false;
const bool fullBorb = false;
const bool noSpread = true;
const bool diffThrust = false;

//deadzone parameter
const float deadZone = 10;

//initalize servo outputs
float elevatorServoOutput = 90;
float rotatorServoOutput = 90;
float rightElevonServoOutput = 90;
float leftElevonServoOutput = 90;

//servo trim
int elevatorServoOutputTrim = 0;
int rotatorServoOutputTrim = 0;
int rightElevonServoOutputTrim = 0;
int leftElevonServoOutputTrim = 0;

//define servo pins
int elevatorServoPin = SERVO1PIN;
int rotatorServoPin = SERVO2PIN;
int rightElevonServoPin = SERVO3PIN;
int leftElevonServoPin = SERVO4PIN;

//initalize reciever input values
float RCpitch;
float RCyaw;
float RCroll;
float MODE;

//define reciever pins
int RCpitchInputPin = RX3;
int RCyawInputPin = RX2;
int RCrollInputPin = RX4;
int MODEInputPin = RX1;

//initialize tail variables
float tailElevonOffset = 0;      //variable to keep track of elevon offset caused by the tail
float optimumRotatorServoOutput; //variable to keep track of optimum rotator servo position
bool isOptimum = true;

//dampener values to scale servo outputs
float pitchDampener = 3;
float elevonDampener = 2;
float tailElevonOffsetDampener = 2;

//timekeeping/datalog varaibles
int iteration = 0;
int SDiteration = 0;
int SDdataLogFrequency = 200;
bool dataLog = false;

//initialize attitude variables
float yaw = 0;
float prevYaw = 0;
float yawChange = 0; //yaw change is simply a visual, not an exact measurement- absolute yaw is, however
float yawChangeMultiplier = 10;
float pitch = 0;
float roll = 0;

//PID controller variables

const float PitchPgain = 1;
const float PitchIgain = 0;
const float PitchDgain = 1;

float PitchProportional;
float PitchIntegral;
float PitchDerivative;
float PitchError;
float PrevPitchError;

float PitchOutput;

float prevPitch;
float pitchChange;
float pitchChangeMultiplier = 10;

// MPU6050 CODE- UNORIGINAL CODE ================================================================

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

//interrput detection
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void MPU6050Setup()
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

//FUNCTIONS ==============================================================================================================

//PWM function
volatile unsigned long PWMTimerStartPitch;
volatile int PWMLastInterruptTimePitch;

volatile unsigned long PWMTimerStartYaw;
volatile int PWMLastInterruptTimeYaw;

volatile unsigned long PWMTimerStartRoll;
volatile int PWMLastInterruptTimeRoll;

volatile unsigned long PWMTimerStartMODE;
volatile int PWMLastInterruptTimeMODE;

//function to read PWM using interrupt pins
void PWMSignalCalculator(float *channel, int pinNum, volatile int *lastInterruptTime, volatile unsigned long *timerStart)
{
  //record the interrupt time
  *lastInterruptTime = micros();

  //start timer when detect input
  if (digitalRead(pinNum) == HIGH)
  {
    *timerStart = micros();
  }
  else
  {
    if (*timerStart != 0)
    {
      //record the time between the square wave
      *channel = map(((volatile int)micros() - *timerStart), 1100, 1900, 90, -90);
      //reset timer
      *timerStart = 0;
    }
  }
}

//functions that have no parameter and call the main PWM reading function- used as a parameter for the attachInterrupt() function
void PWMSignalCalculatorPitch()
{
  PWMSignalCalculator(&RCpitch, RCpitchInputPin, &PWMLastInterruptTimePitch, &PWMTimerStartPitch);
}
void PWMSignalCalculatorYaw()
{
  PWMSignalCalculator(&RCyaw, RCyawInputPin, &PWMLastInterruptTimeYaw, &PWMTimerStartYaw);
}
void PWMSignalCalculatorRoll()
{
  PWMSignalCalculator(&RCroll, RCrollInputPin, &PWMLastInterruptTimeRoll, &PWMTimerStartRoll);
}
void PWMSignalCalculatorMODE()
{
  PWMSignalCalculator(&MODE, MODEInputPin, &PWMLastInterruptTimeMODE, &PWMTimerStartMODE);
  //mode used for datalog, chagned to 0 and 1 in this function
  if (MODE < 0)
  {
    dataLog = true;
  }
  else
  {
    dataLog = false;
  }
}

//radian function converts degrees to radians
float radian(float input)
{
  return input * (3.1416 / 180);
}

//function to calculate tail movement
void tailMovement()
{
  if (PitchOutput == 0)
  {
    PitchOutput++; //no divide by 0
  }
  optimumRotatorServoOutput = map(degrees(atan(RCyaw / PitchOutput)), 90, -90, 180, 0);
  //deadzone- if yaw force is real small and pitch is close to 0 then set yaw to 0
  if (RCyaw > -deadZone && RCyaw < deadZone && PitchOutput < deadZone && PitchOutput > -deadZone)
  {
    optimumRotatorServoOutput = 90;
  }
  isOptimum = true;
  if (optimumRotatorServoOutput < 45)
  {
    isOptimum = false;
  }
  else if (optimumRotatorServoOutput > 135)
  {
    isOptimum = false;
  }
  rotatorServoOutput = constrain(optimumRotatorServoOutput, 45, 135);

  if (isOptimum)
  {
    elevatorServoOutput = 90 + (PitchOutput / (cos(radian(rotatorServoOutput - 90))));
    rotatorServoOutput = 180 - rotatorServoOutput;
    tailElevonOffset = 0;
  }
  else
  {
    //deflect servo to the point that we actually get correct yaw output (0.707 is the cos(45 deg))
    elevatorServoOutput = 90 - abs(RCyaw / 0.707);

    //figure out the extra pitch (pitch generated - pitch required (stabilzed pitch).
    //pitch generated is tan(45 deg) times  yaw force, tan (45 deg) is 1, so pitch generated = RCyaw force generated.
    tailElevonOffset = (abs(PitchOutput) - abs(RCyaw)) / tailElevonOffsetDampener;
  }
  elevatorServoOutput = ((elevatorServoOutput - 90) / pitchDampener) + 90;

  elevatorServoOutput = constrain(elevatorServoOutput + elevatorServoOutputTrim, 0, 180);
  rotatorServoOutput = constrain(rotatorServoOutput + rotatorServoOutputTrim, 0, 180);
}

//function that mixes pitch and roll into elevon movmements
void justElevons()
{
  rightElevonServoOutput = ((RCpitch + RCroll) / elevonDampener) + 90;
  leftElevonServoOutput = (((0 - RCpitch) + RCroll) / elevonDampener) + 90;
}

//function that uses elevons with the tail- mostly just ailerons but can adjust CP of wing for the tail
void elevonWithTail()
{
  rightElevonServoOutput = ((RCroll) / elevonDampener) - tailElevonOffset + 90;
  leftElevonServoOutput = ((RCroll) / elevonDampener) + tailElevonOffset + 90;

  rightElevonServoOutput = constrain(rightElevonServoOutput + rightElevonServoOutputTrim, 0, 180);
  leftElevonServoOutput = constrain(leftElevonServoOutput + leftElevonServoOutputTrim, 0, 180);
}

//writes signal to actuators
void write()
{
  elevatorServo.write(elevatorServoOutput + elevatorServoOutputTrim);
  rotatorServo.write(rotatorServoOutput + rotatorServoOutputTrim);
  rightElevonServo.write(rightElevonServoOutput + rightElevonServoOutputTrim);
  leftElevonServo.write(leftElevonServoOutput + leftElevonServoOutputTrim);
}

//outputs to serial monitor, only used for testing and debugging
void serialOutput()
{

  // Serial.print(yaw);
  // Serial.print("\t");
  // Serial.print(pitch);
  // Serial.print("\t");
  // Serial.print(roll);
  // Serial.print("\t");

  // Serial.print("  isOptimum: ");
  // Serial.print(isOptimum);
  Serial.print("  RCyaw: ");
  Serial.print(RCyaw);
  Serial.print("  RCpitch: ");
  Serial.print(RCpitch);
  Serial.print(" RCroll: ");
  Serial.println(RCroll);
  // Serial.print("  tailElevonOffset: ");
  // Serial.print(tailElevonOffset);
  // Serial.print("  elevator: ");
  // Serial.print(elevatorServoOutput);
  // Serial.print("  rotator: ");
  // Serial.print(rotatorServoOutput);
  // Serial.print("  right elevon: ");
  // Serial.print(rightElevonServoOutput);
  // Serial.print("  left elevon: ");
  // Serial.print(leftElevonServoOutput);
}

//future goal; to be able to change the spread of the tail
void spreadCalc()
{
  //to be implemented
}

void tailAdjustForSpread()
{
  //to be implemented
}

//function to initialize SD read write
void SDSetup()
{

  Serial.print("Initializing SD card...");

  if (!SD.begin(10))
  {
    Serial.println("initialization failed!");
    while (1)
      ;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  file = SD.open("flight_data.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (file)
  {
    Serial.print("Writing to flight_data.txt...");

    file.print("iteration");
    file.print("\t");

    file.print("yaw");
    file.print("\t");
    file.print("yawChange");
    file.print("\t");
    file.print("pitch");
    file.print("\t");
    file.print("roll");
    file.print("\t");

    file.print("RCyaw");
    file.print("\t");
    file.print("RCpitch");
    file.print("\t");
    file.print("RCroll");
    file.print("\t");
    file.print("dataLog");
    file.print("\t");

    file.print("isOptimum");
    file.print("\t");

    file.print("elevatorServoOutput");
    file.print("\t");
    file.print("rotatorServoOutput");
    file.print("\t");
    file.print("rightElevonServoOutput");
    file.print("\t");
    file.print("leftElevonServoOutput");
    file.println("\t");

    // close the file:
    file.close();
    Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

//read IMU and convert to roll pitch yaw
void mpu6050Input()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    prevYaw = yaw;
    prevPitch = pitch;

    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[2] * 180 / M_PI;
    roll = ypr[1] * 180 / M_PI;

    //INVERT
    if (pitch < 0)
    {
      pitch = 180 + pitch;
    }
    else if (pitch >= 0)
    {
      pitch = pitch - 180;
    }
    if (roll < 0)
    {
      roll = 180 + roll;
    }
    else if (roll >= 0)
    {
      roll = roll - 180;
    }
    yaw = 0 - yaw;
    yawChange = (yaw - prevYaw) * yawChangeMultiplier;
    pitchChange = (pitch - prevPitch) * pitchChangeMultiplier;

    if (yawChange >= 300 * yawChangeMultiplier || yawChange <= -300 * yawChangeMultiplier)
    { //when yaw reaches 180 and goes to -180,
      yawChange = 0;
    }

    if (pitchChange >= 300 * pitchChangeMultiplier || pitchChange <= -300 * pitchChangeMultiplier)
    { //when pitch reaches 180 and goes to -180,
      pitchChange = 0;
    }
  }
}

//write flight data to SD card
void SDOutput()
{
  if (SDiteration >= SDdataLogFrequency)
  {
    file = SD.open("flight_data.txt", FILE_WRITE);

    file.print(iteration);
    file.print("\t");

    file.print(yaw);
    file.print("\t");
    file.print(yawChange);
    file.print("\t");
    file.print(pitch);
    file.print("\t");
    file.print(roll);
    file.print("\t");

    file.print(RCyaw);
    file.print("\t");
    file.print(RCpitch);
    file.print("\t");
    file.print(RCroll);
    file.print("\t");
    file.print(dataLog * 100); //datalog and isOptimum multiplied by 100 to make it easier to see on graph
    file.print("\t");

    file.print(isOptimum * 100);
    file.print("\t");

    file.print(elevatorServoOutput);
    file.print("\t");
    file.print(rotatorServoOutput);
    file.print("\t");
    file.print(rightElevonServoOutput);
    file.print("\t");
    file.print(leftElevonServoOutput);
    file.println("\t");

    file.close();

    SDiteration = 0;
  }
}

//keep track of iterations
void timekeeper()
{
  iteration++;   //to display iteration
  SDiteration++; //to make sure SD card outputs at correct time
}

void PitchPID()
{

  PrevPitchError = PitchError;

  PitchError = RCpitch - (pitchChange * pitchChangeMultiplier);

  PitchProportional = PitchError * PitchPgain;

  if (PitchIntegral <= 80 && PitchIntegral >= -80)
  {
    PitchIntegral += PitchError * PitchIgain;
  }

  PitchDerivative = (PitchError - PrevPitchError) * PitchDgain;

  PitchOutput = PitchProportional + PitchIntegral + PitchDerivative;
}

//VOID SETUP ====================================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(RCpitchInputPin, INPUT);
  pinMode(RCyawInputPin, INPUT);
  pinMode(RCrollInputPin, INPUT);
  pinMode(MODEInputPin, INPUT);

  pinMode(elevatorServoPin, OUTPUT);
  pinMode(rotatorServoPin, OUTPUT);
  pinMode(rightElevonServoPin, OUTPUT);
  pinMode(leftElevonServoPin, OUTPUT);

  PWMTimerStartPitch = 0;
  PWMTimerStartRoll = 0;
  PWMTimerStartYaw = 0;
  PWMTimerStartMODE = 0;
  attachInterrupt(digitalPinToInterrupt(RCpitchInputPin), PWMSignalCalculatorPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCrollInputPin), PWMSignalCalculatorRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCyawInputPin), PWMSignalCalculatorYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MODEInputPin), PWMSignalCalculatorMODE, CHANGE);

  elevatorServo.attach(elevatorServoPin);
  rotatorServo.attach(rotatorServoPin);
  rightElevonServo.attach(rightElevonServoPin);
  leftElevonServo.attach(leftElevonServoPin);

  SDSetup();
  MPU6050Setup();
}

//MAIN LOOP ========================================================================================

void loop()
{
  timekeeper();
  if (flyingWing)
  {
    justElevons();
  }
  else if (fullBorb)
  {
    spreadCalc();
    tailMovement();
    tailAdjustForSpread();
  }
  else if (noSpread)
  {
    PitchPID();
    tailMovement();
    elevonWithTail();
  }
  write();
  //  serialOutput();
  mpu6050Input();
  SDOutput();
}
