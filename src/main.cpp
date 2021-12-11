//Teensy Flight Controller- Avian Tail Program- Science Fair Project
//Author- Kevin Shen
//Project Start- 11/1/2021

//Credits

//Parts of code structure inspried by Nicholas Rehm
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

//NEED TO DO
//CHANGE SD CARD OUTPUT TO SOMETHING USEFUL
//INVERT MPU6050 TO GET CORRECT ORIENTATIOON
//CLEAN UP

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

File myFile;
MPU6050 mpu;

//PINS
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

Servo elevatorServo;
Servo rotatorServo;
Servo rightElevonServo;
Servo leftElevonServo;
Servo leftESC;
Servo rightESC;

//CONFIGURATIONS
const bool flyingWing = false;
const bool fullBorb = false;
const bool noSpread = true;
const bool diffThrust = false;

const float deadZone = 10;

float elevatorServoOutput = 90;
float rotatorServoOutput = 90;
float rightElevonServoOutput = 90;
float leftElevonServoOutput = 90;
float rightESCOutput = 0;
float leftESCOutput = 0;

int elevatorServoOutputTrim = 0;
int rotatorServoOutputTrim = 0;
int rightElevonServoOutputTrim = 0;
int leftElevonServoOutputTrim = 0;

int elevatorServoPin = SERVO1PIN;
int rotatorServoPin = SERVO2PIN;
int rightElevonServoPin = SERVO3PIN;
int leftElevonServoPin = SERVO4PIN;
int rightESCPin = SERVO5PIN;
int leftESCPin = SERVO6PIN;

float stabilizedPitch;
float stabilizedYaw;
float stabilizedRoll;
float MODE = 0;
float throttle;

int stabilizedPitchInputPin = RX1;
int stabilizedYawInputPin = RX2;
int stabilizedRollInputPin = RX3;
int MODEInputPin = RX4;
int throttleInputPin = RX5;

float tailElevonOffset = 0;

float optimumRotatorServoOutput;

bool isOptimum = true;

float pitchDampener = 2;
float elevonDampener = 2;
float diffThrustDampener = 3;

// ================================================================
// ===               MPU6050 STUFF- NOT MY WORK                 ===
// ================================================================

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
  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

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

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

//read pwm using interrupts
volatile unsigned long PWMTimerStartStabilizedPitch;
volatile int PWMLastInterruptTimeStabilizedPitch;

volatile unsigned long PWMTimerStartStabilizedYaw;
volatile int PWMLastInterruptTimeStabilizedYaw;

volatile unsigned long PWMTimerStartStabilizedRoll;
volatile int PWMLastInterruptTimeStabilizedRoll;

volatile unsigned long PWMTimerStartMODE;
volatile int PWMLastInterruptTimeMODE;

volatile unsigned long PWMTimerStartThrottle;
volatile int PWMLastInterruptTimeThrottle;

void PWMSignalCalculator(float *channel, int pinNum, volatile int *lastInterruptTime, volatile unsigned long *timerStart)
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  *lastInterruptTime = micros();

  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(pinNum) == HIGH)
  {
    *timerStart = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {

    //only worry about this if the timer has actually started
    if (*timerStart != 0)
    {
      //record the pulse time

      *channel = map(((volatile int)micros() - *timerStart), 1000, 2000, -90, 90);
      //restart the timer
      *timerStart = 0;
    }
  }
}
void PWMSignalCalculatorStabilizedPitch()
{
  PWMSignalCalculator(&stabilizedPitch, stabilizedPitchInputPin, &PWMLastInterruptTimeStabilizedPitch, &PWMTimerStartStabilizedPitch);
}
void PWMSignalCalculatorStabilizedYaw()
{
  PWMSignalCalculator(&stabilizedYaw, stabilizedYawInputPin, &PWMLastInterruptTimeStabilizedYaw, &PWMTimerStartStabilizedYaw);
}
void PWMSignalCalculatorStabilizedRoll()
{
  PWMSignalCalculator(&stabilizedRoll, stabilizedRollInputPin, &PWMLastInterruptTimeStabilizedRoll, &PWMTimerStartStabilizedRoll);
}
void PWMSignalCalculatorMODE()
{
  PWMSignalCalculator(&MODE, MODEInputPin, &PWMLastInterruptTimeMODE, &PWMTimerStartMODE);
}
void PWMSignalCalculatorThrottle()
{
  PWMSignalCalculator(&throttle, throttleInputPin, &PWMLastInterruptTimeThrottle, &PWMTimerStartThrottle);
}

float radian(float input)
{
  return input * (3.1416 / 180);
}

void ESCDifferentialThrust()
{
  leftESCOutput = throttle + (stabilizedYaw / diffThrustDampener);
  rightESCOutput = throttle - (stabilizedYaw / diffThrustDampener);
}

void ESCDirectOutput()
{
  leftESCOutput = throttle;
  rightESCOutput = throttle;
}

void tailMovement()
{
  if (stabilizedPitch == 0)
  {
    stabilizedPitch++;
  }
  optimumRotatorServoOutput = map(degrees(atan(stabilizedYaw / stabilizedPitch)), -90, 90, 0, 180);
  //deadzone- if yaw force is real small and pitch is close to 0 then set yaw to 0
  if (stabilizedYaw > -deadZone && stabilizedYaw < deadZone && stabilizedPitch < deadZone && stabilizedPitch > -deadZone)
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
    elevatorServoOutput = (stabilizedPitch / (cos(radian(rotatorServoOutput - 90)))) + 90;
    tailElevonOffset = 0;
  }
  else
  {
    //deflect servo to the point that we actually get correct yaw output (0.707 is the cos(45 deg))
    elevatorServoOutput = -abs(stabilizedYaw / 0.707) + 90;

    //figure out the extra pitch (pitch generated - pitch required (stabilzed pitch). pitch generated is tan(45 deg) times stabilized yaw force, tan (45 deg) is 1, so pitch generated = stabilizedYaw force generated.
    tailElevonOffset = abs(stabilizedYaw) - abs(stabilizedPitch);
  }
  elevatorServoOutput = ((elevatorServoOutput - 90) / pitchDampener) + 90;

  elevatorServoOutput = constrain(elevatorServoOutput + elevatorServoOutputTrim, 0, 180);
  rotatorServoOutput = constrain(rotatorServoOutput + rotatorServoOutputTrim, 0, 180);
  rotatorServoOutput = 180 - rotatorServoOutput;
}

void justPitch()
{
  elevatorServoOutput = (stabilizedPitch / pitchDampener) + 90;
}

void justElevons()
{
  rightElevonServoOutput = ((stabilizedPitch + stabilizedRoll) / elevonDampener) + 90;
  leftElevonServoOutput = (((0 - stabilizedPitch) + stabilizedRoll) / elevonDampener) + 90;
}

void elevonWithTail()
{
  rightElevonServoOutput = ((stabilizedRoll) / elevonDampener) - tailElevonOffset + 90;
  leftElevonServoOutput = ((stabilizedRoll) / elevonDampener) + tailElevonOffset + 90;

  rightElevonServoOutput = constrain(rightElevonServoOutput + rightElevonServoOutputTrim, 0, 180);
  leftElevonServoOutput = constrain(leftElevonServoOutput + leftElevonServoOutputTrim, 0, 180);
}

void elevonAsAileron()
{
  rightElevonServoOutput = (stabilizedRoll / elevonDampener) + 90;
  leftElevonServoOutput = (stabilizedRoll / elevonDampener) + 90;
}

void write()
{
  elevatorServo.write(elevatorServoOutput + elevatorServoOutputTrim);
  rotatorServo.write(rotatorServoOutput + rotatorServoOutputTrim);
  rightElevonServo.write(rightElevonServoOutput + rightElevonServoOutputTrim);
  leftElevonServo.write(leftElevonServoOutput + leftElevonServoOutputTrim);
  rightESC.write(rightESCOutput);
  leftESC.write(leftESCOutput);
}
void serialOutput()
{
  Serial.print("  isOptimum: ");
  Serial.print(isOptimum);
  Serial.print(" issue tester: ");
  Serial.print(abs(stabilizedYaw / 0.707) + 90);
  Serial.print("  stabilizedYaw: ");
  Serial.print(stabilizedYaw);
  Serial.print("  stabilizedPitch: ");
  Serial.print(stabilizedPitch);
  Serial.print("  tailElevonOffset: ");
  Serial.print(tailElevonOffset);
  Serial.print("  elevator: ");
  Serial.print(elevatorServoOutput);
  Serial.print("  rotator: ");
  Serial.print(rotatorServoOutput);
  Serial.print("  right elevon: ");
  Serial.print(rightElevonServoOutput);
  Serial.print("  left elevon: ");
  Serial.print(leftElevonServoOutput);
}

void spreadCalc()
{
  //to be implemented
}

void tailAdjustForSpread()
{
  //to be implemented
}

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
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void SDOutput()
{

  myFile = SD.open("test.txt", FILE_WRITE);

  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) //THIS IF STATEMENT MAKES IT SLOOOOOW BUT IT IS MIPORTANT FOR IT TOWORK
  {                                            // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    myFile.print("ypr\t");
    myFile.print(ypr[0] * 180 / M_PI);
    myFile.print("\t");
    myFile.print(ypr[1] * 180 / M_PI);
    myFile.print("\t");
    myFile.println(ypr[2] * 180 / M_PI);

    Serial.print("  ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
  }
  myFile.close();
}

//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//

void setup()
{
  Serial.begin(115200);

  pinMode(stabilizedPitchInputPin, INPUT);
  pinMode(stabilizedYawInputPin, INPUT);
  pinMode(stabilizedRollInputPin, INPUT);
  pinMode(MODEInputPin, INPUT);
  pinMode(throttleInputPin, INPUT);

  pinMode(elevatorServoPin, OUTPUT);
  pinMode(rotatorServoPin, OUTPUT);
  pinMode(rightElevonServoPin, OUTPUT);
  pinMode(leftElevonServoPin, OUTPUT);
  pinMode(leftESCPin, OUTPUT);
  pinMode(rightESCPin, OUTPUT);

  PWMTimerStartStabilizedPitch = 0;
  PWMTimerStartStabilizedRoll = 0;
  PWMTimerStartStabilizedYaw = 0;
  PWMTimerStartMODE = 0;
  PWMTimerStartThrottle = 0;
  attachInterrupt(digitalPinToInterrupt(stabilizedPitchInputPin), PWMSignalCalculatorStabilizedPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stabilizedRollInputPin), PWMSignalCalculatorStabilizedRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stabilizedYawInputPin), PWMSignalCalculatorStabilizedYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MODEInputPin), PWMSignalCalculatorMODE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(throttleInputPin), PWMSignalCalculatorThrottle, CHANGE);

  elevatorServo.attach(elevatorServoPin);
  rotatorServo.attach(rotatorServoPin);
  rightElevonServo.attach(rightElevonServoPin);
  leftElevonServo.attach(leftElevonServoPin);
  rightESC.attach(rightESCPin);
  leftESC.attach(leftESCPin);

  SDSetup();
  MPU6050Setup();
}

//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//
void loop()
{

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
    if (MODE < 0) //these two can be switched mid flight and are the same configuration
    {
      tailMovement();
      elevonWithTail();
    }
    else if (MODE >= 0)
    {
      justPitch();
      elevonAsAileron();
    }
  }
  if (diffThrust)
  {
    ESCDifferentialThrust();
  }
  else
  {
    ESCDirectOutput();
  }
  //write();
  //serialOutput();
  SDOutput();
}
