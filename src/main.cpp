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
Servo rightAileronServo;
Servo leftAileronServo;

//initalize servo outputs
float elevatorServoOutput = 90;
float rotatorServoOutput = 90;
float rightAileronServoOutput = 90;
float leftAileronServoOutput = 90;

//servo trim
int elevatorServoOutputTrim = 0;
int rotatorServoOutputTrim = 0;
int rightAileronServoOutputTrim = 0;
int leftAileronServoOutputTrim = 0;

//define servo pins
const int elevatorServoPin = SERVO1PIN;
const int rotatorServoPin = SERVO2PIN;
const int rightAileronServoPin = SERVO3PIN;
const int leftAileronServoPin = SERVO4PIN;

//initalize reciever input values
float RCpitch;
float RCyaw;
float RCroll;
float DataLog;
bool dataLog = false;//toggles data logging
float MODE;  
bool mode = false;//toggles PID on and off

//define reciever pins
const int RCpitchInputPin = RX3;
const int RCyawInputPin = RX2;
const int RCrollInputPin = RX4;
const int DataLogInputPin = RX1;
const int MODEInputPin = RX5;

//initialize tail variables
bool inDeadzone = true;
const float deadZone = 10;
const float forceToBalenceMAVInPitch = -60; //15 degrees, whatever 15 degrees is for the servo
const float tailForceOffset = -60;    // 15 degrees as well. So when tail is upright it will completely fufil force to balence MAVinpitch
float pitchForce;
float tailForce;

//dampener values to scale servo outputs
float elevatorDampener = 3.0;
float AileronDampener = 2.0;
float tailAileronOffsetDampener = 2.0;

//timekeeping varaibles
float timeInSeconds;
float previousIMUTimeInSeconds;
int SDiteration = 0;
int SDdataLogFrequency = 200;
float timeBetweenIMUInputs;
float timeBetweenAveragePitchError;

//initialize attitude variables
float yaw = 0;
float prevYaw = 0;
float yawChange = 0; //deg/sec
float pitchChange = 0; //deg/sec
float pitch = 0;
float roll = 0;
float spikeThreshold = 360; //deg/sec

//PID controller variables

float PitchPgain = 3.0;
float PitchIgain = 0;
float PitchDgain = 0.5;

float RCpitchScalar = 2.0;
float PitchProportional;
float PitchIntegral;
float PitchIntegralSaturationLimit = 45;
float PitchDerivative;
float PitchError;
float PrevPitchError;
float AvgPitchErrorSum;
float AvgPitchError;
float AvgPrevPitchErrorSum;
float AvgPrevPitchError;
float PitchErrorArray [20];
float PitchDerivativeConstrain = 45;

float PitchOutput;


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

volatile unsigned long PWMTimerStartDataLog;
volatile int PWMLastInterruptTimeDataLog;

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
      *channel = map(((volatile float)micros() - *timerStart), 1100.0, 1900.0, 90.0, -90.0);
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
void PWMSignalCalculatorDataLog()
{
  PWMSignalCalculator(&DataLog, DataLogInputPin, &PWMLastInterruptTimeDataLog, &PWMTimerStartDataLog);
  //datalog, chagned to 0 and 1 in this function
  if (DataLog < 0)
  {
    dataLog = true;
  }
  else
  {
    dataLog = false;
  }
}
void PWMSignalCalculatorMODE()
{
  PWMSignalCalculator(&MODE, MODEInputPin, &PWMLastInterruptTimeMODE, &PWMTimerStartMODE);
  //mode used for PID on or off
  if (MODE < 0)
  {
    mode = true;
  }
  else
  {
    mode = false;
  }
}

//PID Control Loop for pitch
void PitchPID()
{  
  //pitchErrorArray keeps track of past 20 pitch error values to average from
  for (int i = 0; i < 19; i++)
  {
    PitchErrorArray[i] = PitchErrorArray[i+1]; //moving everything down one index
  }

  PitchError = (RCpitch/RCpitchScalar) - pitch;

  PitchErrorArray[19] = PitchError; //setting "latest" pitch error

  //find average previous pitch error
  for(int i = 0; i < 10; i++) {
    AvgPrevPitchErrorSum += PitchErrorArray[i];
  }
  AvgPrevPitchError = AvgPrevPitchErrorSum/10;
  AvgPrevPitchErrorSum = 0;

  //find average pitch error
  for(int i = 10; i < 20; i++) {
    AvgPitchErrorSum += PitchErrorArray[i];
    AvgPitchError = AvgPitchErrorSum/10;
  }
  AvgPitchError = AvgPitchErrorSum/10;
  AvgPitchErrorSum = 0;

  //setting time change to be the difference between start of pitch error average and past pitch error average  
  timeBetweenAveragePitchError = timeBetweenIMUInputs*10; //10 iterations apart
  
  PitchProportional = AvgPitchError * PitchPgain; //proportional value

  // if (PitchIntegral < -PitchIntegralSaturationLimit) //prevent windup
  // {
  //   PitchIntegral = -PitchIntegralSaturationLimit;
  // }
  // else if (PitchIntegral > PitchIntegralSaturationLimit)
  // {
  //   PitchIntegral = PitchIntegralSaturationLimit;
  // }
  // else
  // {
  //   PitchIntegral += PitchError * PitchIgain; //discrete integration
  // }

  PitchIntegral = 0;

  PitchDerivative = (AvgPitchError - AvgPrevPitchError)/timeBetweenAveragePitchError * PitchDgain; //dx/dt discrete derivative
  PitchDerivative = constrain(PitchDerivative, -PitchDerivativeConstrain, PitchDerivativeConstrain); //constrain derviative (to avoid large spikes)
  PitchOutput = PitchProportional + PitchIntegral + PitchDerivative; //pitch desired calculation

  PitchOutput = constrain(PitchOutput, -90, 90);

  if(mode == 0) { //toggling PID loop on and off
    PitchOutput = RCpitch;
  }

}

void tailMovement()
{
  inDeadzone = false;

  if (PitchOutput > -tailForceOffset - deadZone && PitchOutput < -tailForceOffset + deadZone && (rotatorServoOutput < -deadZone || rotatorServoOutput > deadZone))
  {
    PitchOutput = -tailForceOffset - deadZone; //pitch is set to pitch up just under the deadzone, to where yaw can still be generated
    inDeadzone = true;                        //to log data
  }

  pitchForce = PitchOutput + forceToBalenceMAVInPitch; //calculate force needed in pitch
  rotatorServoOutput = constrain(0 - degrees(atan(RCyaw / pitchForce)), -90, 90); //calculate rotation of tail

  if (rotatorServoOutput == 0)
  {
    rotatorServoOutput = 0.01; //no dividing by 0
  }

  tailForce = pitchForce / cos(radians(rotatorServoOutput)); //calculate force tail needs to generate

  elevatorServoOutput = tailForce - tailForceOffset; //calculate elevator force based on tail force

  elevatorServoOutput = constrain(90 + ((elevatorServoOutput + elevatorServoOutputTrim) / elevatorDampener), 0, 180);
  rotatorServoOutput = constrain(90 + rotatorServoOutput + rotatorServoOutputTrim, 0, 180);
}

//calculate aileron movements
void ailerons()
{
  rightAileronServoOutput = ((RCroll) / AileronDampener) + 90;
  leftAileronServoOutput = ((RCroll) / AileronDampener) + 90;

  rightAileronServoOutput = constrain(rightAileronServoOutput + rightAileronServoOutputTrim, 0, 180);
  leftAileronServoOutput = constrain(leftAileronServoOutput + leftAileronServoOutputTrim, 0, 180);
}

//writes signal to actuators
void write()
{
  elevatorServo.write(elevatorServoOutput + elevatorServoOutputTrim);
  rotatorServo.write(rotatorServoOutput + rotatorServoOutputTrim);
  rightAileronServo.write(rightAileronServoOutput + rightAileronServoOutputTrim);
  leftAileronServo.write(leftAileronServoOutput + leftAileronServoOutputTrim);
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

    file.print("time(s)");
    file.print("\t");

    file.print("yaw(deg)");
    file.print("\t");
    file.print("yawChange(deg/s)");
    file.print("\t");
    file.print("pitch(deg)");
    file.print("\t");
    file.print("pitchChange(deg/s)");
    file.print("\t");
    file.print("roll(deg)");
    file.print("\t");

    file.print("PitchError");
    file.print("\t");
    file.print("PitchProportional");
    file.print("\t");
    file.print("PitchIntegral");
    file.print("\t");
    file.print("PitchDerivative");
    file.print("\t");

    file.print("RCyaw");
    file.print("\t");
    file.print("pitchForce");
    file.print("\t");
    file.print("tailForce");
    file.print("\t");
    file.print("RCpitch");
    file.print("\t");
    file.print("PitchOutput");
    file.print("\t");
    file.print("RCroll");
    file.print("\t");

    file.print("dataLog");
    file.print("\t");
    file.print("MODE");
    file.print("\t");

    file.print("inDeadzone");
    file.print("\t");

    file.print("elevatorServoOutput");
    file.print("\t");
    file.print("rotatorServoOutput");
    file.print("\t");
    file.print("rightAileronServoOutput");
    file.print("\t");
    file.print("leftAileronServoOutput");
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

    timeBetweenIMUInputs = timeInSeconds - previousIMUTimeInSeconds;
    previousIMUTimeInSeconds = timeInSeconds;

    prevYaw = yaw;
    PrevPitchError = pitch;

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
    yawChange = (yaw - prevYaw) / timeBetweenIMUInputs; //dx/dt (discrete derivative)

    if (yawChange >= spikeThreshold || yawChange <= -spikeThreshold)
    {
      yawChange = 0;
    }

    if (pitchChange >= spikeThreshold || pitchChange <= -spikeThreshold)
    {
      pitchChange = 0;
    }

    //run PID loops
    PitchPID();
    //run logic
    tailMovement();
    ailerons();
    //write to servos
    write();
  }
}

//write flight data to SD card
void SDOutput()
{
  if (SDiteration >= SDdataLogFrequency)
  {
    file = SD.open("flight_data.txt", FILE_WRITE);
    file.print(timeInSeconds);
    file.print("\t");

    file.print(yaw);
    file.print("\t");
    file.print(yawChange);
    file.print("\t");
    file.print(pitch);
    file.print("\t");
    file.print(pitchChange);
    file.print("\t");
    file.print(roll);
    file.print("\t");

    file.print(PitchError);
    file.print("\t");
    file.print(PitchProportional);
    file.print("\t");
    file.print(PitchIntegral);
    file.print("\t");
    file.print(PitchDerivative);
    file.print("\t");

    file.print(RCyaw);
    file.print("\t");
    file.print(pitchForce);
    file.print("\t");
    file.print(tailForce);
    file.print("\t");
    file.print(RCpitch);
    file.print("\t");
    file.print(PitchOutput);
    file.print("\t");
    file.print(RCroll);
    file.print("\t");

    file.print(dataLog * 100);
    file.print("\t");
    file.print(MODE * 100);
    file.print("\t");

    file.print(inDeadzone * 100);
    file.print("\t");

    file.print(elevatorServoOutput);
    file.print("\t");
    file.print(rotatorServoOutput);
    file.print("\t");
    file.print(rightAileronServoOutput);
    file.print("\t");
    file.print(leftAileronServoOutput);
    file.println("\t");

    file.close();

    SDiteration = 0;
  }
}

//keep track of iterations
void timekeeper()
{
  timeInSeconds = millis() / 1000.0;
  SDiteration++; //to make sure SD card outputs at correct time
}

//VOID SETUP ====================================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(RCpitchInputPin, INPUT);
  pinMode(RCyawInputPin, INPUT);
  pinMode(RCrollInputPin, INPUT);
  pinMode(DataLogInputPin, INPUT);
  pinMode(MODEInputPin, INPUT);

  pinMode(elevatorServoPin, OUTPUT);
  pinMode(rotatorServoPin, OUTPUT);
  pinMode(rightAileronServoPin, OUTPUT);
  pinMode(leftAileronServoPin, OUTPUT);

  PWMTimerStartPitch = 0;
  PWMTimerStartRoll = 0;
  PWMTimerStartYaw = 0;
  PWMTimerStartDataLog = 0;
  PWMTimerStartMODE = 0;
  attachInterrupt(digitalPinToInterrupt(RCpitchInputPin), PWMSignalCalculatorPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCrollInputPin), PWMSignalCalculatorRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCyawInputPin), PWMSignalCalculatorYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DataLogInputPin), PWMSignalCalculatorDataLog, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MODEInputPin), PWMSignalCalculatorMODE, CHANGE);

  elevatorServo.attach(elevatorServoPin);
  rotatorServo.attach(rotatorServoPin);
  rightAileronServo.attach(rightAileronServoPin);
  leftAileronServo.attach(leftAileronServoPin);

  SDSetup();
  MPU6050Setup();
}

//MAIN LOOP ========================================================================================

void loop()
{
  mpu6050Input();
  timekeeper();
  SDOutput();
}
//yooo 707 nyooooom
// cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc:ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// cccccccccccccccccccccccccccccccllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// ccccccccccccccccccccclllllllllllllclccllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// ccccccccccccclllllllllllllllllllllllllllllllllccclccccllccccccccccccccccllccccccccccccccccccccccccccccclclllllllllllllcllcccllccccccccccccccccccccccccccclcclccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// clllcccllllllllllllllllllllllllllllllllllllllllllllllllllcccllccclllllllllllccllcllllccllcclclllclllllllllllllllllllllllllllllllllllllllcclllllcccllllllllllllclllllclcccccccccccccclcccccccccclclllcccclccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllclllllllllllllllllllllllllllllllllllcclccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclcccc
// llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccllllcllllcccccccccclcccccccccclccllcccccccl
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllccccccccccccllcclcllllllllllllllllllllllllllllllclllcllllllllllllll
// llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// lllllllllllllllloolooooooooooooooooooollllllllllllllllllolllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// oolooooooolooooooooooooooooooooooooooooooooooooooooooloooollllllllllllllllllllllllllllllllllllllooooooollllollllolllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// loooooooolloooooooooooooooooooooooooooooooooooooooooollllllllllllllloooooloooooooooooooooollollllooooooooooooooooooooooooolloooooooooooooooollllolllllllllllooooooolllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// llllllllllllllllllllooooooooooooooooooooollooooooooolllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooollooooollllooooolloooooooooooooollooooollllllollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// cccccccccllllllllllllllllllooooooooooooollllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolooooolllooollllooollllllllllllloolllllllllllllllllllllllllllllllllllllllll
// ccccccccccccccccccccllllllllllllllloooollllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooolooooooooooooolooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooollloolloooooollooooooooooooooooooooooooooooolollllllllllllllllllllll
// ccccccccccccccccccccccccccccccccllllllllllllllllllllllllllcclllllllllllllllllllllllollllllooooooooooollooooooolllllllllllllllllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooollllllllllllll
// ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllllllllllllllllllllooooooooooolllooooollllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooddddddddddddddddddddddddddddddddddoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooollooo
// cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloolllllllllllllooooooooooooooddddddoooooooooooooooooooooooooooooooooooddddddddddddddddddddddddddddddddddodddooodddooodddoooooooooooooooooooooooooooooooooooooo
// ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloollllooooooooooooooooooooooooooooooooooooooooooooooooooooooodddddddddddddddoodddddooooooooddddddddddddddddddoooooooooooooooooooooooooooooo
// cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclcclllllllllllllllllllllllllllllllccccccccclcclllllllllllllllllllllllllllllllllllllllllllloooolllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooodddoooooodoooooodddddddddddddddddddddddddddddddooooooooooooooooooo
// cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclllllllllllllllllllllllllllllllcccccccclccccccccccllllllllllllllllllllllloollllllllllllllllllllllllllllllllooooooooooooooooooooooooooooollllloooooooooooooooooooooooooooooooooooooooooooddddddddddddddddddddddddddddddddddoooooooooo
// ::ccccccc:::c::::::::::::::::::::cc:::c:::ccccccc:c:::::ccccccccccccccccccccccccccccccccccllllllllllllllllcllccccccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllooooooooooooooooooolllllllllllllooooooooooooooooooooooooooooooooooooooooooooodddddddddddddddddddddooodooooooooooo
// cc::c:::::::::::::::::::::::::::::::::::::::::::::::::::::::cccccccccccccccccccccccccccccccccccllllllllllllllccccccccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllooooooolllllllllllllllllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
// ccccccccccccccc:::::::::::::::::::::::::::::::::::::::::::::::::::::::ccccccccccccccccccccccccccccllllllllllllcccccccccccccccccccllllccccllclllllllllllllllllllllllllllllllclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
// ccccccccccccccccccccccc::::::::ccccccccc:cc::::::::::::::::::::::::::::::::ccccccccccccccccccccccccccccccllcclllccccccccccccccccccccccccccccccclllllllllllllllllllllllllllcccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloolllolollloollllloooooooooooooooooooooooooooooooooooooooo
// llllllllllllllllllllccccccccccccccccccccccccccccccc:c::::::::c::c:::::::::::c::::::::::::::::cccccccccccccccccclccclcllllllllllllcllllllllllllllllllllllllllllllllllllllllcccccllllccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllooooooooooooooooooooo
// oooooooooooooollllllllllllclllllllllllccccclccccccccccccccccccccccccccccccccccccccccc:::::::::::cccccccccccccccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllcllllllllllllllllllllllllllllllllllllllllllloooooooolll
// looooooooooooooooooooooollllllllllllllllllllllllllllllllllcclllcccllllccccccccccccccccccccccc::c::cc:::ccccccccccccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllllllllllccccccclllllllllllllllllllllllllllllllllllllllllllccccccccccccccccccccccccccccccccccccccclcclllllllllllllllllllllllll
// llooooooooooooooooooooooooooooooooollllllllllllllllllllllllllllllllllllllllllllllllllllccccccccccccccccccccccccccccccccccccccccccccccccccccclcccccllccccclllccccllcclllllllllllllllllllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllllllllllllll
// llllllloooooooooooooooooooooooooooooooollllllllllllllllllllooooooooooooooooooooooooooollllllllooooollllllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccllccllcllllcllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllll
// olllllllloooooooooooooooooooooooooooooooooooooollllllloooooooooooooooooooooooooooooooooooooooooooooooooooolllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllllllllllllllllcclcclcllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// oolllllllloooooooooooooooooooooooooooooooooooolollllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooollllllllllllllcccccccccccccccccccccccccccccccccccccllccccccccccllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllloollloooooooooooooooooooooooooooooooooooolllllllllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolllllllllllllcccccccccccccccccccccccccccccccccccccccccllllllllllllllllcclllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// llllllllllllllllllllloooooooooooooooooooooolllllllllllllllllllllloooolllooooooooooooooooooooooooooooooodooooooooooooooooooooooooollllllllllcccccccccccccccccccccccccclcccccccllllllllllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllllllllllllllllllllllllllllllloooooooooollllllllllllllllllllllllooooollolooooooooooooooooooooooodddoooooodooooooooooooodddoodoooooooooolllllllllcccccccccccccccccccccccccclllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// llllllllllllllllllllllllllllllllllllllolooollollllllllllllllllllllllloolllllooooooooooooooooooooooooooooodddddddddddoooooooodddddddddddddddooooooolllllllllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllolooooooollllllllllllllllccccclllllllllllllllllllllllllllllllllllllllllllooooooooooooooooooooooooooooooooddddddddddddoooooooooooodddddddddddddddooooolllllllcccccccccccccccccccccc:;,,;;,,,;:ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc:c::ccccccccccccccccccccccccccccc
// llllllllooooooooooolllllllllllllccllclllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooodddddddddoooooooooooooodddddddddoooodddddddoooooolllllcccccccccccccccccc,..''.....':cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc:::ccccccccccc:::c::ccccccccccccc
// oooolllllloooooooooooooollllllllllllllllllllllllllllllllllllllllllllllllllooooooooooooooooooooooooooooooooooooddooooooooooooooooooooddddddddddddddoddddddddddddoooooooollllccccccccc;.''.......':cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc::cccc:cccc:::::::::::ccccccccccc
// oooooolllllllllooooolllllllllllllooooooollllllllllllllllllllllllllllllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooddddddddddddddddddddoodoooooooooddddooooooollll:.',......''':ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc::::::::ccc::ccccccc
// ooooolllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooodddddddoodddddddddddddddddooodoooooooooooool,',.........':lcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc:cccccccc::ccccccccccccccc
// oooolloolllllllllllllloollllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooodddddodddddooooooooooooooooolll;.,..........':lllllllllllllllcccccccccccccccclllllllllccccccccccllllllccccccccccccccccccccccccccccccccccccccccccccccccccc
// ooooloooollooolllllllllllllllllllllllllllllllllllllllllllllllllldO0000K0xdooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo:','........''';lllllllllooooollllllllllccccccccccllllllcccccllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccc
// oooooooolllllllllllllllllllllllllllllllllllllllllllllllllllllllclx00kdkXX0xlllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooool'',. ..'''''''';lllllllllllllllllllllllllllllllcccccccccccccccclllllllllllllllllccccccccccccccccccccccccccccccccccccccccccc
// llooooooooloolllllllllllllllllllllllllllllllllllllllllllllllllllllkKK00XXNX0dllllllllllllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooool;.,....''''''''',::cccccccccccccccccccccclllllllllllccccccccccccccccccclllllllllllllllllllccccccccccccccccccccccccclllcccccc
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllx0KKKXXNNXOollllllllllllllooollooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolllllool;..''',''''''''''.......'',,;:;,;'.;clllcccccccccccccccccccccccccccccccccccccccclllllllllllllllllccccccccccccccccccclllllllll
// llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllx0KKKXXXNNXkolllllllllllllllllllllllooooooooooooooooooooooooooooooooooooooooooollllllooooooooollllllllllll:..',..'''''''''''. .....';:;::cl:..,cllllllllllllllllccccccccccccccccccccccccccccccccccllllllllllllllcccccccclllcccllllllllll
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllx0KKKKOOO0XKkollllllllllllllllllllllllllooooooooooollllllloooooooooooooollllllllllllllllllllllllllllllllc'......'''''''''''.  ...........'...:llllllllllllllllllllllllllllllllllllllcccccccccccccccllllllllllllllllllllllllllllllllllll
// lllllllllllllllllllllllllllllllllllllllllllllllllllcllllllllllllllllllx0KOl,;::lx00xllllllllllllllllllllllllllllllooloooollllllllllllloooloollllllllllllllllllllllllllllllllc,..'..,''''''''''..'''......''''',,,:lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllo
// lllllllllllllllllllllllllllllllllllllllllllllllccllllclllllllllllllllllxKk;';::;,:xX0dlcllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll,..','.'''''''''''..;ooooooooooooooooollllllllllllllllllllllllllllllllllllllloooooooooooooooooolllllllllllllllllcclcllllllllllll
// llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllxkc;;:c:;;:xXXOocllllccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll:'..''.''''''''''''..;looolloloolllllolllllllllllllllllllllllllllllllllllllllloolooooooooooooooooooooooooooooooooooooooolllllllll
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllclxkl:cc:clllONWXkoc:,'.'','';lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllc,......''''''''',,'',:llooollooollllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooodddddddddddddooooooooo
// ooooooooooooooolllllllllllllllllllllllllllllllllllllllllllllllllllllllllcldd:,;::;,,dXNNNKd'........;llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllc'.''...''''''''''''..'''',,;:::;;;;:clolllollllllllllllllllllllllllllllllllllllllllooooooooooooooooooooooooooodddddddddddddddddddd
// ooooooooooooooooooooooooooooolllllllllllllllllllllllllllllllllllllllllcllcldko;;::,;xNNNNWNk;.......:lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllc,.',...'''''''''''''...   .......'.';:cl;.'collllllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooodddddddddddod
// oooooooooooooooooooooodddddddoooooooooooollllllllllllllllllllllllllllccclcclxK0doolxXNNWWWWWXx:'...'clcccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll;'..'...''''''''..''.'''. .....''',:cccclc...:llllllllllllllllllllllllllllllllllllllllllllooooooooooooooooooooooooooooooooooooodddddd
// oooooooooooooooooooooooooddooodoodooddddoooooooooolllllllllllllllllllllllllllxKXXXXNNNNNNWWWWWNOl,.,clclcclclllllllllllllllllllllllllllllllllllllllllllllllllllllllll:,..:c,..''................  ..............'..,cllllllllllllllllllllllllllllllllllllllllllllllllllllooolllloooooooooooooooooooooooooddo
// ddddddddddddddooooooooooooooooooooooooooooooooooooooolllllllllllllllllllllllllxKNNXNNNNNNNNNWWWWWKkdolccccccllllllllllllllllllllllllllllllllloolllolooooooooooooooool:;;ldo:;:::::::::::::::::::::;;;;;;;;;cc;::::clddooooooooolllllllllllllllllllllllllllllllllllllllllllllllllllllooooollllllooooooooooooo
// ddddddddddddddddddddddooooooooooooooooooooooooooodddoooooolllllloooolllllllllllxKNNNNNNWWNNNNNWWWWWWNKOxxxkkkkOOOOOOOOOOOOOO00OO000000000000000000000000000KKKKKKKKKKKKKKKKKKKKKKKKKKKKXXXXXXXXXXXXXXXXXXXXXXXNNNXXXXXKKKKK00000OOkxddollllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooo
// dodddddddddddddddddddddddddddddddddddddoooooooooooooolllllllllllllllllllllllllx0XNNWNNNWWWNWWWWWWWWWWNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNWNNWNNWWNWNNWWNWWNNNNWWWWWWWWWNNNNNWWWWWWNNNNNXK0Okdllllllllllllllllllllllllllllllllllllllllllllllllllllllllllooooooo
// ooooooooooooooddddddddddddddddddddxxxxxxxdxdddddddooooooolllllllllllllllllllllkNNNWNXNWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWXKNN0o,,clllllllllllllllllllllllllllllllllllllllllllccclllllllllllllo
// ooooooooooooooooooooooooooooodddddddddddddxxxxxxxxxddddddddoooooooooollllcc::;cdxxkdlok0XNNWWWWWWWWWWWWWXKXXKXWWWWWWWWWWWWWWWWWWWWWWWW0kkO0dkXkdOXWNOdlOXkxkx0Ox0XOxxk0k0kkKXXOdkXOok0NWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWKOxddo;.. ..,:lllllllllllllllllllllllllllllllllllllllllllllccccccccclll
// llllloooooooooooooooooooooooooooooooodddddddddddddddddddddddddddddollc:;;,'''.'',:loooooolcclodxkOO0KKXX000O00NNNNNNNNNNWWWWWWWNNNNNWNOx0OkxxOOoxKW0kxox0kdxx0kx0XkxxxOkOxx0KOkxdOOdo0NWWWWWWWWWWWWWWWWWWWWWWNNNNNNNNNNNXXXXKK0Okkxl::;;,.        .;cllllllllllllllllllllllllllllllllllllllllllllllccccccccc
// llllllllllllllllllllloooooooooooooooooooooooooooooooooodddddoolc:;;,'''''..',;::clodoolc;,,;'...'',,,;;:::cccc:cccccccccccccccclcccccccccccccccccccccclllllccccccccllccccccccccccccccccccccccclccclllllllllllllllllllccc:::;;;,,,,,'',',;'..'',;cldd:.,cllllllllllllllllllllllllllllllllllllllllllllllllllcc
// lllllllllllllllllllllllllllllllllllloooooooooooooooooollc:;;,'...'',;;::ccloooooooooooooooollc::::;::cclllooooooooooooooooooooddddddoododoooooooooooooooooooooooooooollllllllllcccccccccc:::::::::::::::::::::;:cccccclllooodddddxxxkkkOOOO00KXXNNWK; .:llllllllllllllllllllllllllllllllllllllllllllllllllll
// lllcccllllllllcllccccccccclllllllllllllllllllllllcc:;,''''',,;;:ccllooooooooooooooooooooooooooooolc:::;;;::::cccccccccccccllllllllooooooooooodxkkkkkxxxxxxxxxxxxxxxxkxxkkkkkkkkkkkkkdooooooooooooooooooooddddoodxddxxxxxddddddoooolooooooooooddk0Okd;';cllllllllllclllllcclllllllllllllllllllllllllllcllllll
// ccccccccccccccccccccccccccccccccccclllllllllllllc:;;;;;:cclllllllllllllllllllllllloooooooooooooooooooolc:;,'...                      .......''',,;:;;;;::;;,;;;;;;;;;;;;;;;;::cccllc'.                    .............                        .,;;:cllcllcclllcclllcccccccccllcllllllcllcclccccccccccccccll
// ::::::::cccccccccccccccccccccccccccccccccccccccllllllllllcccccclllcccllllllllllllllllllllllooooooooooooloooollc:;;,'.......        .....',,,;;,,,:ccc::::;;;;;;;;;;;;;;;;;::ccoxO00Oo'                                  .....               ..';:clllccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// ::::::ccccccccccccccccccccccccccccccccccccccccccccccclcccccccccccccccccccccccccccccccllllllllllllllllllllllllllllllllccc::::;;;,,,,'.....';:ccc;;:::;;;;;;;;;;;,,,,,;;,;;:clodxkkxdoc,.....................            ..''''.      ....',;:cclllcclcccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc::cc::ccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllc::;,;:::;;;,;;;;;;;,,,,,,,,,,;:clooool:;;::;;;;:::ccccccccccccccccc:::::::::;,;:::::;',;;;;;;::cccllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// ccccccccccccccccccccccccccclcccccccccccccccccccccccccccccclcccccc:::::::::ccccccccccccccccccccccccccccccccccccccccccccccccccccccc:::::::;;;;;;;;,,,,,,,,,,,;;:cllllloolc::::ccllllllllllllllllllllllllllllllllllllllllloooloolllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllllcccccccccccccccccccccclllllllllcclllllllllllllllllllllllllllccccccccccccccccccccccccccccccclllllllllccccccccccccccccccccc::;;;;;;;,;,,,,,,,,,,,;:::ccllllollcc:ccllcccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// lllllllllcccccccccccccccccccccllllllllllllllllllllllllooooooooooooollllcccccllllllllllccclllllllllllllllllcccccccccccccc::c:::;;,,,,,,,,,,,,,;;:cccllooooollllllllllccccccccccccccccccccccllcclllllllllllllllllllllcccclllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
// llllllllllcccccccccccccccccccclllllllllllllloooooooooooooooooooooooooooooolllllllllooolllllllllllllllllllllcccccccc::;;;;,,,,,,,,,'',,;::ccclloooooooooooooooooollllllcccccccccccccccccllcccccccccllclllllllllllccclccclllllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccl
// olllllllllllcccccccccccccccccccclllllllllllllooooooooooooooooodddooooooooooooooooooooooolloooooooooooooolllccc::;;,,,''''''''.,;;:ccclloolllllloooooooooooooooooooooolllcccccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllccclllllllllllllcccccccccccccccccccccccccccccccccccccccccccccllllll
// llllllllllllllllccccccccccccccccccllllllllllloooooooooooooooooddoooddoooooooooooooolloolloooooooooooollcc:;;,,,'''''''',,;;;;;:looooolllolllllllooooooooooooooooooooooolllllccccccllllloooooooooooollllllllllllllllllllllllllllllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccccccllllllllll
// lllllllllllllllllllllcccccccccccccclllllllooooooooooooooooooooooooooooooooooooooooollllllloooolllol::;;,,,,,'''',,;;:;,''...',,;::cccc::::clllloooooooooooooooooooooooooolllllllllloooooooooooooooooooollllllllllolllllllllllloolllllllllllllllllllllllllllllllccclccccllllllccllcccllllllllllllllllllllllll
// lllllllllllloollllllllllllllllllllllllllooooooooooooooooooooooooooooooooooooooooooooollcccc:::;;;;,,,,'',,;;:ccclooooc;'. .......'',,''..,:coocclolllllllllllllooooooooollooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// ooooollllllllllllooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolllc:;,,,,,,,'''',,;;:ccloooooooooooooooc'...''',,,,'',;;;::c,..'clllllllllllllloollllllllllllllloooooooooooloollooooooooooooooooooooooooooooooooooolollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// ooooollllloooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooolcc:;;,,,,''''',,;;::ccloooddoooooooollllooooo:....''',,,,,,;::c:lc...;cllllllllllllllllllllllllllllllllllooooooooooollloooooooooooooooooooooooooooooooooooooollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooollc::;,'''''',,,;;;;;:clooddddoodooooooooooooooooooool:,'................'. .:lllllllllllllllllllllllllllllllllllllllloooooolllooolloooooooooooooooooooooooooooooollooollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// llooooooooooooooooooooooooooooooooooooooooooooooooooooooooollcc:;;,''...'',,;:::,'...',,;;:ccc;;;clllooooooooooooooooooooooolc::;;,,,,,,'''',,,:cllllllllllllllllllllcclllllllllllllllllllllllllllooooooooooooolllllllloooooooooooooollllllllllllllllllllllllllllllllloollllllllllllllllllllllllllllllllllll
// oooooooooooooooooollllllllllllooloooolloooooooooooollcc:;;,,''.'',,;:::cclllllll:.........'',''.',:cll:cloooooolooollloollllllllllllllllllllollllllllllllllllllllllllllllllllllllllllllllllllllllllllllolllooollllloloooooooooooooooollllllllllllcccccccclllllllllllllllolllllollllllllllllllllllllllllloooo
// ooooooooooooooooooolllllllllllllllllllllllllllc::;;,''..',,;:::cclllllllcclllllll;'..''',,,,,,;::::::'  'collllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooooooooooolllllllllllllllcccccccccllllllllllllllllllollllllooollllloolooolloollooo
// olloolllllloooooooolllllllllllllllllllcc::;;,,'',,;:::cclllllllccccccccccclllllll;'.......'',,;::::l;...,lllllllllllllllllllllllllllllllllllllllllclccccllllllllllllccccccclccccccclllllllllllllllllllllllllllllllloooooooollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooolllooloo
// lllllllllllllloooooolllllllllllllllcccc:::ccccclooooollllllllcccccccccclllllllllllc:,'...............  .:lllllllllllllllllllllllllllllllllllllllllccllllcclllllccccccccccccccccccccccccccllllllllllllllllllllllllloolllllllllllllllllllllllllollooooooooooolllooollllllllllllllllllllllllllllloooollllllolll
// llllllllllllllloooolllllllllllllllllllllllllllllllllllllllllcccccccccclllllllllllllllllcc::;;;;;,,,,;,;:llllllllllllllllccllllllllllllllllllllllllllllllllllllcccccccccccccccccccccccccccccccccclllllllllllloooloooooooooooooooooooooooooooooooooooooooooollooloolllllllllllllllllllllllllllllllllllllllllll
// lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllccccccccccllllllllllllllllllllllllllllllllllllllllllllccccccccclllllllllllllllllllllllllllllllllllllllllllllcccccccccccccccccccccccllllllllllllooloooooooooooooooooooooooooooooollllllollllllllllllllllllllllllllllllllllllllllllllllllllllllll
// llllllllllllllllllllllllllllllllllllllllllllooollollllllllllllllllllllllccccccccccccccccccccccccccclllccccccccccccccccccccclllllllllllllllllllccccclllllllllllllllllllllllllllllllccccclcllllllllllllllllllllllllllloooooooooooooooooooooollllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// lllllllllllllllllllllllllllllllllllllooooooooooooollllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccllllllllllllllccccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooolllllllllllllllllllllllllllllllllllllllllcclllllllllllllllllllllllllllllllll
// lllcclllllllllllllllllllllllllllllooooooooooooooollllllllllllllllllllllccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccclcccccccccccccccccclllllllllllllllllllllllllllllllllllllllloooooooololllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll
// lcccccclllllllllllllllllllllloooooooooooooooooooooolllllllllllllllllllllcccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc::cccccccccccccccccccllllllllllllllccllccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllloooooooooooooooooooooooooooooooooooooollllllllllllllll
// ccccccclllllllllllllllllllllllooooooooooooolooolllollllllllllllllllllllllllllllcccccccccccc::::ccccccccccccccccccccccccccccccccccccccccccccc::::::::::::::c::::ccccccccllcclccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllooooolllooooooooooooooooooooooooolllllllllllllll
// cccccccccccccccllllllllllllllooooooollllllllllllllllllllllllllllllllllllllllllllcccccccccc::::::::::::::::::::::::::cccccccccccccccccc:::::cccccc:cccc::::::::cccccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllccccclccccccclllllllllllllllllllllllllllllllllllloollllllllllllllllllllllllll
// cccccccccccccccccllllllllllllllllllllllllllllllllllllllllllllllllllllccllllllccccccccc::::::::::::::::::::::::::::::::::::::::::::::::::::ccccccccccccccccccc::ccccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllllcccccccccccccccccccccclllllllllllllllllllllllllllllllllllllllllllllllllllll
// :::cc:::ccccccccccccclllllllllllllllllccccccccccccccccccccccllllllccccccccccccccc:::::::::::::::::::::::::::::::::;;;;::::::::::::::::::::::cccccccccccccccccccc:ccccccccccccccccccccclllllooooooooooolllllllllllllllllllllllllccccccccccccccccccccccccccccccccclccllllllllllllllllllllllllllllllllllccccccc
// ;::::::::::::::ccccccccccccccclcccccccccc::::::ccccccccccccccccccccccccccccc::::::::::::::::::::::::::::::::::::;;;;;;;;:::::::::::::::::::::::cccccccccccccccccc:ccccccccccccccccccccclllllloooooooooooooollllllllllllllccccccccccllllcccccccccccccccccccccccccccccccllllllllllllllllllllllllllllllllllllll
// ;;;;:::::::::::::::::ccccccccccc::::::::::::::::::::cccccc:cccccccccc:::::::::::::::::::::::::::;;;;;:::::::::::;;;;;;;;:::::::::::::::::::cccccccccccccccccccccccccccccc:::::ccccccccccllllllllllllllolloolllllllllllllllllllllcclllllllllccccccccccccccccccccccccccccccccclccclcllllllllllllllllllllllllll
// ;;;;;::::;;;:;;;;;;;;;;;;;;::::::;;;;;;;;:::::;;;;;;;::::::::::cc::::::::::::::::::::::::;;;;;:::::::;::::::::::::::::::::::::::::::::::::ccccccccccccccccccccccccccccc:::::::::ccccccccccllllllllllllllllllllllllllllllllllllllllllllccccccccccccccccccccccc::::cccccccccccccccccccclllllllllllllllllllllll
// ;;;;;;;;;;;;;;;;;,,,;;,,,,,,;;;;;;;;;;;;;;;;;;;;;;;;;;;;;::::::::::::::::::::::::::::::;;;;;;;;:::::::;;;::::::::::::::::::::::::::::::::::::::::ccccccccccccccccccc:::::::::::::::::ccccccllllllllllllllllllllllllllllllllllllllllllllllllcccccccccccccc:::::::::ccccccccccccccccclllllllllllllllllllllcccc
// ::;;;;;;;;;;;;;;;;;:;;;;;;,,,,,,,,,,,,,,,,,,,,,,;;;;;:::;;;;;;;;;;;;;;;;;;;;;;;;;;;::::;;;,,,;;;;:;;;;;;;;;:::::::::::::::::::::::::c:::::::ccc:cccccccccccccccccccc:::::;;;::::::::::cccccccccllllllllllllllllllllllllllllllllllllllllllllllccccccccccccccc:::cccccccccccccccccccccccccccccccccclccllllcccc
// ::::;;:;:::::::::::::::;;;;;;;;;,,,,,,,,,,,,,,,,,,,,;;;;;;,,;;;;;;;;;;;;;;;;;;,,,,,;;;;;;;;,,,;;;;;;;;;;;;;;:::;:::::::::::::::::::::::::::ccccccccc::::::cccccccccc::::::;;;;;;;;::::::cccccccccccccllllcccccccccccccccccccccllcccccllcccllccccclllcccccccccccccccccccc::::::::c::::::ccccccccccccccccccccc
// :::::::::::::::::::::::::;;;;;;;;,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;;;,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;;;;;;;;:::::::::::cccc::cc::::::cccccccc:::::cccclllllllccc:::::;;;;;;;:::;::::::::::::::ccccccccccccccccccccccccccccccccccccccccccccccccccccccllllccllcccccccccccccccc::::::::ccccccccccccccccccc
// ;;;;;;;:::::::::::ccc::::::;;;;,,,,,,,;,,,,,;;,,,,,,,,,,,,,,,,,,,,,,;;,,,,,,,,,;,,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;;;:::::::cccccccccccccccccccccccccccllllloooollllllcc::::;::::::::;;;;;;;;;;;;;;;;;;;;;;;::::::::::::::::::::::::c::ccccccccccccccccccllcccccccllllllllllllccccccccccccccccccccccc::cccccc
// ;;;;;;;;;::::::::::::::cccc:::;;;;;;,,,,;;;;;;;;;;;;;;;;;;,,;;;;;;;;;,,,,,,,,,,,,,,,,,,''''''',,,,,,,,,,,,,,,,,,;;;;;;:::cccccccccccccccccccclllllllllooooooooooooolllccc::::::::::;;,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;;;;;;;;;;;;:::;;:::::::::::cccccccccccccccccccllllllllllllllllllllllcccccccccccccccccc
// ;;;;;;;;;:::::::::::::cccccc:::::::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:::;;;;;;;;;;;;;;,,,,''''''''''''',,,,,,,,,,,,,;;;;;:::cccccccclllllllllllllllllllllllllllloooollllllllllccccc:::;;,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;::;;;;;;;;;;;;;;;;:::::::cccccccccccccccccclllllllllllllllllllllllllllllllllccccc
// ;;;;;;;;;;:::::::::::::ccccccc:::::;;;;;;;;,,,,,,,;;;;;;;;;;;;;;;;:;;:::::;;;;;;;;;;,,,,''''''''''''''''',,,,;;;;::::ccccclllllllllllllllllllllllllllllllllllllllllllllllccc:::;;,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;::::::::::::::::::::::;::::::::::::::::::cccccccccccccccclllllllllllllllllllllllllll
// ,,,,,,,;;;;;::;;:::::::::::cccccc:::;;;;;;,,,,,,,,;;,,,,,;;;;;;;;;;;;;;:::::;;;;;;;;;;,,,''''''''',,,,,,;;;;;;::::ccccccccclllllllllllllllllllllllllllllllllllllllllllcccccc::::;,,''',,,,,,,,,,,,,,,,,,,,;,,,,;;,,;;;;:::::::cccc:::::ccccccc:::::::::::::::::::::::::::cccccccccccllllllllcccccccccccccccc
// ,,,,,,,,,,,;;;;;;;;;;:::::::::::::::::::;;,,,,,,,,,'',,,,,,,;;;;;;;;;;;;;;;:;;;;;;;;,,,''''''''',,,,,;;;;;;;::::::cccccccccccccccccccccccccccccccccccccccccccccccccccccccccc::::::;,,,,,,,,,,,,,,,,,,;;;;;,;;;;;;;;;;;;;::::::::::::::::ccccccccc:::::::::::::::::::::ccccccccccllllllllllllllllccllllllcccc
// ',,,,,,,,,,,,,;;;;;;;;;;;;;;;;;::::::::::;,,''',,,'',,,''''',,,,,,;;;;;;;;;;;;;,,,,,,,,,''''''''''',,,;;;;;;;;:::::::::::ccccccccccccccccc::ccc:c:::cccccccccccccccccccccccc:::::::;;;,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;;;;;;;;::::::::::::::::ccccccc:::::::::::::::::::::ccccccccllllllllllllllllllllllllllll
// ''''''''''''''',,,,;;;;;;;;;;;;;;;::::::::;,,,,,,,,,,,,'''''''',,,,,;;;;;;;;;;;;;,,,,,,,,,,'''''''''',,,,,,,;;;;::::::::::::cc::c::::::::::::::::::::::::::::::ccccccccc::::::::::::;;;;,,,,,,,,,,,,,,;;;;;;;;;;;;;;;;;;;;:::::::::::::::::::::cccccccccccccc:::::::::::ccccccccccllllllllllllllllllllllllll
// ''''''''''''''''''''',,,,,,,,,,,;;;;;;;;;;;,,,,'''''''''''''''''',,,,,;;;;;;;;;;;;;;,,,,,,;,,''''''''',,,,,,;;;;;;;;::::::::::::::::::;:::::;;;;;;;::::::;;::::::ccccccccccccccc::::;;;;,,,,,,,,,,,,,,,,,,,;;;;;;;;;;;;::::::::::::::::::::::::::::cccccccccccc:::ccccccccccccccccclllllllllllllllllllllllll
// ,,,,,,,,,,,,'',,'',,,'''''',,,,,,,,;;,,,,,,;;,,,,,,,,'',,,,'''''',,,,,,;;,,,,,,,;;;;;,,,,,;;;,''',,,,'',,,;;,,,,,,;;;;;;;;;::;;;;::::;;;;::;;;;;;;;;;;;;;,,;;;:::::::::::::ccccccc:::::;;;,,,,,,,;;,;;;;;;;;;;;;;;;;;;;;;;:::::::::::::::::::::::::cccccccccccccccccccccccccccccccccclccccccclllllllllllllll
// ;;;;;;;;;;;;;;;;;;;;;;,,,,,,,;;;;;;;;;;,,,;;;;;;;;;;;;;,,,,,,,,,,,,,,,,,,,,,''',,,,,;;;,,,,;;;,,',,;;;,,,,,;,,,,,;;,,,;;;;;;;;;;;;;;;;;;;;;;;;,,,,,,,,,,,,,,,;;;::::::::::::::::::::::::::;,,,,,,;;;;;;:::::;;;;;,,;;;;;;;;;;:::::::::::::::::::::::::ccccccccc:cccccccccccccccccccccccclllcclllllllllllllll
// ;;;;;;;;;;;,,;;;;;;;::::;;;::::::::::;;;;;;;;;;;;;:;;;;;;,,,,''''',,,,,,,,,,,,,,,,,,,,,,,,,,;;;,,,,;;;;;;;;;;,,,,,,,,,,,;,,,,,,,,,,,;,,,,,,,,,,,,,,,''''''',,,;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:::::::;;;;,,,,,,,;;;;;;::::::::::::::::::::::ccccccccccccccccccccccccccccccclllllllllllllllllllllll
// ;;;;;;;;;;;;;,;;;;;;::::::::::::::::::::;;;;;;;;;;::;;;;;;,,,,''',,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,;;;;;;;,,,,,,,,,,,,,,,,',,,,,,,,,,,,,,,''''',,,,,''''',''',,;;;;,,;;,,;;;,,;;;;;;;,,,,,;;;;;;;;;;;;;;;;;;;;:::;;;;;;,,,,,,,;;;;;;;;;;;;:::::::::::::::ccccccccccccccccccccccccccccccllllllllllllllllllllll
