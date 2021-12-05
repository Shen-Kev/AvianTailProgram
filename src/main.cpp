#include <Arduino.h>

//PINS  //HIIIIIIIIIIIII
#define S3PIN 2
#define S4PIN 3
#define S5PIN 4
#define S6PIN 5
#define S7PIN 6
#define S8PIN 7

#define SERVO1PIN 23
#define SERVO2PIN 22
#define SERVO3PIN 19
#define SERVO4PIN 18
#define SERVO5PIN 15
#define SERVO6PIN 14

#include <Servo.h>
Servo elevatorServo;
Servo rotatorServo;
Servo rightElevonServo;
Servo leftElevonServo;

//configurations!
const bool flyingWing = true;
const bool fullBorb = false;
const bool noSpread = false;

const float deadZone = 10;

float elevatorServoOutput = 90;
float rotatorServoOutput = 90;
float rightElevonServoOutput = 90;
float leftElevonServoOutput = 90;

int elevatorServoOutputTrim = 0;
int rotatorServoOutputTrim = 0;
int rightElevonServoOutputTrim = 0;
int leftElevonServoOutputTrim = 0;

int elevatorServoPin = SERVO1PIN;
int rotatorServoPin = SERVO2PIN;
int rightElevonServoPin = SERVO3PIN;
int leftElevonServoPin = SERVO4PIN;

float stabilizedPitch;
float stabilizedYaw;
float stabilizedRoll;
int MODE = 0;
int stabilizedPitchInputPin = S3PIN;
int stabilizedYawInputPin = S4PIN;
int stabilizedRollInputPin = S5PIN;
int MODEInputPin = S6PIN;

float tailElevonOffset = 0;

float optimumRotatorServoOutput;

bool isOptimum = true;

float pitchDampener = 2;
float elevonDampener = 2;

void setup()
{
  pinMode(stabilizedPitchInputPin, INPUT);
  pinMode(stabilizedYawInputPin, INPUT);
  pinMode(stabilizedPitchInputPin, INPUT);
  elevatorServo.attach(elevatorServoPin);
  rotatorServo.attach(rotatorServoPin);
  rightElevonServo.attach(rightElevonServoPin);
  leftElevonServo.attach(leftElevonServoPin);

  //Serial.begin(115200);
}

float radian(float input)
{
  return input * (3.1416 / 180);
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
}

void justPitch()
{
  elevatorServoOutput = (stabilizedPitch / pitchDampener) + 90;
}

void justElevons()
{
  rightElevonServoOutput = ((stabilizedPitch + stabilizedRoll) / elevonDampener) + 90;
  leftElevonServoOutput = ((stabilizedPitch - stabilizedRoll) / elevonDampener) + 90;
}


void elevonWithTail()
{
  rightElevonServoOutput = ((0 + stabilizedRoll) / elevonDampener) + tailElevonOffset + 90;
  leftElevonServoOutput = ((0 - stabilizedRoll) / elevonDampener) + tailElevonOffset + 90;

  rightElevonServoOutput = constrain(rightElevonServoOutput + rightElevonServoOutputTrim, 0, 180);
  leftElevonServoOutput = constrain(leftElevonServoOutput + leftElevonServoOutputTrim, 0, 180);
}

void elevonAsAileron()
{
  rightElevonServoOutput = (stabilizedRoll / elevonDampener) + 90;
  leftElevonServoOutput = 90 - (stabilizedRoll / elevonDampener);
}

void write()
{
  elevatorServo.write(elevatorServoOutput + elevatorServoOutputTrim);
  rotatorServo.write(rotatorServoOutput + rotatorServoOutputTrim);
  rightElevonServo.write(rightElevonServoOutput + rightElevonServoOutputTrim);
  leftElevonServo.write(leftElevonServoOutput + leftElevonServoOutputTrim);
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
  Serial.println(leftElevonServoOutput);
}

void spreadCalc()
{
  //to be implemented
}

void tailAdjustForSpread()
{
  //to be implemented
}

void rxInput() {
  stabilizedPitch = map(pulseIn(stabilizedPitchInputPin, HIGH), 1000, 2000, -90, 90);
  stabilizedYaw = map(pulseIn(stabilizedYawInputPin, HIGH), 1000, 2000, -90, 90);
  stabilizedRoll = map(pulseIn(stabilizedRollInputPin, HIGH), 1000, 2000, -90, 90);
  MODE = pulseIn(MODEInputPin, HIGH);  
}

void loop()
{
  rxInput();

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
    if (MODE < 1500) //these two can be switched mid flight and are the same configuration
    {
      tailMovement();
      elevonWithTail();
    }
    else if (MODE >= 1500)
    {
      justPitch();
      elevonAsAileron();
    }
  }
  write();
  //serialOutput();
}
