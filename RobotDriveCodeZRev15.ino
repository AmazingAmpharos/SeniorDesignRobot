/*
   7.2 Changes
   -Adapted 7.1 changes to full battery
   -Uncommented Color Checking
   7.1 Changes
   -Added New IR sensor functionality to the code
   7 Changes:
   Added FindPeg() to code and several color sensing functions outside main loop
   5.9 Changes:
   -TurnRight() now correctly handles undershoot.
   -DriveLeftSense(), DriveRightSense(), and DriveFrontSense() have brakes.
   -DriveRightNotLeft() now works correctly [DriveLeftNotRight() was never broken]
   -Some superfluous delays in sensing were removed (important ones remain).
   -Some superfluous checks from old ideas were removed for code speed and readability (no change to functionality).
   -Specific serial printing commented out for code speed.
   5.5 Changes:
    -Changes to DriveRightSense() and DriveLeftSense() ?????
    5.0 Changes:
    -TurnLeft() and TurnRight() programmed to work with new IMU.
    -TurnLeft() and TurnRight() now rely on two parameters. First parameter is speed (integer between 0 and 200) and second is target direction.
    -Target direction is defined as EastFace, NorthFace, WestFace, or SouthFace. Robot starts oriented at EastFace.
    4.0 Changes:
   -RobotStart() function added to replace initializations. This function initializes the compass's reference value, kills all motors, and sets the initial position of the gripper to up.
   -Compass reference value now calculated via polling to improve accuracy.
   -TurnLeft() and TurnRight() functions added for 90 degree turns via compass. Currently poorly parameterized.
   -Initial readings of sensors moved into DriveLeftSense() and DriveRightSense() to simplify calls to those functions.
   3.5 Changes:
   -None (Input changes here)
   3.0 Changes:
   -Put DriveLeftSense() and DriveRightSense() as pre-defined functions.
   -Made turning while driving forward slightly faster.
   -Right sensor pins defined.
   -Cleaned up FrontIR sensor use to respond more quickly.
   -Added in Howard's compass code, did appropriate integration work and cast all compass variables as floats.
   -Defined function RobotPark() that causes the robot to stay parked indefinitely upon being called.
   -Swapped Pin assignment of L/R Ultrasonics back and moved them to make room for other I/O on board
   1.6 Changes:
   -Added FrontIR sensor to code, if the sensor reads less than 27, the robot stops.
   -Cleared commented code blocks from v1f.5
   -Swapped Pin assignment of L/R side Ultrasonics
   1.5 Changes:
   -Serial1 printing of current state
   -LED on indicator
    1.4 Changes
    -Added Serial1 Printing
*/
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Average.h>
Average<byte> ave(15);
Adafruit_BNO055 bno = Adafruit_BNO055();
// These constants won't change.  They're used to give names to the pins used;

const byte MotorB1 = 3;
const byte MotorB2 = 4;
const byte MotorA1 = 5;
const byte MotorA2 = 6;
const byte SideBLPulse = 10;
const byte SideBLEcho = 9;
const byte SideFLPulse = 8;
const byte SideFLEcho = 7;
const byte SideBRPulse = 14;
const byte SideBREcho = 15;
const byte SideFRPulse = 16;
const byte SideFREcho = 17;
const byte FrontLPulse = 22;
const byte FrontLEcho = 23;
const byte FrontRPulse = 20;
const byte FrontREcho = 21;
const byte S0 = 24;//pinB
const byte S1 = 25;//pinA
const byte S2 = 26;//pinE
const byte S3 = 27;//pinF
const byte LED = 13;//pinD
const byte taosOutPin = 28;//pinC
byte colorcounter = 0;
byte loopcontrol = 10;
//int IRcounter = 1; //Counter for checking IR
const byte Turf = 255;
const byte City = 200;
const byte up = 37 ;
const byte down = 165;
const byte openit = 63;
const byte closeit = 175;
const byte fwd = 150;
const byte tfwd = 200;
char report[80];

// These are the variables that will vary at runtime.

int FrontSideSense = 0;
int BackSideSense = 0;
int ExtraSense = 0;
int FrontIR = 0;
int GripperIR = 0;
float EastFace;
float WestFace;
float NorthFace;
float SouthFace;
float CurrentTurn;
bool PegColor = false;
bool RiverPeg = false;


//---------------------------servo setup**************************
Servo tilt;  // create servo object to control a servo
Servo grip;  // create servo object to control a servo
//------------------------------------


void setup() {
  // Set pins to appropriate I/O modes.
  TCS3200setup();
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(SideFLPulse, OUTPUT);
  pinMode(SideFLEcho, INPUT);
  pinMode(SideBLPulse, OUTPUT);
  pinMode(SideBLEcho, INPUT);
  pinMode(SideFRPulse, OUTPUT);
  pinMode(SideFREcho, INPUT);
  pinMode(SideBRPulse, OUTPUT);
  pinMode(SideBREcho, INPUT);
  pinMode(FrontLPulse, OUTPUT);
  pinMode(FrontLEcho, INPUT);
  pinMode(FrontRPulse, OUTPUT);
  pinMode(FrontREcho, INPUT);
  pinMode(LED, OUTPUT); //For "ON" inidication from LED
  //**************************************************************
  tilt.attach(11);  // attaches tilt servo to pin 11
  grip.attach(12);  // attaches grip servo to pin 12
  Serial1.begin(9600);
  Wire.begin();
  if (!bno.begin())
  {
    Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Motor 1 side is the positive side, Motor 2 side is the negative terminal. Set A2 and B1 high to turn clockwise; A1 and B2 high will turn counterclockwise.
  // Motors operate with speeds varying from about 100 to 200.
  RobotStart();
// Peg 1 Code
  DriveFrontBlind(255, 35);
  DriveLeftSenseIMUCity(fwd, 1, EastFace);
  DriveBackBlind(fwd, 25);
  delay(300);
  TurnLeftBurst();
  TurnLeft(150, NorthFace);
  TurnStraight(190, NorthFace);
  delay(300);
  DriveFrontBlind(255, 35);
  DriveFrontSense(120, 27);
  delay(300);
  TurnRightBurst();
  TurnRight(145, EastFace);
  TurnStraight(190, EastFace);
  delay(300);
  DriveFrontBlind(255, 35);
  DriveLeftSenseIMUCity(fwd, 1, EastFace);
  delay(200);
  TurnStraight(190, EastFace + 5);
  delay(200);
  DriveFrontBlind(255, 35);
  DriveRightNotLeftSenseIMU(fwd, EastFace);
  DriveCenterSenseIMU(fwd, 12, EastFace);
  TurnStraight(190, EastFace);
  DriveFrontBlind(fwd, 450);
  delay(300);
  TurnStraight(190, EastFace);
  FindPeg(City);
  delay(300);
  TurnStraight(190, EastFace);
  delay(300);
  DriveBackSense(fwd, 10);
  DriveBackBlind(fwd, 200);
  delay(300);
  TurnRightBurst();
  TurnRight(145, SouthFace);
  delay(500);
  DriveFrontSense(120, 15);
  delay(300);
  TurnRightBurst();
  TurnRight(145, WestFace);
  TurnStraight(190, WestFace);
  delay(300);
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 10, WestFace);
  ReturnPeg();
// Peg 2 Code
  delay(300);
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 1, EastFace);
  delay(300);
  TurnLeftBurst();
  TurnLeft(145, NorthFace);
  TurnStraight(190, NorthFace);
  delay(300);
  DriveForwardPeg2(fwd);
  DriveFrontBlind(fwd, 300);
  delay(300);
  TurnLeftBurst();
  TurnLeft(145, WestFace);
  TurnStraight(190, WestFace);
  delay(300);
  DriveFrontBlind(255, 35);
  DriveFrontBlind(fwd, 500);
  TurnStraight(190, WestFace);
  DriveFrontBlind(fwd, 300);
  DriveCenterSenseTime(fwd, 18, 15);
  delay(300);
  TurnStraight(190, WestFace);
  delay(200);
  FindPeg(City);
  delay(300);
  TurnStraight(190, WestFace);
  delay(300);
  DriveBackBlind(255, 35);
  DriveBackSenseIMU(fwd + 10, 10, WestFace);
  DriveBackBlind(fwd + 10, 50);
  delay(300);
  TurnLeftBurst();
  TurnLeft(145, SouthFace);
  TurnStraight(190, SouthFace);
  delay(300);
  DriveFrontSense(120, 27);
  delay(300);
  TurnRightBurst();
  TurnRight(145, WestFace+2);
  TurnStraight(190, WestFace+2);
  delay(300);
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 1, WestFace);
  delay(300);
  ReturnPeg();
  delay(300);
//  Peg 3 Code
  DriveFrontBlind(255, 35);
  DriveLeftSenseIMUCity(fwd, 1, EastFace);
  delay(200);
  TurnStraight(190, EastFace + 5);
  delay(200);
  DriveFrontBlind(255, 35);
  DriveRightNotLeftSenseIMU(fwd, EastFace);
  delay(200);
  DriveLeftSenseCloseIMU(fwd, 1, EastFace);
  delay(200);
  TurnLeft(145, NorthFace);
  TurnStraight(190, NorthFace);
  delay(200);
  DriveFrontBlind(255, 35);
  DriveForwardPeg2RightSide(fwd);
  DriveFrontBlind(fwd, 500);
  DriveForwardPeg2RightSide(fwd);
  delay(200);
  TurnRight(145, NorthFace + 60);
  delay(200);
  DriveFrontIMUMixed(tfwd, fwd+20, 60000, NorthFace + 60, 26);
  DriveBackBlind(tfwd, 280);
  delay(200);
  TurnLeft(145, NorthFace);
  delay(200);
  DriveFrontBlind(tfwd, 800);
  delay(200);
  DriveFrontSearchPeg4(200, 200);
  ReturnPeg4();
// Peg 4 Code  
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 1, EastFace);
  DriveBackBlind(fwd, 50);
  delay(300);
  TurnLeftBurst();
  TurnLeft(145, NorthFace);
  delay(300);
  DriveForwardPeg2(fwd);
  DriveFrontBlind(fwd, 350);
  DriveForwardPeg2(fwd);
  DriveFrontBlind(fwd, 445);
  delay(300);
  TurnLeft(145, WestFace+25);
  TurnStraight(255, WestFace);
  delay(300);
  DriveFrontIMUMixed(tfwd, tfwd, 550, WestFace, 1);
  delay(300);
  DriveFrontSearch(fwd+20, tfwd);
  delay(300);
  GetPeg3();
  DriveFrontIMUMixed(tfwd, tfwd, 1000, WestFace, 1);
  TurnStraight(255, WestFace + 42);
  DriveFrontSearchFinal(tfwd, 150);
  delay(200);
  TurnLeft(145, WestFace - 35);
  delay(200);
  FindPegTurf(Turf);
  delay(200);
  TurnRight(145, NorthFace);
  delay(200);
  DriveFrontSense(tfwd, 27);
  delay(200);
  TurnRight(145, EastFace);
  delay(200);
  DriveLeftSenseIMU(tfwd, 27, EastFace);
  delay(200);
  TurnRight(145, SouthFace);
  delay(200);
  LeftWallSpace(200);
  DriveLeftSenseIMUSpecial(tfwd + 20, 26, SouthFace);
  delay(200);
  DriveBackBlind(255, 180);
  TurnRight(145, SouthFace + 68);
  delay(200);
  DriveFrontBlind(tfwd, 450);
  DriveFrontSpecial(tfwd);
  delay(200);
  TurnLeft(145, SouthFace);
  TurnStraight(190, SouthFace);
  delay(200);
  DriveFrontSense(fwd, 27);
  delay(200);
  TurnRight(145, WestFace+2);
  TurnStraight(190, WestFace+2);
  delay(200);
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 1, WestFace);
  ReturnPegFinal();
  RobotPark();
}

int ReadSonicSensor(byte Pulse, byte Echo) {
  long duration;
  digitalWrite(Pulse, LOW);
  delayMicroseconds(2);
  digitalWrite(Pulse, HIGH);
  delayMicroseconds(10);
  digitalWrite(Pulse, LOW);
  duration = pulseIn(Echo, HIGH);
  //Calculate the distance (in m m) based on the speed of sound.
  return duration / 5.82;
}

float ReadCompass() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //Serial1.println(euler.x());
  return euler.x();
}

void DriveRightSense(int Speed, int Distance) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
        Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveLeftSense(int Speed, int Distance) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void TurnLeft(int Speed, float TargetDirection) {
  int TempSpeed = Speed;
  boolean Overshoot = true;
  float CurrentTurn = ReadCompass();
  byte i = 10;
  int j = 0;
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  float InitialTurn = CurrentTurn;
  float TempTurn;
  Serial1.println("Before Turning");
  Serial1.println(CurrentTurn);
  while (abs(CurrentTurn) > 30) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB2, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  j = 0;
  analogWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  delay(200);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  Serial1.println("Undershoot?");
  Serial1.println(CurrentTurn);
  i = 10;
  while (CurrentTurn > 5) {
    Overshoot = false;
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB2, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  j = 0;
  analogWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  delay(1000);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  i = 20;
  Serial1.println("Overshoot?");
  Serial1.println(CurrentTurn);
  Speed = TempSpeed;
  if (abs(CurrentTurn) > 25 || Overshoot == true) {
    Speed = Speed * .6;
  }
  while (abs(CurrentTurn) > 5) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    //  Serial1.println("Iterator");
    //  Serial1.println(i);
    //  Serial1.println("Speed");
    //  Serial1.println(Speed);
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 20;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 180) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  delay(100);
}

void TurnRight(int Speed, float TargetDirection) {
  int TempSpeed = Speed;
  boolean Overshoot = true;
  float CurrentTurn = ReadCompass();
  byte i = 10;
  int j = 0;
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  float InitialTurn = CurrentTurn;
  float TempTurn;
  Serial1.println("Before Turning");
  Serial1.println(CurrentTurn);
  while (abs(CurrentTurn) > 30) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  j = 0;
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  delay(200);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  Serial1.println("Undershoot?");
  Serial1.println(CurrentTurn);
  i = 10;
  while (CurrentTurn < -5) {
    Overshoot = false;
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  j = 0;
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  delay(1000);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  i = 20;
  Serial1.println("Overshoot?");
  Serial1.println(CurrentTurn);
  Speed = TempSpeed;
  if (abs(CurrentTurn) > 25 || Overshoot == true) {
    Speed = Speed * .6;
  }
  while (abs(CurrentTurn) > 5) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB2, Speed);
    i--;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 20;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 180) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  delay(100);
}

void RobotPark() {
  while (1) {
    analogWrite(MotorA1, 0);
    analogWrite(MotorA2, 0);
    analogWrite(MotorB1, 0);
    analogWrite(MotorB2, 0);
    digitalWrite(LED, LOW);
  }
}

void DriveFrontSense(int Speed, int Distance) {
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  while (FrontIR > Distance) {
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    Serial1.println(FrontIR);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveFrontBlind(int Speed, int Time) {
  int i = 0;
  while (i < Time) {
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    delay(1);
    i++;
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void DriveBackBlind(int Speed, int Time) {
  int i = 0;
  while (i < Time) {
    analogWrite (MotorB2, Speed);
    analogWrite (MotorA2, Speed);
    delay(1);
    i++;
  }
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void RobotStart() {
  delay(400);
  EastFace = ReadCompass();
  SouthFace = EastFace + 90;
  if (SouthFace >= 360) {
    SouthFace = SouthFace - 360;
  }
  WestFace = SouthFace + 90;
  if (WestFace >= 360) {
    WestFace = WestFace - 360;
  }
  NorthFace = WestFace + 90;
  if (NorthFace >= 360) {
    NorthFace = NorthFace - 360;
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);
  digitalWrite(LED, HIGH); //turn on LED
  //************************************************************
  delay(60); //Delay for Servo position consistency
  tilt.write(37);   //lift to ~90 degrees
  delay(60);
  while (1) {
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    Serial1.println("Stuck in FL loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
  while (1) {
    FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    Serial1.println("Stuck in BL loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
  while (1) {
    FrontSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    Serial1.println("Stuck in BR loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
  while (1) {
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    Serial1.println("Stuck in FR loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
  while (1) {
    FrontSideSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
    Serial1.println("Stuck in L loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
  while (1) {
    FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
    Serial1.println("Stuck in R loop");
    if (FrontSideSense > 10) {
      break;
    }
  }
}

void DriveRightNotLeftSense(int Speed) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  delay(2);
  int BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
  while ((FrontSideSense < 300 || BackSideSense < 300) && BackOppositeSideSense > 250) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Straight", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Runaway", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Crash", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
        snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Broken", FrontSideSense, BackSideSense, BackOppositeSideSense);
        Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void DriveLeftNotRightSense(int Speed) {
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  delay(2);
  int BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
  while ((FrontSideSense < 300 || BackSideSense < 300) && BackOppositeSideSense > 250) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 250 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 60) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
        Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void FindPeg(int Speed) {
  int FrontRight, FrontLeft, FrontDiff;
  char report[80]; //Variable to generate Serial1 Output
  delay(5);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
  delay(5);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
  Serial1.println(report);
  Serial1.println("Beginning Loop");
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);

  while ((FrontRight >= 55) && (FrontLeft >= 55)) {
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed);
    delay(70);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    delay(500);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
    delay(5);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
    Serial1.println("Not yet reached Peg");
  }
  delay(5);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
  delay(5);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  FrontDiff = FrontRight - FrontLeft;
  while (abs(FrontDiff) >= 7) {
    delay(5);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
    delay(5);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    while (FrontRight > FrontLeft && (FrontRight < 250)) {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
      delay(5);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left both see peg");
    }
    while (FrontLeft > FrontRight && (FrontLeft < 250)) {
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
      delay(5);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right both see peg");
    }

    while (FrontRight > FrontLeft && (FrontRight > 250))
    {
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA2, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
      delay(5);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left Right not see peg");
    }

    while (FrontLeft > FrontRight && (FrontLeft > 250))
    {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
      delay(5);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right Left not see peg");
    }
    delay(5);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 10;
    delay(5);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    FrontDiff = FrontRight - FrontLeft;
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
  }
  byte IRArray[15];
  for (byte i = 0; i < 15; i++) {
    IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
    ave.push(IRArray[i]);
    Serial1.print(IRArray[i]);
    delay(2);
  }
  FrontIR = ave.mode();
  Serial1.println(" ");
  Serial1.println(FrontIR);
  Serial1.println("Pre-grab");
  while (FrontIR > 12 || FrontIR < 11) {
    while (FrontIR > 12 && FrontIR < 26) {
      Serial1.println("Forward");
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, Speed);
      delay(25);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
    while (FrontIR < 11 || FrontIR > 25) {
      Serial1.println("Backward");
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB2, Speed);
      delay(25);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
  }
  grip.write(openit);
  delay(500);
  tilt.write(down);
  delay(1000);
  analogWrite(MotorA1, Speed);
  analogWrite(MotorB1, Speed);
  delay(70);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  delay(500);
  grip.write(closeit);  //grab
  delay(1000);
  tilt.write(down-1);
  delay(500);   
  tilt.write(up);   //lift to ~90 degrees
  
  delay(1000);
  while (loopcontrol > 0) {
    if (detectColor(taosOutPin) == 0) {
      colorcounter++;
    }
    loopcontrol--;
    delay(100);
  }
  if (colorcounter > 0) {
    PegColor = true;
    Serial1.println("Red");
  }
  else {
    PegColor = false;
    Serial1.println("Yellow");
  }
  colorcounter = 0;
  loopcontrol = 10;
  taosMode(0);
  digitalWrite(LED,HIGH);
  return;
}

//-----------------------------------------------------------------------


int detectColor(int taosOutPin) {

  float blue = colorRead(taosOutPin, 2, 1);
  float green = colorRead(taosOutPin, 3, 1);
  if (green > blue)
    return 0;
  else
    return 1;

}

/*
This section will return the pulseIn reading of the selected color.
It will turn on the sensor at the start taosMode(1), and it will power off the sensor at the end taosMode(0)
color codes: 0=white, 1=red, 2=blue, 3=green
if LEDstate is 0, LED will be off. 1 and the LED will be on.
taosOutPin is the ouput pin of the TCS3200.
*/

float colorRead(int taosOutPin, int color, boolean LEDstate) {

  //turn on sensor and use highest frequency/sensitivity setting
  taosMode(1);

  //setting for a delay to let the sensor sit for a moment before taking a reading.
  int sensorDelay = 100;

  //set the S2 and S3 pins to select the color to be sensed
  if (color == 0) { //white
    digitalWrite(S3, LOW); //S3
    digitalWrite(S2, HIGH); //S2
    // Serial1.print(" w");
  }

  else if (color == 1) { //red
    digitalWrite(S3, LOW); //S3
    digitalWrite(S2, LOW); //S2
//    Serial1.print(" r");
  }

  else if (color == 2) { //blue
    digitalWrite(S3, HIGH); //S3
    digitalWrite(S2, LOW); //S2
//    Serial1.print(" b");
  }

  else if (color == 3) { //green
    digitalWrite(S3, HIGH); //S3
    digitalWrite(S2, HIGH); //S2
//    Serial1.print(" g");
  }

  // create a var where the pulse reading from sensor will go
  float readPulse;

  //  turn LEDs on or off, as directed by the LEDstate var
  if (LEDstate == 0) {
    digitalWrite(LED, LOW);
  }
  if (LEDstate == 1) {
    digitalWrite(LED, HIGH);
  }

  // wait a bit for LEDs to actually turn on, as directed by sensorDelay var
  delay(sensorDelay);

  // now take a measurement from the sensor, timing a low pulse on the sensor's "out" pin
  readPulse = pulseIn(taosOutPin, LOW, 80000);

  //if the pulseIn times out, it returns 0 and that throws off numbers. just cap it at 80k if it happens
  if (readPulse < .1) {
    readPulse = 80000;
  }

  //turn off color sensor and LEDs to save power
  //taosMode(0);

  // return the pulse value back to whatever called for it...
  return readPulse;

}

// Operation modes area, controlled by hi/lo settings on S0 and S1 pins.
//setting mode to zero will put taos into low power mode. taosMode(0);

void taosMode(int mode) {

  if (mode == 0) {
    //power OFF mode-  LED off and both channels "low"
    digitalWrite(LED, LOW);
    digitalWrite(S0, LOW); //S0
    digitalWrite(S1, LOW); //S1
    //  Serial1.println("mOFFm");

  } else if (mode == 1) {
    //this will put in 1:1, highest sensitivity
    digitalWrite(S0, HIGH); //S0
    digitalWrite(S1, HIGH); //S1
    // Serial1.println("m1:1m");

  } else if (mode == 2) {
    //this will put in 1:5
    digitalWrite(S0, HIGH); //S0
    digitalWrite(S1, LOW); //S1
    //Serial1.println("m1:5m");

  } else if (mode == 3) {
    //this will put in 1:50
    digitalWrite(S0, LOW); //S0
    digitalWrite(S1, HIGH); //S1
    //Serial1.println("m1:50m");
  }

  return;

}

void TCS3200setup() {

  //initialize pins
  pinMode(LED, OUTPUT); //LED pinD

  //color mode selection
  pinMode(S2, OUTPUT); //S2 pinE
  pinMode(S3, OUTPUT); //s3 pinF

  //color response pin (only actual input from taos)
  pinMode(taosOutPin, INPUT); //taosOutPin pinC

  //communication freq (sensitivity) selection
  pinMode(S0, OUTPUT); //S0 pinB
  pinMode(S1, OUTPUT); //S1 pinA

  return;

}

void DropPeg()
{
  tilt.write(down);  //set back down
  delay(1000);
  grip.write(openit);   //release
  delay(1000);
  tilt.write(up);   //lift to ~90 degrees
  delay(2000);
}

void DriveCenterSense(int Speed, int Distance) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideFRPulse, SideFREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while (FrontSideSense < 300 && BackSideSense < 300 && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 160) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 115) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveBackSense(int Speed, int Distance) {
  boolean Reverse = true;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    Reverse = false;
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, Speed + 0);
      analogWrite(MotorA2, Speed);
    }
    BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, 0);
      analogWrite(MotorA2, Speed);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, Speed + 0);
      analogWrite(MotorA2, 0);
    }
    analogWrite (MotorB2, Speed + 0);
    analogWrite (MotorA2, Speed);
    BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  if (Reverse == false) {
    analogWrite(MotorA1, Speed/4);
    analogWrite(MotorB1, Speed/4);
    delay(25);
    analogWrite(MotorA1, Speed/2);
    analogWrite(MotorB1, Speed/2);
    delay(50);
    analogWrite(MotorA1, Speed*3/4);
    analogWrite(MotorB1, Speed*3/4);
    delay(100);
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed + 0);
    delay(100);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void ReturnPeg() {
  if (PegColor == false) {
    TurnStraight(190, WestFace);
    delay(200);
    DriveFrontBlind(255, 35);
    DriveRightSense(fwd, 28);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackBlind(fwd + 10, 100);
    delay(200);
    DropPeg();
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackSenseIMU(fwd + 10, 1, WestFace);
    DriveBackBlind(fwd + 10, 300);
    delay(300);
    TurnLeftBurst();
    TurnLeft(145, SouthFace);
    DriveBackBlind(255, 50);
    TurnLeftBurst();
    TurnLeft(145, EastFace);
    TurnStraight(190, EastFace);
  }
  else {
    TurnLeftBurst();
    TurnLeft(145, SouthFace);
    TurnStraight(190, SouthFace);
    delay(300);
    DriveFrontBlind(255, 35);
    DriveFrontSense(120, 27);
    delay(300);
    TurnLeftBurst();
    TurnLeft(145, EastFace);
    TurnStraight(190, EastFace);
    delay(300);
    DriveFrontBlind(255, 35);
    DriveFrontBlind(fwd, 800);
    TurnStraight(190, EastFace);
//    DriveCenterSenseTime(fwd, 28, 25);
//    delay(200);
//    TurnStraight(190, EastFace);
//    delay(200);
//    DriveFrontBlind(255, 35);
    DriveCenterSenseIMU(fwd + 10, 28, EastFace);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackBlind(fwd + 10, 100);
    delay(200);
    DropPeg();
    delay(300);
    TurnStraight(190, EastFace);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackSenseIMU(fwd + 10, 1, EastFace);
    DriveBackBlind(fwd + 10, 50);
    delay(300);
    TurnLeftBurst();
    TurnLeft(145, NorthFace);
    TurnStraight(190, NorthFace);
    delay(300);
    DriveFrontBlind(255, 25);
    DriveFrontSense(120, 27);
    delay(300);
    TurnRightBurst();
    TurnRight(145, EastFace);
    TurnStraight(145, EastFace);
  }
}

void TurnStraight(int Speed, float TargetDirection) {
  int j = 0;
  float CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  Serial1.println(CurrentTurn);
  if (CurrentTurn > 0) {
  while (abs(CurrentTurn) > 5) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    j++;
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB2, Speed);
  }
  }
  else {
  while (abs(CurrentTurn) > 5) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    j++;
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveFrontBlind(255, 150);
    }
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
  }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);
  delay(100);
}

void DriveForwardPeg2(int Speed) {
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  delay(7);
  Serial1.println(BackSideSense);
  while (BackSideSense > 200) {
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
    delay(7);
    Serial1.println(BackSideSense);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveLeftSenseClose(int Speed, int Distance) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 60 && FrontSideSense < 90 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 50) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void TurnLeftBurst() {
  analogWrite(MotorA1, 255);
  analogWrite(MotorB2, 255);
  delay(10);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
}

void TurnRightBurst() {
  analogWrite(MotorA2, 255);
  analogWrite(MotorB1, 255);
  delay(10);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
}

void DriveCenterSenseTime(int Speed, int Distance, int Time) {
  int count = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideFRPulse, SideFREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while (FrontSideSense < 300 && BackSideSense < 300 && FrontIR > Distance && count < Time) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance && count < Time) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, Speed);
      count++;
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 160) && FrontSideSense < 300 && FrontIR > Distance && count < Time) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      count++;
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 115) && FrontSideSense < 300 && FrontIR > Distance && count < Time) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, 0);
      count++;
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveFrontIMUMixed(int Speed1, int Speed2, int Time, float TargetDirection, int Distance) {
  int i = 0;
  float CurrentTurn = 0;
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  while (i < Time && FrontIR > Distance) {
    analogWrite (MotorB1, Speed1);
    analogWrite (MotorA1, Speed2);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 5) {
      TurnStraight(255, TargetDirection);
    }
    delay(1);
    i++;
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void DriveFrontSearch(int Speed1, int Speed2) {
  float CurrentTurn = 0;
  FrontSideSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
  Serial1.print("Front Left: ");
  Serial1.println(FrontSideSense);
  while (FrontSideSense > 300) {
    analogWrite (MotorB1, Speed1);
    analogWrite (MotorA1, Speed2);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - WestFace;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 5) {
      TurnStraight(255, WestFace);
    }
    delay(7);
    FrontSideSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
    Serial1.print("Front Left: ");
    Serial1.println(FrontSideSense);
  }
  delay(10);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void FindPegTurf(int Speed) {
  int FrontRight, FrontLeft, FrontDiff;
  char report[80]; //Variable to generate Serial1 Output
  delay(50);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
  delay(10);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
  Serial1.println(report);
  Serial1.println("Beginning Loop");
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);

  while ((FrontRight >= 100) && (FrontLeft >= 100)) {
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed);
    delay(110);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    delay(500);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
    Serial1.println("Not yet reached Peg");
  }
  delay(10);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
  delay(7);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  FrontDiff = FrontRight - FrontLeft;
  while (abs(FrontDiff) >= 12) {
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    while (FrontRight > FrontLeft && (FrontRight - FrontLeft < 50)) {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left both see peg");
    }
    while (FrontLeft > FrontRight && (FrontLeft - FrontRight < 50)) {
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(7);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right both see peg");
    }

    while (FrontRight > FrontLeft && (FrontRight - FrontLeft > 50))
    {
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA2, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left Right not see peg");
    }

    while (FrontLeft > FrontRight && (FrontLeft - FrontRight > 50))
    {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right Left not see peg");
    }
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    FrontDiff = FrontRight - FrontLeft;
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
  }
  while ((FrontRight >= 50) && (FrontLeft >= 50)) {
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed);
    delay(70);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    delay(500);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
    Serial1.println("Not yet reached Peg");
  }
  delay(10);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
  delay(10);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  FrontDiff = FrontRight - FrontLeft;
  while (abs(FrontDiff) >= 7) {
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    while (FrontRight > FrontLeft && (FrontRight < 250)) {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left both see peg");
    }
    while (FrontLeft > FrontRight && (FrontLeft < 250)) {
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right both see peg");
    }

    while (FrontRight > FrontLeft && (FrontRight > 250))
    {
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA2, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left Right not see peg");
    }

    while (FrontLeft > FrontRight && (FrontLeft > 250))
    {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right Left not see peg");
    }
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    FrontDiff = FrontRight - FrontLeft;
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
  }
  DriveBackBlind(Turf, 65);
  byte IRArray[15];
  for (byte i = 0; i < 15; i++) {
    IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
    ave.push(IRArray[i]);
    Serial1.print(IRArray[i]);
    delay(2);
  }
  FrontIR = ave.mode();
  Serial1.println(" ");
  Serial1.println(FrontIR);
  Serial1.println("Pre-grab");
  while (FrontIR > 12 || FrontIR < 11) {
    while (FrontIR > 12 && FrontIR < 26) {
      Serial1.println("Forward");
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, Speed);
      delay(35);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
    while (FrontIR < 11 || FrontIR > 25) {
      Serial1.println("Backward");
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB2, Speed);
      delay(25);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
  }
  grip.write(openit);
  delay(500);
  tilt.write(down);
  delay(1000);
  analogWrite(MotorA1, Speed);
  analogWrite(MotorB1, Speed);
  delay(85);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  delay(500);
  grip.write(closeit);  //grab
  delay(1000);
  tilt.write(down-1);
  delay(500);   
  tilt.write(up);   //lift to ~90 degrees
  
  delay(1000);
  while (loopcontrol > 0) {
    if (detectColor(taosOutPin) == 0) {
      colorcounter++;
    }
    loopcontrol--;
    delay(100);
  }
  if (colorcounter > 0) {
    PegColor = true;
    Serial1.println("Red");
  }
  else {
    PegColor = false;
    Serial1.println("Yellow");
  }
  colorcounter = 0;
  loopcontrol = 10;
  taosMode(0);
  digitalWrite(LED,HIGH);
  return;
}

void GetPeg3() {
  TurnRight(145, WestFace + 10.5);
  delay(200);
  DriveFrontBlind(tfwd, 150);
  FrontSideSense = ReadSonicSensor(FrontLPulse, FrontLEcho);  
  if (FrontSideSense > 200) {
    TurnRight(145, WestFace + 25.5);
    FindPegTurfTimeout(Turf);
  }
  else {
    TurnRight(145, WestFace + 25.5);
    FindPegTurfTimeout(Turf);
  }
  if (RiverPeg == false) {
    DriveFrontBlind(tfwd, 300);
    TurnLeft(255, SouthFace);
    delay(200);
    DriveFrontSense(tfwd, 26);
    delay(200);
    TurnLeft(145, NorthFace + 85);
    delay(200);
    DriveRightSenseMixed(fwd+50, tfwd+30, 1);
    DriveBackBlind(tfwd, 100);
    delay(200);
    TurnRight(145, SouthFace - 40);
    delay(200);
    DriveFrontBlind(fwd+10, 600);
    DriveFrontSense(fwd-20, 26);
    DriveBackBlind(fwd, 100);
    delay(200);
    TurnRight(145, WestFace);
    DriveLeftSense(fwd, 1);
    delay(200);
    TurnLeft(145, SouthFace);
//    DriveForwardPeg2RightSide(fwd);
//    TurnStraight(255, SouthFace);
//    DriveFrontBlind(fwd, 500);
//    DriveForwardPeg2RightSide(fwd);
    delay(200);
    DriveFrontBlind(255, 35);
    DriveFrontSense(fwd, 25);
    delay(200);
    TurnRight(145, WestFace);
    delay(200);
    DriveFrontBlind(fwd, 50);
    delay(200);
    TurnStraight(230, WestFace+2);
    delay(200);
    DriveFrontBlind(255, 35);
    DriveLeftSenseCloseIMU(fwd, 1, WestFace);
    ReturnPegFinal();
    RobotPark();
  }
 else {
    if (ExtraSense == 32222) {
      DriveBackBlind(tfwd, 50);
    }
    TurnRight(255, NorthFace + 45);
    delay(200);
    DriveFrontIMUMixed(tfwd, tfwd, 1000, NorthFace + 45, 1);
    DriveFrontIMUMixed(tfwd, tfwd, 1000, NorthFace + 45, 27);
    DriveBackBlind(tfwd, 100);
    delay(200);
    TurnLeft(145, WestFace);
    delay(200);
 }
}

void DriveRightSenseMixed(int Speed1, int Speed2, int Distance) {
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorB1, Speed2 + 0);
      analogWrite(MotorA1, Speed1);
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed2 + 0);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
            snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
            Serial1.println(report);
      analogWrite(MotorA1, Speed1);
      analogWrite(MotorB1, 0);
    }
    analogWrite (MotorB1, Speed2 + 0);
    analogWrite (MotorA1, Speed1);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
        Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed1/4);
  analogWrite(MotorB2, Speed2/4);
  delay(25);
  analogWrite(MotorA2, Speed1/2);
  analogWrite(MotorB2, Speed2/2);
  delay(50);
  analogWrite(MotorA2, Speed1*3/4);
  analogWrite(MotorB2, Speed2*3/4);
  delay(100);
  analogWrite(MotorA2, Speed1);
  analogWrite(MotorB2, Speed2);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveLeftSenseMixed(int Speed1, int Speed2, int Distance) {
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed2 + 0);
      analogWrite(MotorA1, Speed1);
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed1);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed2 + 0);
      analogWrite(MotorA1, 0);
    }
    analogWrite (MotorB1, Speed2);
    analogWrite (MotorA1, Speed1);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed1/4);
  analogWrite(MotorB2, Speed2/4);
  delay(25);
  analogWrite(MotorA2, Speed1/2);
  analogWrite(MotorB2, Speed2/2);
  delay(50);
  analogWrite(MotorA2, Speed1*3/4);
  analogWrite(MotorB2, Speed2*3/4);
  delay(100);
  analogWrite(MotorA2, Speed1);
  analogWrite(MotorB2, Speed2);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveForwardPeg2RightSide(int Speed) {
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  delay(7);
  Serial1.println(BackSideSense);
  while (BackSideSense > 150) {
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
    delay(7);
    Serial1.println(BackSideSense);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveFrontSearchPeg4(int Speed, int Distance) {
  float CurrentTurn = 0;
  byte i = 0;
  bool flag = false;
  ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
  delay(5);
  BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
  delay(5);
  FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
  while (BackSideSense < Distance && FrontSideSense > 350 && ExtraSense > 100) {
      analogWrite(MotorA1, 255);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - NorthFace;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 20) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, NorthFace);
      }
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
      delay(5);
      ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
      i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
    }
  analogWrite(MotorA1, 0);
  delay(5);
  FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
  delay(5);
  ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
  Serial1.print("Front Right: ");
  Serial1.println(FrontSideSense);
  Serial1.print("Front Left: ");
  Serial1.println(ExtraSense);
  i = 0;
  while (FrontSideSense > 350 && ExtraSense > 100) {
    flag = true;
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    analogWrite (MotorB2, 0);
    analogWrite (MotorA2, 0);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - NorthFace;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 5) {
      Serial1.println("I'm trying to turn");
      TurnStraight(255, NorthFace);
    }
    delay(5);
    FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
    Serial1.print("Front Right: ");
    Serial1.println(FrontSideSense);
    Serial1.print("Front Left: ");
    Serial1.println(ExtraSense);
    i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
  }
  delay(10);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  delay(100);
  i = 0;
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  if (FrontIR < 28 && flag == true) {
    DriveBackBlind(tfwd, 150);
    TurnLeft(145, WestFace);
    FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
    ExtraSense = 1000;
    Serial1.print("Front Right: ");
    Serial1.println(FrontSideSense);
    while (FrontSideSense > 200) {
      analogWrite (MotorB1, Speed);
      analogWrite (MotorA1, Speed);
      analogWrite (MotorB2, 0);
      analogWrite (MotorA2, 0);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - WestFace;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 5) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, WestFace);
      }
      delay(5);
      FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
      Serial1.print("Front Right: ");
      Serial1.println(FrontSideSense);
      i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
    }
    delay(10);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
  }
  return;
}

void ReturnPeg4() {
    if (FrontSideSense > 300 && ExtraSense > 160) {
    TurnLeft(145, WestFace + 70);
    DriveFrontBlind(tfwd, 500);
    TurnLeft(145, WestFace + 37);
    FindPegTurf(Turf);
    TurnRight(145, EastFace);
    DriveFrontSense(tfwd, 26);
    TurnRight(145, SouthFace);
  }
  else if (FrontSideSense > 200 && ExtraSense > 160) {
    TurnLeft(145, WestFace + 70);
    DriveFrontBlind(tfwd, 300);
    TurnLeft(145, WestFace + 50);
    FindPegTurf(Turf);
    TurnRight(145, EastFace);
    DriveFrontSense(tfwd, 26);
    TurnRight(145, SouthFace);
  }
  else {
    TurnStraight(255, NorthFace);
    DriveBackBlind(255, 150);
    FindPegTurf(Turf);
    TurnRight(145, EastFace);
    DriveFrontSense(tfwd, 26);
    TurnRight(145, SouthFace);
  }
  LeftWallSpace(200);
  DriveLeftSenseIMUSpecial(tfwd + 20, 26, SouthFace);
  delay(200);
  DriveBackBlind(255, 180);
  TurnRight(145, SouthFace + 68);
  delay(200);
  DriveFrontBlind(tfwd, 250);
  DriveFrontSpecial(tfwd);
//  DriveFrontBlind(tfwd, 50);
  delay(200);
  TurnLeft(145, SouthFace);
  delay(200);
  DriveFrontSense(fwd, 25);
  delay(200);
  TurnRight(145, WestFace+2);
  delay(200);
  DriveFrontBlind(255, 35);
  DriveLeftSenseCloseIMU(fwd, 1, WestFace);
  ReturnPeg();
}

void DriveRightSenseFinal(int Speed) {
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(5);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  delay(5);
  FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(5);
  while (FrontIR > 500) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  while (FrontIR < 500 && FrontIR > 350) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  while (FrontIR > 500 || FrontIR < 350) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(5);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(5);
    FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(5);
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
    while (FrontIR < 500 && FrontIR > 350) {
      while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR < 300 && FrontIR > 100) {
        FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
        delay(5);
        BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
        delay(5);
        FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
        delay(5);
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
        Serial1.println(report);
        analogWrite(MotorB1, Speed + 0);
        analogWrite(MotorA1, Speed);
      }
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
        FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
        delay(5);
        BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
        delay(5);
        FrontIR = (480 - analogRead(A14)) / 25 + 27;
        delay(5);
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
        Serial1.println(report);
        analogWrite(MotorA1, 0);
        analogWrite(MotorB1, Speed + 0);
      }
      while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR < 300 && FrontIR > 100) {
        FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
        delay(5);
        BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
        delay(5);
        FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
        delay(5);
        snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
        Serial1.println(report);
        analogWrite(MotorA1, Speed);
        analogWrite(MotorB1, 0);
      }
      analogWrite (MotorB1, Speed + 0);
      analogWrite (MotorA1, Speed);
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontIR = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(5);
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
      Serial1.println(report);
    }
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    analogWrite(MotorA2, Speed/4);
    analogWrite(MotorB2, Speed/4);
    delay(25);
    analogWrite(MotorA2, Speed/2);
    analogWrite(MotorB2, Speed/2);
    delay(50);
    analogWrite(MotorA2, Speed*3/4);
    analogWrite(MotorB2, Speed*3/4);
    delay(100);
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB2, Speed);
    delay(100);
    analogWrite(MotorA2, 0);
    analogWrite(MotorB2, 0);
    return;
}

void DriveBackSenseMixed(int Speed1, int Speed2, int Distance) {
  boolean Reverse = true;
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    Reverse = false;
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, Speed2 + 0);
      analogWrite(MotorA2, Speed1);
    }
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, 0);
      analogWrite(MotorA2, Speed1);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB2, Speed2 + 0);
      analogWrite(MotorA2, 0);
    }
    analogWrite (MotorB2, Speed2 + 0);
    analogWrite (MotorA2, Speed1);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  if (Reverse == false) {
    analogWrite(MotorA1, Speed1/4);
    analogWrite(MotorB1, Speed2/4);
    delay(25);
    analogWrite(MotorA1, Speed1/2);
    analogWrite(MotorB1, Speed2/2);
    delay(50);
    analogWrite(MotorA1, Speed1*3/4);
    analogWrite(MotorB1, Speed2*3/4);
    delay(100);
    analogWrite(MotorA1, Speed1);
    analogWrite(MotorB1, Speed2 + 0);
    delay(100);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void LeftWallSpace(int Distance) {
  float CurrentTurn = 0;
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  while (FrontSideSense < Distance) {
    analogWrite(MotorA1, 255);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - SouthFace;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 10) {
      Serial1.println("I'm trying to turn");
      TurnStraight(255, SouthFace);
    }
    delay(5);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  }
  analogWrite(MotorA1, 0);
}

void DriveFrontSpecial(int Speed) {
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    ExtraSense = 3000;
    Serial1.print("BR Sensor: ");
    Serial1.println(BackSideSense);
    while (BackSideSense <= ExtraSense + 40) {
      analogWrite (MotorB1, Speed);
      analogWrite (MotorA1, Speed);
      analogWrite (MotorB2, 0);
      analogWrite (MotorA2, 0);
      delay(5);
      ExtraSense = BackSideSense;
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      Serial1.print("BR Sensor: ");
      Serial1.println(BackSideSense);
    }
    delay(10);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    analogWrite(MotorA2, Speed/4);
    analogWrite(MotorB2, Speed/4);
    delay(25);
    analogWrite(MotorA2, Speed/2);
    analogWrite(MotorB2, Speed/2);
    delay(50);
    analogWrite(MotorA2, Speed*3/4);
    analogWrite(MotorB2, Speed*3/4);
    delay(100);
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB2, Speed);
    delay(100);
    analogWrite(MotorA2, 0);
    analogWrite(MotorB2, 0);
}

void FindPegTurfTimeout(int Speed) {
  int FrontRight, FrontLeft, FrontDiff;
  byte i = 0;
  char report[80]; //Variable to generate Serial1 Output
  delay(50);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
  delay(10);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
  Serial1.println(report);
  Serial1.println("Beginning Loop");
  analogWrite(MotorA1, 0);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorB2, 0);

  while ((FrontRight >= 100) && (FrontLeft >= 100)) {
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed);
    delay(110);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    delay(500);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
    Serial1.println("Not yet reached Peg");
    i++;
    if (i == 4) {
      TurnStraight(255, WestFace + 10.5);
    }
    if (i == 10) {
      RiverPeg = true;
      return;
    }
  }
  delay(10);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
  delay(7);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  FrontDiff = FrontRight - FrontLeft;
  while (abs(FrontDiff) >= 12) {
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    while (FrontRight > FrontLeft && (FrontRight - FrontLeft < 50)) {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left both see peg");
      i++;
      if (i == 20) {
        RiverPeg = true;
        return;
    }
    }
    while (FrontLeft > FrontRight && (FrontLeft - FrontRight < 50)) {
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(7);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right both see peg");
      i++;
      if (i == 20) {
        RiverPeg = true;
        return;
    }
    }

    while (FrontRight > FrontLeft && (FrontRight - FrontLeft > 50))
    {
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA2, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left Right not see peg");
      i++;
      if (i == 20) {
        RiverPeg = true;
        return;
    }
    }

    while (FrontLeft > FrontRight && (FrontLeft - FrontRight > 50))
    {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right Left not see peg");
      i++;
      if (i == 20) {
        RiverPeg = true;
        return;
    }
    }
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    FrontDiff = FrontRight - FrontLeft;
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
  }
  while ((FrontRight >= 50) && (FrontLeft >= 50)) {
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed);
    delay(70);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
    delay(500);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho);
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
    Serial1.println("Not yet reached Peg");
    i++;
      if (i == 24) {
        RiverPeg = true;
        ExtraSense = 32222;
        return;
    }
  }
  delay(10);
  FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
  delay(10);
  FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
  FrontDiff = FrontRight - FrontLeft;
  while (abs(FrontDiff) >= 7) {
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    while (FrontRight > FrontLeft && (FrontRight - FrontLeft < 50)) {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left both see peg");
      i++;
      if (i == 30) {
        RiverPeg = true;
        ExtraSense = 32222;
        return;
    }
    }
    while (FrontLeft > FrontRight && (FrontLeft - FrontRight < 50)) {
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right both see peg");
      i++;
      if (i == 30) {
        RiverPeg = true;
        return;
    }
    }

    while (FrontRight > FrontLeft && (FrontRight - FrontLeft > 50))
    {
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA2, Speed);
      delay(50);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Right>Left Right not see peg");
      i++;
      if (i == 30) {
        RiverPeg = true;
        return;
    }
    }

    while (FrontLeft > FrontRight && (FrontLeft - FrontRight > 50))
    {
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB2, Speed);
      delay(50);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
      delay(10);
      FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
      snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
      Serial1.println(report);
      Serial1.println("Left>Right Left not see peg");
      i++;
      if (i == 30) {
        RiverPeg = true;
        return;
    }
    }
    delay(10);
    FrontLeft = ReadSonicSensor(FrontLPulse, FrontLEcho) + 11;
    delay(10);
    FrontRight = ReadSonicSensor(FrontRPulse, FrontREcho);
    FrontDiff = FrontRight - FrontLeft;
    snprintf(report, sizeof(report), "Left: {%6d}    Right: {%6d}", FrontLeft, FrontRight);
    Serial1.println(report);
  }
  DriveBackBlind(Turf, 65);
  byte IRArray[15];
  for (byte i = 0; i < 15; i++) {
    IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
    ave.push(IRArray[i]);
    Serial1.print(IRArray[i]);
    delay(2);
  }
  FrontIR = ave.mode();
  Serial1.println(" ");
  Serial1.println(FrontIR);
  Serial1.println("Pre-grab");
  while (FrontIR > 12 || FrontIR < 11) {
    while (FrontIR > 12 && FrontIR < 26) {
      Serial1.println("Forward");
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, Speed);
      delay(35);
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
    while (FrontIR < 11 || FrontIR > 25) {
      Serial1.println("Backward");
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB2, Speed);
      delay(25);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      for (byte i = 0; i < 15; i++) {
        IRArray[i] = (480 - analogRead(A14)) / 25 + 27;
        ave.push(IRArray[i]);
        Serial1.print(IRArray[i]);
        delay(2);
      }
      FrontIR = ave.mode();
      Serial1.println(" ");
      Serial1.println(FrontIR);
    }
  }
  grip.write(openit);
  delay(500);
  tilt.write(down);
  delay(1000);
  analogWrite(MotorA1, Speed);
  analogWrite(MotorB1, Speed);
  delay(85);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  delay(500);
  grip.write(closeit);  //grab
  delay(1000);
  tilt.write(down-1);
  delay(500);   
  tilt.write(up);   //lift to ~90 degrees
  
  delay(1000);
  while (loopcontrol > 0) {
    if (detectColor(taosOutPin) == 0) {
      colorcounter++;
    }
    loopcontrol--;
    delay(100);
  }
  if (colorcounter > 0) {
    PegColor = true;
    Serial1.println("Red");
  }
  else {
    PegColor = false;
    Serial1.println("Yellow");
  }
  colorcounter = 0;
  loopcontrol = 10;
  taosMode(0);
  digitalWrite(LED,HIGH);
  return;
}

void DriveFrontSearchFinal(int Speed, int Distance) {
  float CurrentTurn = 0;
  bool flag = false;
  byte i = 0;
  ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
  delay(5);
  BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
  delay(5);
  FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
  while (BackSideSense < Distance && FrontSideSense > 375 && ExtraSense > 100) {
      analogWrite(MotorA1, 255);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - WestFace;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 20) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, WestFace);
      }
      delay(5);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(5);
      FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
      delay(5);
      ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
      i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
    }
  analogWrite(MotorA1, 0);
  delay(5);
  FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
  delay(5);
  ExtraSense = ReadSonicSensor(FrontLPulse, FrontLEcho);
  Serial1.print("Front Right: ");
  Serial1.println(FrontSideSense);
  Serial1.print("Front Left: ");
  Serial1.println(ExtraSense);
  i = 0;
  while (FrontSideSense > 375 && ExtraSense > 100) {
    flag = true;
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    analogWrite (MotorB2, 0);
    analogWrite (MotorA2, 0);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - WestFace;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 5) {
      Serial1.println("I'm trying to turn");
      TurnStraight(255, WestFace);
    }
    delay(5);
    FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
    Serial1.print("Front Right: ");
    Serial1.println(FrontSideSense);
    Serial1.print("Front Left: ");
    Serial1.println(ExtraSense);
    i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
  }
  delay(10);
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  delay(100);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  i = 0;
  if (FrontIR < 24 && flag == true) {
    TurnLeft(145, SouthFace);
    FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
    ExtraSense = 1000;
    Serial1.print("Front Right: ");
    Serial1.println(FrontSideSense);
    while (FrontSideSense > 200) {
      analogWrite (MotorB1, Speed);
      analogWrite (MotorA1, Speed);
      analogWrite (MotorB2, 0);
      analogWrite (MotorA2, 0);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - SouthFace;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 5) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, SouthFace);
      }
      delay(5);
      FrontSideSense = ReadSonicSensor(FrontRPulse, FrontREcho);
      Serial1.print("Front Right: ");
      Serial1.println(FrontSideSense);
      i++;
      if (i == 150) {
        i = 0;
        DriveBackBlind(255, 100);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
        delay(50);
        DriveFrontBlind(255, 60);
      }
    }
    delay(10);
    analogWrite(MotorA1, 0);
    analogWrite(MotorB1, 0);
  }
  return;
}

void ReturnPegFinal() {
  if (PegColor == false) {
    TurnStraight(230, WestFace);
    delay(200);
    DriveFrontBlind(255, 35);
    DriveRightSense(fwd, 28);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackBlind(fwd + 10, 150);
    delay(200);
    DropPeg();
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackSense(fwd + 10, 1);
    DriveBackBlind(fwd + 10, 300);
    delay(300);
    TurnLeftBurst();
    TurnLeft(145, SouthFace);
    TurnStraight(190, SouthFace);
    DriveFrontSense(fwd, 26);
    delay(200);
    TurnRight(145, WestFace);
    TurnStraight(190, WestFace);
    delay(200);
    DriveFrontSense(fwd, 26);
  }
  else {
    TurnLeftBurst();
    TurnLeft(145, SouthFace);
    TurnStraight(190, SouthFace);
    delay(300);
    DriveFrontBlind(255, 35);
    DriveFrontSense(120, 27);
    delay(300);
    TurnLeftBurst();
    TurnLeft(145, EastFace);
    TurnStraight(190, SouthFace);
    delay(300);
    DriveFrontBlind(255, 35);
    DriveFrontBlind(fwd, 800);
//    DriveCenterSenseTime(fwd, 28, 25);
//    delay(200);
//    TurnStraight(230, EastFace);
//    delay(200);
//    DriveFrontBlind(255, 35);
    DriveCenterSenseIMU(fwd + 10, 28, EastFace);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackBlind(fwd + 10, 150);
    delay(200);
    DropPeg();
    delay(300);
    TurnStraight(230, EastFace);
    delay(300);
    DriveBackBlind(255, 35);
    DriveBackSenseIMU(fwd + 10, 1, EastFace);
    DriveBackBlind(fwd + 10, 700);
  }
}

void DriveLeftSenseIMU(int Speed, int Distance, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  float CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  if (abs(CurrentTurn) > 10) {
    Serial1.println("I'm trying to turn");
    TurnStraight(255, TargetDirection);
  }
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 10) {
      Serial1.println("I'm trying to turn");
      TurnStraight(255, TargetDirection);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void TurnRightSpecial(int Speed, float TargetDirection) {
  int TempSpeed = Speed;
  boolean Overshoot = true;
  float CurrentTurn = ReadCompass();
  byte i = 10;
  int j = 0;
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  float InitialTurn = CurrentTurn;
  float TempTurn;
  Serial1.println("Before Turning");
  Serial1.println(CurrentTurn);
  while (abs(CurrentTurn) > 30) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      DriveFrontBlind(255, 400);
      delay(200);
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(200);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(200);
      analogWrite(MotorA2, Speed);
      analogWrite(MotorB1, Speed);
      delay(200);
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(200);
    }
  }
  j = 0;
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  delay(200);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  Serial1.println("Undershoot?");
  Serial1.println(CurrentTurn);
  i = 10;
  while (CurrentTurn < -5) {
    Overshoot = false;
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA2, Speed);
    analogWrite(MotorB1, Speed);
    i--;
    j++;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 10;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 200) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA2, 0);
      analogWrite(MotorB1, 0);
      delay(500);
      DriveFrontBlind(255, 400);
    }
  }
  j = 0;
  analogWrite(MotorA2, 0);
  analogWrite(MotorB1, 0);
  delay(1000);
  CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  InitialTurn = CurrentTurn;
  i = 20;
  Serial1.println("Overshoot?");
  Serial1.println(CurrentTurn);
  Speed = TempSpeed;
  if (abs(CurrentTurn) > 25 || Overshoot == true) {
    Speed = Speed * .6;
  }
  while (abs(CurrentTurn) > 5) {
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB2, Speed);
    i--;
    TempTurn = CurrentTurn - InitialTurn;
    if (i == 0 && abs(TempTurn) < 2) {
      i = 20;
      InitialTurn = CurrentTurn;
      Speed = Speed * 1.2;
    }
    if (i == 180) {
      InitialTurn = CurrentTurn;
      i = 10;
    }
    if (j == 1750) {
      j = 0;
      analogWrite(MotorA1, 0);
      analogWrite(MotorB2, 0);
      delay(500);
      DriveBackBlind(255, 400);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB2, 0);
  delay(100);
}

void DriveLeftSenseIMUCity(int Speed, int Distance, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  float CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  if (abs(CurrentTurn) > 10) {
    Serial1.println("I'm trying to turn");
    TurnStraight(190, TargetDirection);
  }
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 10) {
      Serial1.println("I'm trying to turn");
      TurnStraight(190, TargetDirection);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveLeftSenseCloseIMU(int Speed, int Distance, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  float CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  if (abs(CurrentTurn) > 10) {
    Serial1.println("I'm trying to turn");
    TurnStraight(190, TargetDirection);
  }
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 60 && FrontSideSense < 90 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 90) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 50) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 10) {
      Serial1.println("I'm trying to turn");
      TurnStraight(190, TargetDirection);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveBackSenseIMU(int Speed, int Distance, float TargetDirection) {
  boolean Reverse = true;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    Reverse = false;
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB2, Speed + 0);
      analogWrite(MotorA2, Speed);
    }
    BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB2, 0);
      analogWrite(MotorA2, Speed);
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && FrontIR > Distance) {
      BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB2, Speed + 0);
      analogWrite(MotorA2, 0);
    }
    analogWrite (MotorB2, Speed + 0);
    analogWrite (MotorA2, Speed);
    BackSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    FrontSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  if (Reverse == false) {
    analogWrite(MotorA1, Speed/4);
    analogWrite(MotorB1, Speed/4);
    delay(25);
    analogWrite(MotorA1, Speed/2);
    analogWrite(MotorB1, Speed/2);
    delay(50);
    analogWrite(MotorA1, Speed*3/4);
    analogWrite(MotorB1, Speed*3/4);
    delay(100);
    analogWrite(MotorA1, Speed);
    analogWrite(MotorB1, Speed + 0);
    delay(100);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void DriveCenterSenseIMU(int Speed, int Distance, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideFRPulse, SideFREcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  while (FrontSideSense < 300 && BackSideSense < 300 && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 160) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 115) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB1, Speed);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}

void DriveRightNotLeftSenseIMU(int Speed, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBRPulse, SideBREcho);
  delay(2);
  int BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
  while ((FrontSideSense < 300 || BackSideSense < 300) && BackOppositeSideSense > 250) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 100 && FrontSideSense < 150 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Straight", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 150) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Runaway", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorA1, 0);
      analogWrite(MotorB1, Speed + 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 90) && FrontSideSense < 300 && BackOppositeSideSense > 250) {
      FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
      delay(2);
      BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Crash", FrontSideSense, BackSideSense, BackOppositeSideSense);
      Serial1.println(report);
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(190, TargetDirection);
      }
      analogWrite(MotorA1, Speed);
      analogWrite(MotorB1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
    }
    analogWrite (MotorB1, Speed + 0);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFRPulse, SideFREcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBRPulse, SideBREcho);
    delay(2);
    BackOppositeSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
        snprintf(report, sizeof(report), "Front: {%6d} Back: {%6d} Opp: {%6d} Broken", FrontSideSense, BackSideSense, BackOppositeSideSense);
        Serial1.println(report);
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  return;
}

void DriveLeftSenseIMUSpecial(int Speed, int Distance, float TargetDirection) {
  byte i = 0;
  FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
  delay(2);
  BackSideSense = ReadSonicSensor (SideBLPulse, SideBLEcho);
  FrontIR = (480 - analogRead(A14)) / 25 + 27;
  snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Start", FrontSideSense, BackSideSense);
  Serial1.println(report);
  float CurrentTurn = ReadCompass();
  CurrentTurn = CurrentTurn + 360 - TargetDirection;
  while (CurrentTurn > 180) {
    CurrentTurn = CurrentTurn - 360;
  }
  if (abs(CurrentTurn) > 10) {
    Serial1.println("I'm trying to turn");
    TurnStraight(255, TargetDirection);
  }
  while ((FrontSideSense < 300 || BackSideSense < 300) && FrontIR > Distance) {
    while (abs(FrontSideSense - BackSideSense) < 3 && FrontSideSense > 150 && FrontSideSense < 220 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Straight", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    delay(2);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    while (((FrontSideSense > BackSideSense) || FrontSideSense > 220) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Runaway", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, 0);
      analogWrite(MotorA1, Speed);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    while (((FrontSideSense < BackSideSense) || FrontSideSense < 150) && FrontSideSense < 300 && FrontIR > Distance) {
      FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
      delay(2);
      BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
      FrontIR = (480 - analogRead(A14)) / 25 + 27;
      snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}     Crash", FrontSideSense, BackSideSense);
      Serial1.println(report);
      analogWrite(MotorB1, Speed + 0);
      analogWrite(MotorA1, 0);
      i++;
      if (i == 150) {
        i = 0;
        DriveFrontBlind(255, 60);
        Serial1.println("Whooooooooooooooooooaaaaaaaaaaaaaaah!");
        Speed = Speed * 1.1;
      }
      CurrentTurn = ReadCompass();
      CurrentTurn = CurrentTurn + 360 - TargetDirection;
      while (CurrentTurn > 180) {
        CurrentTurn = CurrentTurn - 360;
      }
      if (abs(CurrentTurn) > 10) {
        Serial1.println("I'm trying to turn");
        TurnStraight(255, TargetDirection);
      }
    }
    analogWrite (MotorB1, Speed);
    analogWrite (MotorA1, Speed);
    FrontSideSense = ReadSonicSensor(SideFLPulse, SideFLEcho);
    delay(2);
    BackSideSense = ReadSonicSensor(SideBLPulse, SideBLEcho);
    FrontIR = (480 - analogRead(A14)) / 25 + 27;
    snprintf(report, sizeof(report), "Front: {%6d}    Back: {%6d}    Broken", FrontSideSense, BackSideSense);
    Serial1.println(report);
    CurrentTurn = ReadCompass();
    CurrentTurn = CurrentTurn + 360 - TargetDirection;
    while (CurrentTurn > 180) {
      CurrentTurn = CurrentTurn - 360;
    }
    if (abs(CurrentTurn) > 10) {
      Serial1.println("I'm trying to turn");
      TurnStraight(255, TargetDirection);
    }
  }
  analogWrite(MotorA1, 0);
  analogWrite(MotorB1, 0);
  analogWrite(MotorA2, Speed/4);
  analogWrite(MotorB2, Speed/4);
  delay(25);
  analogWrite(MotorA2, Speed/2);
  analogWrite(MotorB2, Speed/2);
  delay(50);
  analogWrite(MotorA2, Speed*3/4);
  analogWrite(MotorB2, Speed*3/4);
  delay(100);
  analogWrite(MotorA2, Speed);
  analogWrite(MotorB2, Speed);
  delay(100);
  analogWrite(MotorA2, 0);
  analogWrite(MotorB2, 0);
  return;
}
