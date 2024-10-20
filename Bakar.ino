#include <SoftwareSerial.h>

// Enable or disable motor logs
#define MOTOR_DEBUGGING true

// Set to true if controller app is not used
bool start = true;

// Adjust these pins according your robot
const byte IRSensorLeftPin = 12;
const byte IRSensorRightPin = 11;

const byte leftMotorEnablePin = 10;
const byte leftMotorPin1 = 9;
const byte leftMotorPin2 = 8;

const byte rightMotorEnablePin = 5;
const byte rightMotorPin1 = 6;
const byte rightMotorPin2 = 7;

const byte bluetoothRXPin = 3;
const byte bluetoothTXPin = 4;

// Motor Speed
int motorSpeed = 60;
int motorSpeedBias = 127 - 0;

enum Movement
{
  STOP,
  STRAIGHT,
  LEFT,
  RIGHT
};
const char *movementNames[] = {"Stop", "Straight", "Left", "Right"};

// Saved Last Turn
Movement movement = STOP;
Movement lastTurn = STRAIGHT;

enum MessageName
{
  START_ROBOT,
  STOP_ROBOT,
  MOTOR_SPEED,
  MOTOR_SPEED_BIAS
};

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

SoftwareSerial BTSerial(bluetoothRXPin, bluetoothTXPin);

void rotateMotor(Movement _movement)
{
  movement = _movement;

  leftMotorSpeed = abs(motorSpeed);
  rightMotorSpeed = abs(motorSpeed);
  if (motorSpeedBias > 127)
  {
    rightMotorSpeed = min((rightMotorSpeed + motorSpeedBias - 127), 255);
  }
  else if (motorSpeedBias < 127)
  {
    leftMotorSpeed = min((leftMotorSpeed - motorSpeedBias + 127), 255);
  }

  if (movement == LEFT)
  {
    leftMotorSpeed = 120;
    rightMotorSpeed = 100;
  }
  else if (movement == RIGHT)
  {
    leftMotorSpeed = 100;
    rightMotorSpeed = 120;
  }

  if (movement == STOP)
  {
    rightMotorSpeed = 0;
    leftMotorSpeed = 0;

    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);

    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }
  else if (movement == STRAIGHT)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);

    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else if (movement == LEFT)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);

    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else if (movement == RIGHT)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);

    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }

  analogWrite(leftMotorEnablePin, leftMotorSpeed);
  analogWrite(rightMotorEnablePin, rightMotorSpeed);
}

void handleMessage(MessageName messageName)
{
  byte rb;

  if (BTSerial.available() == 0)
    return;
    
  rb = BTSerial.read();
  
  if (messageName == MOTOR_SPEED)
  {
    motorSpeed = rb;
  }
  else if (messageName == MOTOR_SPEED_BIAS)
  {
    motorSpeedBias = rb;
  }
}

void handleBT()
{
  char rc;

  if (BTSerial.available() > 0)
  {
    rc = BTSerial.read();

    if (rc == 'a')
      handleMessage(MOTOR_SPEED);
    else if (rc == 'b')
      handleMessage(MOTOR_SPEED_BIAS);
    else if (rc == 't')
    {
      start = true;
      return;
    }
    else if (rc == 'p')
    {
      start = false;
      rotateMotor(STOP);
      return;
    }
    else if (rc == 'r') {
      BTSerial.write("s"); // S: Speed
      BTSerial.write(motorSpeed);
      BTSerial.write("b"); // S: Speed
      BTSerial.write(motorSpeedBias);
    }
  }
}

void setup()
{
  // This sets PWM frequency for D5 & D6 as a hz.
  TCCR0B = (TCCR0B & B11111000) | B00000011;
  // This sets PWM frequency for D9 & D10 as a hz.
  TCCR1B = (TCCR1B & B11111000) | B00000011;

  Serial.begin(9600);
  BTSerial.begin(9600);

  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(IRSensorLeftPin, INPUT);
  pinMode(IRSensorRightPin, INPUT);
  rotateMotor(STOP);
}

void loop()
{
  handleBT();

  if (!start)
    return;

  int leftIRSensorValue = digitalRead(IRSensorLeftPin);
  int rightIRSensorValue = digitalRead(IRSensorRightPin);

  // If none of the sensors detects black line, then go straight
  if (leftIRSensorValue == LOW && rightIRSensorValue == LOW)
  {
    rotateMotor(STRAIGHT);
  }
  // If left sensor detects black line, then turn left
  else if (leftIRSensorValue == HIGH && rightIRSensorValue == LOW)
  {
    rotateMotor(LEFT);
    lastTurn = LEFT;
  }
  // If right sensor detects black line, then turn right
  else if (leftIRSensorValue == LOW && rightIRSensorValue == HIGH)
  {
    rotateMotor(RIGHT);
    lastTurn = RIGHT;
  }
  // If both the sensors detect black line, then stop
  else if (leftIRSensorValue == HIGH && rightIRSensorValue == HIGH)
  {
    // rotateMotor(0, 0);
    if (lastTurn == LEFT)
    {
      rotateMotor(LEFT);
    }
    else if (lastTurn == RIGHT)
    {
      rotateMotor(RIGHT);
    }
  }

  // DEBUGGING
#if MOTOR_DEBUGGING
    Serial.print("L1: ");
    Serial.print(digitalRead(leftMotorPin1));
    Serial.print(", L2: ");
    Serial.print(digitalRead(leftMotorPin2));
    Serial.print(", R1: ");
    Serial.print(digitalRead(rightMotorPin1));
    Serial.print(", R2: ");
    Serial.print(digitalRead(rightMotorPin2));
    Serial.print("    Sensor: ");
    if (leftIRSensorValue == LOW)
      Serial.print("LOW ");
    else
      Serial.print("HIGH");
    if (rightIRSensorValue == LOW)
      Serial.print("LOW    ");
    else
      Serial.print("HIGH  ");
    Serial.print("Last Turn: ");
    Serial.print(movementNames[lastTurn]);
    Serial.print("    Movement:");
    Serial.print(movementNames[movement]);
    Serial.print("    Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print("  ");
    Serial.println(rightMotorSpeed);
#endif
}