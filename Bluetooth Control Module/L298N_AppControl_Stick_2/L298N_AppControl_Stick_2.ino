#include <ArduinoBlue.h>

// Motor A (LEFT) connections
int enA = 2;
int in1 = 4;
int in2 = 5;

// Motor B (RIGHT) connections
int enB = 3;
int in3 = 6;
int in4 = 7;

int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;
float spd;

ArduinoBlue phone(Serial1);

float scaleSpeed(int spd);

void moveForward();
void moveReverse();
void moveStop();

void turnLeft();
void turnRight();

void leftWheelForward();
void leftWheelReverse();
void rightWheelForward();
void rightWheelReverse();
void leftWheel(uint8_t m1, uint8_t m2);
void rightWheel(uint8_t m3, uint8_t m4);

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  Serial.begin(9600);
  Serial1.begin(9600); // HM-11

  delay(100);

  Serial.println("Ready");
}

void loop() {
  throttle = phone.getThrottle();
  steering = phone.getSteering();

  // Only change movement if either throttle or steering changes
  if (prevThrottle != throttle || prevSteering != steering) {
    prevThrottle = throttle;
    prevSteering = steering;

    if (steering == 49) { // STRAIGHT
      spd = scaleSpeed(throttle);
      analogWrite(enA, spd);
      analogWrite(enB, spd);

      if (throttle < 49) { // REVERSE
        moveReverse();
      }
      else if (throttle > 49) { // FORWARD
        moveForward();
      }
      else if (throttle == 49) { // STILL
        moveStop();
      }
    }
    else {
      spd = scaleSpeed(steering);
      analogWrite(enA, spd);
      analogWrite(enB, spd);

      if (steering < 49) { // LEFT
        turnLeft();
      }
      else if (steering > 49) { // RIGHT
        turnRight();
      }
    }
    printInfo();
    delay(500);
  }
}

// Move Both Wheels Forward
void moveForward() {
  Serial.print("Move Forward.");
  leftWheelForward();
  rightWheelForward();
}

// Move Both Wheels In Reverse
void moveReverse() {
  Serial.print("Move Reverse.");
  leftWheelReverse();
  rightWheelReverse();
}

// Stop Both Wheels
void moveStop() {
  Serial.println("Stop.");
  leftWheelStop();
  rightWheelStop();
}

// Turn Vehicle Left
void turnLeft() {
  Serial.print("Turn Left.");
  leftWheelReverse();
  rightWheelForward();
}

// Turn Vehicle Right
void turnRight() {
  Serial.print("Turn Right.");
  leftWheelForward();
  rightWheelReverse();
}

// Set Left Wheel Forward
void leftWheelForward() {
  Serial.print("\tLEFT:  Forward\t");
  leftWheel(HIGH, LOW);
}

// Set Left Wheel In Reverse
void leftWheelReverse() {
  Serial.print("\tLEFT:  Reverse\t");
  leftWheel(LOW, HIGH);
}

// Turn Both Left Wheel Inputs Off
void leftWheelStop() {
  Serial.print("\tLEFT:  Stop\t");
  leftWheel(LOW, LOW);
}

// Set Right Wheel Forward
void rightWheelForward() {
  Serial.print("\t\tRIGHT: Forward\t");
  rightWheel(HIGH, LOW);
}

// Set Right Wheel In Reverse
void rightWheelReverse() {
  Serial.print("\t\tRIGHT: Reverse\t");
  rightWheel(LOW, HIGH);
}

// Turn Both Right Inputs Wheel Off
void rightWheelStop() {
  Serial.print("\tRIGHT: Stop\t");
  rightWheel(LOW, LOW);
}

// Left Wheel Motion Control
void leftWheel(uint8_t m1, uint8_t m2) {
  digitalWrite(in1, m1);
  digitalWrite(in2, m2);
  Serial.print("IN1: ");
  Serial.print(in1);
  Serial.print("\tIN2: ");
  Serial.println(in2);
}

// Right Wheel Motion Control
void rightWheel(uint8_t m3, uint8_t m4) {
  digitalWrite(in3, m3);
  digitalWrite(in4, m4);
  Serial.print("IN3: ");
  Serial.print(in3);
  Serial.print("\tIN4: ");
  Serial.println(in4);
}

// Outputs value from 0 to 255
float scaleSpeed(int spd) {
  return abs((spd - 49) * 5.1); // abs((spd - 49) * 255 / (99 - 49))
}

void printInfo(){
  Serial.println();
  Serial.println("Throttle   Steering   Adj. Spd");
  Serial.print(throttle);
  Serial.print("\t   ");
  Serial.print(steering);
  Serial.print("\t      ");
  Serial.println(spd);
  Serial.println("----------");
}
