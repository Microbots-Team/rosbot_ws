# include <AccelStepper.h>

// Define pins for the stepper motors
AccelStepper LeftMotor(1, 2, 5);  // STEP pin 2, DIR pin 5
AccelStepper RightMotor(1, 3, 6); // STEP pin 3, DIR pin 6

// Enable and limit switch pins
const int EnablePin = 8;
const int LimitSwitchPin = 9;

// Robot parameters
int wheelSpeed = 1000;  // Maximum speed for motors
int stepsPerRevolution = 400;
float wheelCircumference = 0.2;
float distancePerStep = wheelCircumference / stepsPerRevolution;

// Backward handling
int backwardDuration = 500;
bool isMovingBackward = false;
unsigned long backwardStartTime = 0;

// Serial command buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  pinMode(EnablePin, OUTPUT);
  pinMode(LimitSwitchPin, INPUT_PULLUP);
  disableMotors();
  LeftMotor.setMaxSpeed(wheelSpeed);
  RightMotor.setMaxSpeed(wheelSpeed);
  inputString.reserve(50); // Reserve memory for incoming commands
}

void loop() {
  // Check limit switch
  if (digitalRead(LimitSwitchPin) == LOW) {
    handleLimitSwitch();
  }

  // Handle backward duration
  if (isMovingBackward && (millis() - backwardStartTime >= backwardDuration)) {
    stopMoving();
    disableMotors();
    isMovingBackward = false;
  }

  // Check for complete serial message
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Run motors
  LeftMotor.runSpeed();
  RightMotor.runSpeed();

  // Send step count
  sendStepCount();
}

// Handle limit switch trigger
void handleLimitSwitch() {
  stopMoving();
  Serial.println("LIMIT SWITCH HIT");
  moveBackward();
  backwardStartTime = millis();
  isMovingBackward = true;
}

// Enable/disable motors
void enableMotors() {
  digitalWrite(EnablePin, LOW);
}

void disableMotors() {
  digitalWrite(EnablePin, HIGH);
}

// Motion functions
void moveForward(float speed) {
  LeftMotor.setSpeed(speed);
  RightMotor.setSpeed(speed);
}

void moveBackward() {
  LeftMotor.setSpeed(-wheelSpeed);
  RightMotor.setSpeed(-wheelSpeed);
}

void rotateLeft(float speed) {
  LeftMotor.setSpeed(-speed);
  RightMotor.setSpeed(speed);
}

void rotateRight(float speed) {
  LeftMotor.setSpeed(speed);
  RightMotor.setSpeed(-speed);
}

void stopMoving() {
  LeftMotor.setSpeed(0);
  RightMotor.setSpeed(0);
  disableMotors() ;
}


void parseCommand(String cmd) {
  enableMotors();
  float linear = 0;
  float angular = 0;

  cmd.trim();  // Clean up any spaces

  // Debug: print the whole command


  // Search for linear.x and angular.z parts
  int linIndex = cmd.indexOf("linear.x:");
  int angIndex = cmd.indexOf("angular.z:");



  // Extract the linear and angular values by finding substrings between key labels
  if (linIndex != -1 && angIndex != -1) {
    // Extract linear part after "linear.x:"
    String linearStr = cmd.substring(linIndex + 9, cmd.indexOf(",", linIndex)); // Find end of linear value
    // Extract angular part after "angular.z:"
    String angularStr = cmd.substring(angIndex + 9); // Skip the "angular.z:" part

    // Remove colon (if exists) at the start of angular value
    if (angularStr.startsWith(":")) {
      angularStr = angularStr.substring(1);  // Remove the colon
    }



    // Trim and convert to float
    linearStr.trim();
    angularStr.trim();

    linear = linearStr.toFloat();
    angular = angularStr.toFloat();



    // Logic to stop or move motors
    if (abs(linear) < 0.001 && abs(angular) < 0.001) {
      stopMoving();
      disableMotors();
    } else {
      float leftSpeed = (linear * wheelSpeed) - (angular * wheelSpeed);
      float rightSpeed = (linear * wheelSpeed) + (angular * wheelSpeed);



      LeftMotor.setSpeed(leftSpeed);
      RightMotor.setSpeed(rightSpeed);
    }
  }
}



// Send step counts every 100ms
void sendStepCount() {
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= 100) {
    lastSendTime = currentTime;
    long leftSteps = LeftMotor.currentPosition();
    long rightSteps = RightMotor.currentPosition();
    Serial.print("Left Steps: ");
    Serial.print(leftSteps);
    Serial.print(" | Right Steps: ");
    Serial.println(rightSteps);
  }
}

// Serial event for receiving commands
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}