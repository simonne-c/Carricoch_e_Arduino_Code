
/*
  El Carricoch-e:
*/

#include <IRremote.hpp>


// 1. Define the pins for the IR receiver and both motors.
const int IR_PIN = 10;

const int RIGHT_PWM = 9;
const int RIGHT_IN1 = 12;
const int RIGHT_IN2 = 11;

const int LEFT_PWM = 6;
const int LEFT_IN1 = 7;
const int LEFT_IN2 = 8;


// 2. Define three line-tracking sensor pins.
// The sensors detect whether they are above a black line.
const int LINE_LEFT = A1;
const int LINE_CENTER = A2;
const int LINE_RIGHT = A3;


// 3. Define the ultrasonic sensor pins.
// Trig sends a sound pulse; Echo receives its reflection.
const int TRIG_PIN = 4;
const int ECHO_PIN = 5;


// 4. Define the IR remote button commands.
// The # button activates automatic line following.
const int FORWARD = 24;
const int BACKWARD = 82;
const int LEFT = 8;
const int RIGHT = 90;
const int STOP = 28;
const int LINE_MODE = 13;


// 5. Set driving speeds, obstacle distance, and signal timeout.
const int STRAIGHT_SPEED = 255;
const int TURN_SPEED = 200;
const int LINE_SPEED = 150;
const int STOP_DISTANCE = 20;

const unsigned long TIMEOUT = 350;

unsigned long lastSignal = 0;

bool moving = false;
bool movingForward = false;
bool lineMode = false;


// 6. Control both motors.
// HIGH and LOW determine direction.
// PWM determines how fast the motors rotate.
void moveMotors(bool rightForward, bool leftForward, int speed) {
  digitalWrite(RIGHT_IN1, rightForward ? HIGH : LOW);
  digitalWrite(RIGHT_IN2, rightForward ? LOW : HIGH);

  digitalWrite(LEFT_IN1, leftForward ? HIGH : LOW);
  digitalWrite(LEFT_IN2, leftForward ? LOW : HIGH);

  analogWrite(RIGHT_PWM, speed);
  analogWrite(LEFT_PWM, speed);

  moving = true;
  movingForward = rightForward && leftForward;
}


// 7. Stop both motors.
// Set motor speed to zero and disable direction outputs.
void stopMotors() {
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);

  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);

  moving = false;
  movingForward = false;
}


// 8. Measure how far away an obstacle is.
// Return the distance in centimeters.
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  // Return zero when no valid echo was received.
  if (duration == 0) {
    return 0;
  }

  return duration * 0.0343 / 2;
}


// 9. Follow a black line using three sensors.
// The center sensor drives forward.
// Left and right sensors correct the direction.
void followLine() {
  bool left = digitalRead(LINE_LEFT) == LOW;
  bool center = digitalRead(LINE_CENTER) == LOW;
  bool right = digitalRead(LINE_RIGHT) == LOW;

  if (center) {
    moveMotors(true, true, LINE_SPEED);
  }
  else if (left) {
    moveMotors(true, false, TURN_SPEED);
  }
  else if (right) {
    moveMotors(false, true, TURN_SPEED);
  }
  else {
    stopMotors();
  }
}


// 10. Initialize the Arduino pins and infrared receiver.
// Begin with the motors stopped.
void setup() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(LINE_LEFT, INPUT);
  pinMode(LINE_CENTER, INPUT);
  pinMode(LINE_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
}


// 11. Continuously check the IR remote, sensors, and motors.
void loop() {

  // 12. Read the command received from the IR remote.
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;
    bool repeat = IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;

    // 13. Keep manual movement active when a button is held.
    if (repeat) {
      if (moving && !lineMode) {
        lastSignal = millis();
      }
    }

    // 14. Activate line following with the # button.
    else if (command == LINE_MODE) {
      lineMode = true;
      stopMotors();
    }

    // 15. Move forward manually.
    else if (command == FORWARD) {
      lineMode = false;
      moveMotors(true, true, STRAIGHT_SPEED);
      lastSignal = millis();
    }

    // 16. Move backward manually.
    else if (command == BACKWARD) {
      lineMode = false;
      moveMotors(false, false, STRAIGHT_SPEED);
      lastSignal = millis();
    }

    // 17. Turn left manually.
    else if (command == LEFT) {
      lineMode = false;
      moveMotors(true, false, TURN_SPEED);
      lastSignal = millis();
    }

    // 18. Turn right manually.
    else if (command == RIGHT) {
      lineMode = false;
      moveMotors(false, true, TURN_SPEED);
      lastSignal = millis();
    }

    // 19. Stop the vehicle and exit line-following mode.
    else if (command == STOP) {
      lineMode = false;
      stopMotors();
    }

    // 20. Prepare to receive another remote command.
    IrReceiver.resume();
  }


  // 21. Check for obstacles when moving forward or following a line.
  // Stop the vehicle if an obstacle is within 20 cm.
  if (lineMode || movingForward) {
    float distance = getDistance();

    if (distance <= STOP_DISTANCE) {
      stopMotors();
      return;
    }
  }


  // 22. Follow the black line when automatic mode is active.
  if (lineMode) {
    followLine();
  }


  // 23. Stop manual movement if the remote signal is lost.
  // Line-following mode does not require continuous remote signals.
  if (!lineMode && moving && millis() - lastSignal > TIMEOUT) {
    stopMotors();
  }
}
