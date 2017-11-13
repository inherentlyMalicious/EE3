#define IRSensor_Front  A0
#define IRSensor_Left   A3
#define IRSensor_Right  A7

enum IRSENSOR {
  IR_FRONT,
  IR_LEFT,
  IR_RIGHT
};

#define LED_R 6
#define LED_G 5
#define LED_B 7

enum LED {
  RED,
  GREEN,
  BLUE,
  ALL
};

const int ON = 1;
const int OFF = 0;

const int IR_LEFT_GOAL = 775;
const int IR_FRONT_GOAL = 575;
const int IR_RIGHT_GOAL = 475;

#define LEFT_MOTOR      11
// 31 to start, 12 at lowest
#define RIGHT_MOTOR     3
// 31 to start, 15 at lowest

const int LEFT_MOTOR_THRESHOLD = 26;
const int RIGHT_MOTOR_THRESHOLD = 25;

const int LEFT_MOTOR_BASEVALUE = 38;
const int RIGHT_MOTOR_BASEVALUE = 38;

const int LEFT_MOTOR_OFFVALUE = 16;
const int RIGHT_MOTOR_OFFVALUE = 15;

const int LEFT_MOTOR_MAX = 55;
const int RIGHT_MOTOR_MAX = 50;

enum Direction {
  FORWARD,
  LEFT,
  RIGHT,
  STOP
};

/*** WHEN READING IN SENSORS, NOT THEM TO GET TRUE VALUE */

void powerLED(LED led, int power) {
  switch (led) {
    case RED:
      if (power) {
        digitalWrite(LED_R, HIGH);        
      } else {
        digitalWrite(LED_R, LOW);
      }
      break;
    case GREEN:
      if (power) {
        digitalWrite(LED_G, HIGH);        
      } else {
        digitalWrite(LED_G, LOW);
      }
      break;
    case BLUE:
      if (power) {
        digitalWrite(LED_B, HIGH);        
      } else {
        digitalWrite(LED_B, LOW);
      }
      break;
    case ALL:
      if (power) {
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, HIGH);
      } else {
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_B, LOW);
      }
      break;
    default:
      break;
  }
}

void powerMotor(Direction dir, int adjustment) {
  //int leftPower = map(analogRead(LEFT_MOTOR), 1000, 0, 0, 100);
  //int rightPower = map(analogRead(RIGHT_MOTOR), 1000, 0, 0, 100);
  int leftPower = analogRead(LEFT_MOTOR);
  int rightPower = analogRead(RIGHT_MOTOR);

  Serial.print("Left motor value:");
  Serial.print(leftPower);
  Serial.print("\n");
  Serial.print(map(analogRead(LEFT_MOTOR), 1000, 0, 0, 100));
  Serial.print("\n");
  Serial.print("Right motor value:");
  Serial.print(rightPower);
  Serial.print("\n");
  Serial.print(map(analogRead(RIGHT_MOTOR), 1000, 0, 0, 100));
  Serial.print("\n");
  
//  if (leftPower <= LEFT_MOTOR_OFFVALUE || rightPower <= RIGHT_MOTOR_OFFVALUE) {
//    powerLED(RED, ON);
//    analogWrite(LEFT_MOTOR, 0);
//    analogWrite(RIGHT_MOTOR, 0);
//    delay(50);
//    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
//    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
//    delay(500);
//    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
//    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
//    powerLED(RED, OFF);
//    delay(50);
//    return;
//  }
  
  switch (dir) {
    case FORWARD:
      powerLED(GREEN, ON);
      leftPower += adjustment;
      rightPower += adjustment;
      break;
    case LEFT:
      powerLED(RED, ON);
      leftPower -= adjustment;
      rightPower += adjustment;
      break;
    case RIGHT:
      powerLED(BLUE, ON);
      leftPower += adjustment;
      rightPower -= adjustment;
      break;
    case STOP:
      powerLED(RED, ON);
      powerLED(BLUE, ON);
      analogWrite(LEFT_MOTOR, OFF);
      analogWrite(RIGHT_MOTOR, OFF);
    default:
      return;
  }

  analogWrite(LEFT_MOTOR, min( max(leftPower, LEFT_MOTOR_THRESHOLD), LEFT_MOTOR_MAX));
  analogWrite(RIGHT_MOTOR, min( max(rightPower, RIGHT_MOTOR_THRESHOLD), RIGHT_MOTOR_MAX));
}

int readSensor(IRSENSOR sensor) {
  switch (sensor) {
    case IR_FRONT:
      return analogRead(IRSensor_Front);
    case IR_LEFT:
      return analogRead(IRSensor_Left);
    case IR_RIGHT:
      return analogRead(IRSensor_Right);
    default:
      return 0;  
  }
}

void setup() {
  pinMode(A0, INPUT);
  pinMode(A3, INPUT);
  pinMode(A7, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  powerLED(ALL, ON);
  delay(1500);
  powerLED(GREEN, OFF);
  delay(1500);
  powerLED(RED, OFF);
  delay(1500);
  powerLED(BLUE, OFF);

  powerLED(RED, ON);
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0);
    delay(50);
    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
    delay(500);
    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
    powerLED(RED, OFF);
    delay(50);

  while (true) {
    powerMotor(FORWARD, 0);
  }
}
