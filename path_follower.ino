#define IRSensor_Front  "A0"
#define IRSensor_Left   "A3"
#define IRSensor_Right  "A7"

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

#define LEFT_MOTOR      11
// 31 to start, 12 at lowest
#define RIGHT_MOTOR     3
// 31 to start, 15 at lowest

const int LEFT_MOTOR_THRESHOLD = 40;
const int RIGHT_MOTOR_THRESHOLD = 40;

const int LEFT_MOTOR_BASEVALUE = 50;
const int RIGHT_MOTOR_BASEVALUE = 50;

const int RIGHT_MOTOR_MAX = 100;
const int LEFT_MOTOR_MAX = 100;

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
  int leftPower = analogRead(LEFT_MOTOR);
  int rightPower = analogRead(RIGHT_MOTOR);
  
  switch (dir) {
    case FORWARD:
      powerLED(GREEN, ON);
      leftPower += 5;
      leftPower += 5;
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
  powerLED(LED_G, HIGH);
  powerLED(LED_B, HIGH);
  powerLED(LED_R, HIGH);

//  Serial.print("Front: ");
//  Serial.print(readSensor(IR_FRONT));
//  Serial.print("\n");
//  Serial.print("Left: ");
//  Serial.print(readSensor(IR_LEFT));
//  Serial.print("\n");
//  Serial.print("Right: ");
//  Serial.print(readSensor(IR_RIGHT));
//  Serial.print("\n");

//  int speed = 31;
//  while (speed > 12) {
//    analogWrite(RIGHT_MOTOR, map(speed, 0, 100, 0, 255));
//    Serial.print("Current speed is: ");
//    Serial.print(speed);
//    Serial.print("\n");
//    speed -= 1;
//    delay(2000);
//  }
//  analogWrite(LEFT_MOTOR, map(0, 0, 100, 0, 255));

  delay(1000);
}
