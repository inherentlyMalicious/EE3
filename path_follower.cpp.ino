#define IRSensor_Right
#define IRSensor_Left
#define IRSensor_Front

#define LED_RED
#define LED_GREEN
#define LED_BLUE

enum LED {
  RED,
  GREEN,
  BLUE,
  ALL
};

const int ON = 1;
const int OFF = 0;

#define LEFT_MOTOR
#define RIGHT_MOTOR

const int MOTOR_THRESHOLD

enum Direction {
  FORWARD,
  LEFT,
  RIGHT,
  STOP
};

/*** WHEN READING IN SENSORS, NOT THEM TO GET TRUE VALUE */

void setup() {
  pinMode(IRSensor_Right, INPUT);
  pinMode(IRSensor_Left, INPUT);
  pinMode(IRSensor_Front, INPUT);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
}

void powerLED(LED led, int power) {
  switch (led) {
    case RED:
      if (power) {
        digitalWrite(LED_RED, HIGH);        
      } else {
        digitalWrite(LED_RED, LOW);
      }
      break;
    case GREEN:
      if (power) {
        digitalWrite(LED_GREEN, HIGH);        
      } else {
        digitalWrite(LED_GREEN, LOW);
      }
      break;
    case BLUE:
      if (power) {
        digitalWrite(LED_BLUE, HIGH);        
      } else {
        digitalWrite(LED_BLUE, LOW);
      }
      break;
    case ALL:
      if (power) {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
      } else {
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
      }
      break;
    default:
      break;
  }
}

void powerMotor(Direction dir) {
  leftPower = analogRead(LEFT_MOTOR);
  rightPower = analogRead(RIGHT_MOTOR);
  
  switch (dir) {
    case FORWARD:
      powerLED(GREEN, ON);
      leftPower += 5;
      leftPower += 5;
      powerLED(GREEN, OFF);
      break;
    case LEFT:
      powerLED(RED, ON);
      leftPower -= 5;
      rightPower += 5;
      powerLED(RED, OFF);
      break;
    case RIGHT:
      powerLED(BLUE, ON);
      leftPower += 5;
      rightPower -= 5;
      powerLED(BLUE, OFF);
      break;
    case STOP:
      powerLED(RED, ON);
      powerLED(BLUE, ON);
      analogWrite(LEFT_MOTOR, OFF);
      analogWrite(RIGHT_MOTOR, OFF);
      powerLED(RED, OFF);
      powerLED(BLUE, OFF);
    default:
      return;
  }

  analogWrite(LEFT_MOTOR, max(leftPower, MOTOR_THRESHOLD));
  analogWrite(RIGHT_MOTOR, max(rightPower, MOTOR_THRESHOLD));
}

void loop() {
  if(IRSensor_Front){ //Check this later
      //Send same PWM to the motors
      digitalWrite(GREEN, HIGH);
  }
  
  if(IRSensor_Left ){
    //Ramp up PWM to the right motor and dial down PWM to the left motor
    digitalWrite(  
  }

}
