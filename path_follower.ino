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

const int LEFT_MOTOR_THRESHOLD = 29;
const int RIGHT_MOTOR_THRESHOLD = 31;

const int LEFT_MOTOR_BASEVALUE = 37; //Originally 38 each
const int RIGHT_MOTOR_BASEVALUE = 39;

const int LEFT_MOTOR_OFFVALUE = 16;
const int RIGHT_MOTOR_OFFVALUE = 15;

const int LEFT_MOTOR_MAX = 55;
const int RIGHT_MOTOR_MAX = 50;

enum Direction {
  NONE,
  FORWARD,
  LEFT,
  RIGHT,
  STOP
};

enum Zone {
    LEFT_FAR_L,
    LEFT_CLOSE_L,
    LEFT_CLOSE_R,
    LEFT_FAR_R,
    LEFT_OFF,
    FRONT_FAR_L,
    FRONT_CLOSE_L,
    FRONT_ON,
    FRONT_CLOSE_R,
    FRONT_FAR_R,
    FRONT_OFF,
    RIGHT_FAR_L,
    RIGHT_CLOSE_L,
    RIGHT_CLOSE_R,
    RIGHT_FAR_R,
    RIGHT_OFF
};

struct state {
  Zone zone;
  Direction dir;
};

typedef struct state State;

double locationOld = 0;
unsigned long timeOld = 0;
double errorSum = 0;

double kp = 5;
double kd = 0;
double ki = 0;

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

void powerMotor(Direction dir, int amount) {
  //int leftPower = map(analogRead(LEFT_MOTOR), 1000, 0, 0, 100);
  //int rightPower = map(analogRead(RIGHT_MOTOR), 1000, 0, 0, 100);
  int leftPower = analogRead(LEFT_MOTOR);
  int rightPower = analogRead(RIGHT_MOTOR);

  //Serial.print("Left motor value:");
  //Serial.print(leftPower);
  //Serial.print("\n");
  //Serial.print(map(analogRead(LEFT_MOTOR), 1000, 0, 0, 100));
  //Serial.print("\n");
  //Serial.print("Right motor value:");
  //Serial.print(rightPower);
  //Serial.print("\n");
  //Serial.print(map(analogRead(RIGHT_MOTOR), 1000, 0, 0, 100));
  //Serial.print("\n");
  
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
      break;
    case LEFT:
      powerLED(RED, ON);
      rightPower = RIGHT_MOTOR_THRESHOLD + amount;
      leftPower = LEFT_MOTOR_THRESHOLD - amount;
      break;
    case RIGHT:
      powerLED(BLUE, ON);
      leftPower = LEFT_MOTOR_THRESHOLD + amount;
      rightPower = RIGHT_MOTOR_THRESHOLD - amount;
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

double getLocation() {
  const int ZONE_LEFT_FAR = 0;
  const int ZONE_LEFT_CLOSE = 1;
  const int ZONE_ON = 2;
  const int ZONE_RIGHT_CLOSE = 3;
  const int ZONE_RIGHT_FAR = 4;

  const int RIGHT_THRESHOLDS[5] = {986, 660, 528, 825, 986};
  const int RIGHT_SLOPES[5] = {-163, -66, 0, 148.5, 80.5};
  const int FRONT_THRESHOLDS[5] = {985, 922, 641, 767, 975};
  const int FRONT_SLOPES[5] = {-31.5, -140.5, 0, 63, 104};
  const int LEFT_THRESHOLDS[5] = {990, 970, 699, 761, 985};
  const int LEFT_SLOPES[5] = {-10, -135.5, 0, 31, 112};

  int valFront = readSensor(IR_FRONT);
  int valLeft = readSensor(IR_LEFT);
  int valRight = readSensor(IR_RIGHT);
  
  if (curState.direction == LEFT) {
    
  } else if (curState.direction == RIGHT) {
    
  } else { // direction is forward
    if (valLeft <= LEFT_THRESHOLDS[ZONE_RIGHT_FAR]) {
      curState.direction = LEFT;
      curState.zone = RIGHT_FAR_L;
    } else if (valRight <= RIGHT_THRESHOLDS[ZONE_LEFT_FAR]) {
      curState.direction = RIGHT;
      curState.zone = LEFT_FAR_R;
    } else { // All is good and going forward
      return 0;
    }
  }
}

void movementCalculation(Direction* dir, double* output) {
  // Error check with Direction:NONE?
  unsigned long timeCur = millis();
  double timeChange = timeCur - timeOld;

  double locationCur = getLocation();

  double error = locationCur;
  double errorChange = (locationCur - locationOld) / timeChange;
  errorSum += (error * timeChange);

  if (error < -0.5) {
    if (*dir != RIGHT) {
      errorSum = 0;
    }
    *dir = RIGHT;
  } else if (error > 0.5) {
    *dir = LEFT;
    if (*dir != LEFT) {
      errorSum = 0;
    }
  } else {
    errorSum = 0;
    *dir = FORWARD;
  }

  *output = kp * error + kd * errorChange + ki * errorSum;

  locationOld = locationCur;
  timeOld = timeCur;
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

  State stateCur;
  stateCur.zone = FRONT_ON;
  stateCur.dir = FOWARD;

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

  while (true) {
  Serial.print("Left sensor: ");
  Serial.print(readSensor(IR_LEFT));
  Serial.print("\n");
  Serial.print("Front sensor: ");
  Serial.print(readSensor(IR_FRONT));
  Serial.print("\n");
  Serial.print("Right sensor: ");
  Serial.print(readSensor(IR_RIGHT));
  Serial.print("\n");
  delay(1500);
  }

//  int previousState[2] = {2, curDir};
  
  
//  powerLED(RED, ON);
//    analogWrite(LEFT_MOTOR, 0);
//    analogWrite(RIGHT_MOTOR, 0);
//    delay(50);
//    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
//    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
//    delay(1000);
//    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
//    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
//    powerLED(RED, OFF);
//    delay(50);
//
//  while (true) {
//    Direction dir = NONE;
//    double ammount = 0;
//
//    movementCalculation(&dir, &ammount);
//
//    powerMotor(dir, ammount);
//  }
}
