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

const int LEFT_MOTOR_THRESHOLD = 34;
const int RIGHT_MOTOR_THRESHOLD = 37;

const int LEFT_MOTOR_BASEVALUE = 39; //Originally 38 each
const int RIGHT_MOTOR_BASEVALUE = 39;

const int LEFT_MOTOR_OFFVALUE = 16;
const int RIGHT_MOTOR_OFFVALUE = 15;

const int LEFT_MOTOR_MAX = 100;
const int RIGHT_MOTOR_MAX = 100;

enum Direction {
  NONE,
  FORWARD,
  LEFT,
  RIGHT,
  STOP
};

enum Zone {
  ZONE_LEFT_FAR,
  ZONE_LEFT_CLOSE,
  ZONE_LEFT_GAP,
  ZONE_FRONT,
  ZONE_RIGHT_GAP,
  ZONE_RIGHT_CLOSE,
  ZONE_RIGHT_FAR
};
 
struct state {
  float distance;
  Zone zone;
  Direction side;
  Direction dir;
};

typedef struct state State;

State curState;

double locationOld = 0;
unsigned long timeOld = 0;
double errorSum = 0;

double kp = 0.6;
double kd = 0;
double ki = 0;

  Direction dir = NONE;
  double ammount = 0;

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
//  int leftPower = analogRead(LEFT_MOTOR);
//  int rightPower = analogRead(RIGHT_MOTOR);
  int leftPower, rightPower;
  
  switch (dir) {
    case FORWARD:
      powerLED(GREEN, ON);
      break;
    case LEFT:
      powerLED(RED, ON);
      rightPower = RIGHT_MOTOR_BASEVALUE + amount;
      leftPower = LEFT_MOTOR_BASEVALUE;
      break;
    case RIGHT:
      powerLED(BLUE, ON);
      leftPower = LEFT_MOTOR_BASEVALUE + amount;
      rightPower = RIGHT_MOTOR_BASEVALUE;
      break;
    case STOP:
      powerLED(RED, ON);
      powerLED(BLUE, ON);
      analogWrite(LEFT_MOTOR, OFF);
      analogWrite(RIGHT_MOTOR, OFF);
    default:
      return;
  }
  int leftPowerLimited = min( max(leftPower, LEFT_MOTOR_THRESHOLD), LEFT_MOTOR_MAX);
  int rightPowerLimited = min( max(rightPower, RIGHT_MOTOR_THRESHOLD), RIGHT_MOTOR_MAX);
  analogWrite(LEFT_MOTOR, map(leftPowerLimited, 0.0, 100.0, 0.0, 255.0));
  analogWrite(RIGHT_MOTOR, map(rightPowerLimited, 0.0, 100.0, 0.0, 255.0));
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

void setDirection() {
  int valLeft = readSensor(IR_LEFT);
  int valRight = readSensor(IR_RIGHT);
  if (valLeft <= 980) {
    curState.zone = ZONE_LEFT_CLOSE;
    curState.dir = LEFT;
    curState.side = RIGHT;
    getLocation();
  } else if (valRight <= 980) {
    curState.zone = ZONE_RIGHT_CLOSE;
    curState.dir = RIGHT;
    curState.side = LEFT;
    getLocation();
  }
}

double getLocation() {
  
  const int valLeftTarget = 700;
  const int valLeftMax = 980;
  const int valFrontTarget = 700;
  const int valFrontMax = 980;
  const int valRightTarget = 600;
  const int valRightMax = 980;

  int valLeft = readSensor(IR_LEFT);
  float absDistanceLeft = pow(abs(((valLeft - 761.959) / 235) + 0.093), 0.5);
  int valFront = readSensor(IR_FRONT);
  float absDistanceFront = pow(abs(((valFront - 720.870) / 271) + 0.065), 0.5);
  int valRight = readSensor(IR_RIGHT);
  float absDistanceRight = pow(abs(((valRight - 596.321) / 401) - 0.041), 0.5);
  switch (curState.zone) {
    case ZONE_LEFT_FAR:
      if (curState.dir == LEFT && valLeft <= valLeftMax) {
        curState.zone = ZONE_LEFT_CLOSE;
        curState.side = LEFT;
      }
      break;
      case ZONE_LEFT_CLOSE:
      switch (curState.dir) {
        case LEFT:
          if (curState.side == LEFT && valLeft <= valLeftTarget) {
            curState.side = RIGHT;
          } else if (curState.side == RIGHT && valLeft >= valLeftMax) {
            curState.zone = ZONE_LEFT_GAP;
          }
        break;
        case RIGHT:
          if (curState.side == RIGHT && valLeft <= valLeftTarget) {
            curState.side = LEFT;
          } else if (curState.side == LEFT && valLeft >= valLeftMax) {
            curState.zone = ZONE_LEFT_FAR;
          }
        break;
        default:
          break;
      }
      break;
    case ZONE_LEFT_GAP:
      switch (curState.dir) {
        case RIGHT:
          if (valLeft <= valLeftMax) {
            curState.zone = ZONE_LEFT_CLOSE;
            curState.side = RIGHT;
          }
        case LEFT:
          if (valFront <= valFrontMax) {
            curState.zone = ZONE_FRONT;
            curState.side = LEFT;
          }
        break;
        default:
        break;
      }
      break;
    case ZONE_FRONT:
      switch (curState.dir) {
        case RIGHT:
          if (curState.side == RIGHT && valFront <= valFrontTarget) {
            curState.side = LEFT;
          } else if (curState.side == LEFT && valFront >= valFrontMax) {
            curState.zone = ZONE_LEFT_GAP;
          }
        break;
        case LEFT:
          if (curState.side == LEFT && valFront <= valFrontTarget) {
            curState.side = RIGHT;
          } else if (curState.side == RIGHT && valFront >= valFrontMax) {
            curState.zone = ZONE_RIGHT_GAP;
          }
        break;
        default:
        break;
      }
      break;
    case ZONE_RIGHT_GAP:
      switch (curState.dir) {
        case LEFT:
          if (valRight <= valRightMax) {
            curState.zone = ZONE_RIGHT_CLOSE;
            curState.side = LEFT;
          }
        case RIGHT:
          if (valFront <= valFrontMax) {
            curState.zone = ZONE_FRONT;
            curState.side = RIGHT;
          }
        break;
        default:
        break;
      }
      break;
    case ZONE_RIGHT_CLOSE:
      switch (curState.dir) {
        case LEFT:
          if (curState.side == LEFT && valRight <= valRightTarget) {
            curState.side = RIGHT;
          } else if (curState.side == RIGHT && valRight >= valRightMax) {
            curState.zone = ZONE_RIGHT_FAR;
          }
        break;
        case RIGHT:
          if (curState.side == RIGHT && valRight <= valRightTarget) {
            curState.side = LEFT;
          } else if (curState.side == LEFT && valRight >= valRightMax) {
            curState.zone = ZONE_RIGHT_GAP;
          }
        break;
        default:
          break;
      }
      break;
    case ZONE_RIGHT_FAR:
      if (curState.dir == RIGHT && valRight <= valRightMax) {
        curState.zone = ZONE_RIGHT_CLOSE;
        curState.side = RIGHT;
      }
      break;
  }
  switch (curState.zone) {
    case ZONE_LEFT_FAR:
      return -8;
      break;
    case ZONE_LEFT_CLOSE:
      if (curState.side == LEFT) {
        return -4 - absDistanceLeft;
      } else if (curState.side == RIGHT) {
        return -4 + absDistanceLeft;
      }
      break;
    case ZONE_LEFT_GAP:
      return -2;
      break;
    case ZONE_FRONT:
      if (curState.side == LEFT) {
        return 0 - absDistanceFront;
      } else if (curState.side == RIGHT) {
        return  + absDistanceFront;
      } 
      break;
    case ZONE_RIGHT_GAP:
      return 2;
      break;
    case ZONE_RIGHT_CLOSE:
      if (curState.side == LEFT) {
        return 4 - absDistanceRight;
      } else if (curState.side == RIGHT) {
        return 4 + absDistanceRight;
      }
      break;
    case ZONE_RIGHT_FAR:
      return 8;
      break;
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

  curState.zone = ZONE_FRONT;
  curState.side = NONE;
  curState.dir = FORWARD;

  Serial.begin(9600);

  powerLED(ALL, ON);
  delay(1500);
  powerLED(GREEN, OFF);
  delay(1500);
  powerLED(RED, OFF);
  delay(1500);
  powerLED(BLUE, OFF);

 // Kickstart
    powerLED(RED, ON);
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0);
    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_BASEVALUE, 0.0, 100.0, 0.0, 255.0));
    delay(50);
    analogWrite(LEFT_MOTOR, map(LEFT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
    analogWrite(RIGHT_MOTOR, map(RIGHT_MOTOR_THRESHOLD, 0.0, 100.0, 0.0, 255.0));
    delay(50);
    powerLED(RED, OFF);
}

void loop() {
  if (curState.side == NONE) {
    setDirection();
  } else {
    movementCalculation(&dir, &ammount);
    powerMotor(dir, ammount);  
  }

  Serial.print("Zone: ");
  Zone currentzone = curState.zone;
  switch (currentzone) {
    case ZONE_LEFT_FAR:
      Serial.println("ZONE LEFT FAR");
      break;
    case ZONE_LEFT_CLOSE:
      Serial.println("ZONE LEFT CLOSE");
      break;
    case ZONE_LEFT_GAP:
      Serial.println("ZONE LEFT GAP");
      break;
    case ZONE_FRONT:
      Serial.println("ZONE FRONT");
      break;
    case ZONE_RIGHT_GAP:
      Serial.println("ZONE RIGHT GAP");
      break;
    case ZONE_RIGHT_CLOSE:
      Serial.println("ZONE RIGHT CLOSE");
      break;
    case ZONE_RIGHT_FAR:
      Serial.println("ZONE RIGHT FAR");
      break;
    }
    Serial.print("Currently on: ");
    Direction currentside = curState.side;
    switch (currentside) {
      case LEFT:
        Serial.println("LEFT");
      break;
      case RIGHT:
        Serial.println("RIGHT");
      break;
      default:
      break;
    }
    Serial.print("Currently moving: ");
    Direction currentdir = curState.dir;
    switch (currentdir) {
      case LEFT:
        Serial.println("LEFT");
      break;
      case RIGHT:
        Serial.println("RIGHT");
      break;
      default:
      break;
    }

  powerLED(ALL, OFF);
}
