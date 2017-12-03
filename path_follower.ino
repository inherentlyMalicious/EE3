// Zones ***********************************/

enum Direction {
  NONE,
  FORWARD,
  LEFT,
  RIGHT,
  STOP
};

// Zones describing where the black line is in relation to the sensors
enum Zone {
  ZONE_LEFT_CLOSE,
  ZONE_LEFT_GAP,
  ZONE_FRONT,
  ZONE_RIGHT_GAP,
  ZONE_RIGHT_CLOSE,
  ZONE_END
};

// END *************************************/

// State ************************************/

struct state {
  float distance;
  Zone zone;
  Direction side;
  Direction dir;
  boolean complete;
};

typedef struct state State;

State curState;

unsigned long lastLeftRead = stopDeltaT + 10;
unsigned long lastRightRead = 0;
unsigned long stopDeltaT = 200; // milliseconds

// END **************************************/

// Sensors *********************************/

enum IRSENSOR {
  IR_FRONT,
  IR_LEFT,
  IR_RIGHT
};

#define IRSensor_Front  A0
#define IRSensor_Left   A3
#define IRSensor_Right  A7

// END *************************************/

// LEDs ************************************/

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

// END *************************************/

// Motor ***********************************/

#define LEFT_MOTOR      11
#define RIGHT_MOTOR     3

const float LEFT_RIGHT_MOTOR_RATIO = 1.2;

const int LEFT_MOTOR_OFFVALUE = 41;
const int LEFT_MOTOR_THRESHOLD = 51;
const int LEFT_MOTOR_BASEVALUE = 55;
const int LEFT_MOTOR_FRONT_BASEVALUE = 81;
const int LEFT_MOTOR_FRONT_KICKSTART = 108;
const int LEFT_MOTOR_MAX = 200;

const int RIGHT_MOTOR_OFFVALUE = 38;
const int RIGHT_MOTOR_THRESHOLD = 48;
const int RIGHT_MOTOR_BASEVALUE = 62;
const int RIGHT_MOTOR_FRONT_BASEVALUE = 87;
const int RIGHT_MOTOR_FRONT_KICKSTART = 111;
const int RIGHT_MOTOR_MAX = 200;

// END **************************************/

// PID **************************************/

Direction dir = NONE;
double amount = 0;

double locationOld = 0;
unsigned long timeOld = 0;
double errorSum = 0;

double kp = 30;
double kd = -1;
double ki = 0;

// END *************************************/

// Debugging *******************************/

const boolean debug = false;
const boolean debugMotor = false;
const boolean debugSensor = false;
const boolean debugPID = false;
const boolean debugZone = false;

// END *************************************/

// Scale x which ranges from in_min to in_max to 0.0 to 1.0
float absScale(float x, float in_min, float in_max)
{
  return abs((x - in_min) / (in_max - in_min));
}

// Helper to turn on/off LEDs
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

// Helper to power motors
void powerMotor(Zone zone, int amount) {
  int leftPower, rightPower;

  switch (zone) {
    case ZONE_LEFT_CLOSE:
    case ZONE_LEFT_GAP:
      powerLED(RED, ON);
      if (debug && debugMotor) {
        Serial.print("Left side\n");
      }
      if (curState.dir == LEFT) {
        if (debug && debugMotor) {
          Serial.print("Left turn\n");
        }
        leftPower = 0;//LEFT_MOTOR_BASEVALUE;
        rightPower = RIGHT_MOTOR_BASEVALUE + int(amount * LEFT_RIGHT_MOTOR_RATIO);
      } else if (curState.dir == RIGHT) {
        if (debug && debugMotor) {
          Serial.print("Right turn\n");
        }
        leftPower = LEFT_MOTOR_BASEVALUE + amount;
        rightPower = 0;
      }
      break;
    case ZONE_FRONT:
      powerLED(GREEN, ON);
      if (debug && debugMotor) {
        Serial.print("No turn (forward)\n");
      }

      kickstart();
      leftPower = LEFT_MOTOR_FRONT_BASEVALUE;
      rightPower = RIGHT_MOTOR_FRONT_BASEVALUE;
      break;
    case ZONE_RIGHT_GAP:
    case ZONE_RIGHT_CLOSE:
      if (debug & debugMotor) {
        Serial.print("Right side\n");
      }
      powerLED(BLUE, ON);
      if (curState.dir == LEFT) {
        if (debug & debugMotor) {
          Serial.print("Left turn\n");
        }
        leftPower = 0;
        rightPower = RIGHT_MOTOR_BASEVALUE + int(amount * LEFT_RIGHT_MOTOR_RATIO);
      } else if (curState.dir == RIGHT) {
        if (debug & debugMotor) {
          Serial.print("Right turn\n");
        }
        leftPower = LEFT_MOTOR_BASEVALUE + amount;
        rightPower = 0;//RIGHT_MOTOR_BASEVALUE;
      }
      break;
    case ZONE_END:
      powerLED(RED, ON);
      powerLED(BLUE, ON);
      analogWrite(LEFT_MOTOR, OFF);
      analogWrite(RIGHT_MOTOR, OFF);
      return;
      break;
    default:
      powerLED(RED, OFF);
      powerLED(BLUE, OFF);
      powerLED(GREEN, OFF);
      break;
  }

  // Limit motor powers to above the minimum speed they work at and below maximum wanted speed
  int leftPowerLimited = min( max(leftPower, LEFT_MOTOR_THRESHOLD), LEFT_MOTOR_MAX);
  int rightPowerLimited = min( max(rightPower, RIGHT_MOTOR_THRESHOLD), RIGHT_MOTOR_MAX);
  if (debug && debugMotor) {

    Serial.print("Ammount to increase by: ");
    Serial.print(amount);
    Serial.print("\n");
    Serial.print("Motor left value is ");
    Serial.print(leftPowerLimited);
    Serial.print(", and Motor right value is ");
    Serial.print(rightPowerLimited);
    Serial.print("\n");

  }

  analogWrite(LEFT_MOTOR, leftPowerLimited);
  analogWrite(RIGHT_MOTOR, rightPowerLimited);
}

// Helper function to read sensor values
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

// Function to set the starting location/direction of robot
void setDirection() {
  powerMotor(ZONE_FRONT, 0);
  int valLeft = readSensor(IR_LEFT);
  int valRight = readSensor(IR_RIGHT);
  if (valLeft <= 980) { // Line is by the right sensor, robot drifting left
    curState.zone = ZONE_LEFT_CLOSE;
    curState.dir = LEFT;
    curState.side = RIGHT;
    getLocation();
  } else if (valRight <= 980) { // Line is by the left sensor, robot drifting right
    curState.zone = ZONE_RIGHT_CLOSE;
    curState.dir = RIGHT;
    curState.side = LEFT;
    getLocation();
  }
}

// Get current location of line based off sensor readings
double getLocation() {
  // Threshold values to be used
  const int valLeftTarget = 650;
  const int valLeftMax = 980;
  const int valFrontTarget = 550;
  const int valFrontMax = 980;
  const int valRightTarget = 400;
  const int valRightMax = 980;
  const float distanceTarget = 0.33;
  const float distanceMax = 0.97;

  // absDistance values based off exponential line of best fit
  // value is then mapped to conform to the 0-1 cm range it can see
  float valLeft = readSensor(IR_LEFT);
  float absDistanceLeft = pow(abs((valLeft - 624.279) / 355), 0.5) - 0.045;
  absDistanceLeft = absScale(absScale(absDistanceLeft, 0.61, 0.97), 0.2, 1.0);
  float valFront = readSensor(IR_FRONT);
  float absDistanceFront = pow(abs((valFront - 488.434) / 652), 0.5) - 0.127;
  absDistanceFront = absScale(absDistanceFront, 0.3, 0.75);
  float valRight = readSensor(IR_RIGHT);
  float absDistanceRight = pow(abs((valRight - 300.789) / 626), 0.5) + 0.018;
  absDistanceRight = absScale(absDistanceRight, 0.41, 1.06);

  if (debug && debugSensor) {
      Serial.print("Left Sensor: ");
      Serial.print(absDistanceLeft);
      Serial.print("\nFront Sensor: ");
      Serial.print(absDistanceFront);
      Serial.print("\nRight Sensor: ");
      Serial.print(absDistanceRight);
      Serial.print("\n");
  }

  switch (curState.zone) {
    case ZONE_LEFT_CLOSE:
      switch (curState.dir) {
        case LEFT: // Turning left
          if (curState.side == LEFT && absDistanceLeft <= distanceTarget) {
            // Line going from left side of the left sensor to the right side
            curState.side = RIGHT;
          } else if (curState.side == RIGHT && absDistanceLeft >= distanceMax) {
            // Line going from right side of the left sensor to between left and front sensors
            curState.zone = ZONE_LEFT_GAP;
          }
          break;
        default:
          break;
      }
      break;
    case ZONE_LEFT_GAP:
      switch (curState.dir) {
        case RIGHT: // Turning right
          if (absDistanceLeft <= distanceMax) {
            // Line going from between the left and front sensors to the left sensor
            curState.zone = ZONE_LEFT_CLOSE;
            curState.side = RIGHT;
            curState.dir = LEFT;
          }
        case LEFT: // Turning left
          if (absDistanceFront <= distanceMax) {
            // Line going from between the left and front sensors to the front sensor
            curState.zone = ZONE_FRONT;
            curState.side = LEFT;
          }
          break;
        default:
          break;
      }
      break;
    case ZONE_FRONT:
      if (absDistanceLeft <= distanceMax) { // Line is by the right sensor, robot drifting left
        curState.zone = ZONE_LEFT_CLOSE;
        curState.dir = LEFT;
        curState.side = RIGHT;
      } else if (absDistanceRight <= distanceMax) { // Line is by the left sensor, robot drifting right
        curState.zone = ZONE_RIGHT_CLOSE;
        curState.dir = RIGHT;
        curState.side = LEFT;
      }
      break;
    case ZONE_RIGHT_GAP:
      switch (curState.dir) {
        case LEFT: // Turning left
          if (absDistanceRight <= distanceMax) {
            // Line going from between the right and front sensors to the right sensor
            curState.zone = ZONE_RIGHT_CLOSE;
            curState.side = LEFT;
            curState.dir = RIGHT;
          }
        case RIGHT: // Turning right
          if (absDistanceFront <= distanceMax) {
            // Line going from between the right and front sensors to the front sensor
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
        case RIGHT: // Turning right
          if (curState.side == RIGHT && absDistanceRight <= distanceTarget) {
            // Line going from the right side of right sensor to the left side
            curState.side = LEFT;
          } else if (curState.side == LEFT && absDistanceRight >= distanceMax) {
            // Line going from the left side of front sensor to between the right and front sensors
            curState.zone = ZONE_RIGHT_GAP;
          }
          break;
        default:
          break;
      }
      break;
  }

  float distance = 0;
  // Get the distance from the location and the side its on
  switch (curState.zone) {
    case ZONE_LEFT_CLOSE:
      if (curState.side == LEFT) {
        distance = 4 + absDistanceLeft;
      } else if (curState.side == RIGHT) {
        distance = 4 - absDistanceLeft;
      }
      break;
    case ZONE_LEFT_GAP:
      distance = 2;
      break;
    case ZONE_FRONT:
      distance = 0 + absDistanceFront;
      break;
    case ZONE_RIGHT_GAP:
      distance = 2;
      break;
    case ZONE_RIGHT_CLOSE:
      if (curState.side == LEFT) {
        distance = 4 - absDistanceRight;
      } else if (curState.side == RIGHT) {
        distance = 4 + absDistanceRight;
      }
      break;
  }

  if (debug && debugSensor) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("\n");
  }

  return distance;
}

// PID
void movementCalculation(Direction* dir, double* output) {
  unsigned long timeCur = millis();
  unsigned long timeChange = timeCur - timeOld;

  // get and update location
  double locationCur = getLocation();

  // goal location is '0' -> line directly in center
  double error = locationCur;
  double errorChange = (locationCur - locationOld) / timeChange * 1000;
  errorSum += int(error * timeChange);
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

  if (debug && debugPID) {
    double kpAmount = kp * error;
    double kdAmount = kd * errorChange;
    double kiAmount = ki * errorSum;
    Serial.print("Delta t: ");
    Serial.print(timeChange);
    Serial.print("\nError: ");
    Serial.print(error);
    Serial.print("\nError change: ");
    Serial.print(errorChange);
    Serial.print("\nError sum: ");
    Serial.print(errorSum);
    Serial.print("\nkp amount: ");
    Serial.print(kpAmount);
    Serial.print("\nkd amount: ");
    Serial.print(kdAmount);
    Serial.print("\nki amount: ");
    Serial.print(kiAmount);
  }

  *output = (kp * error) + (kd * errorChange) + (ki * errorSum);
  // Save old values
  locationOld = locationCur;
  timeOld = timeCur;
}

void kickstart() {
  analogWrite(LEFT_MOTOR, LEFT_MOTOR_FRONT_KICKSTART);
  analogWrite(RIGHT_MOTOR, RIGHT_MOTOR_FRONT_KICKSTART);
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
  curState.complete = false;

  if (debug) {
    Serial.begin(9600);
  }

  powerLED(ALL, ON);
  delay(1500);
  powerLED(GREEN, OFF);
  delay(1500);
  powerLED(RED, OFF);
  delay(1500);
  powerLED(BLUE, OFF);

  // Kickstart
  powerLED(GREEN, ON);
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(LEFT_MOTOR, LEFT_MOTOR_FRONT_KICKSTART);
  analogWrite(RIGHT_MOTOR, RIGHT_MOTOR_FRONT_KICKSTART);
  delay(20);
}

void loop() {
  if (readSensor(IR_LEFT) < 980) {
    lastLeftRead = millis();
  }
  if (readSensor(IR_RIGHT) < 980) {
    lastRightRead = millis();
  }

  if (abs(lastRightRead - lastLeftRead) <= stopDeltaT) {
    curState.complete = true;
    powerLED(ALL, OFF);
    powerMotor(ZONE_END, 0);
  }
  
  if (!curState.complete) {
    if (curState.side == NONE) {
      setDirection();
    } else {
      movementCalculation(&dir, &amount);
      powerMotor(curState.zone, amount);
    }
  
    powerLED(ALL, OFF);
    
    if (debug && debugZone) {
      Serial.print("Zone: ");
      Zone currentzone = curState.zone;
      switch (currentzone) {
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
    }
  }
}
