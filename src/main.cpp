#include <Arduino.h>
#include <Metro.h>
#include <Servo.h>
#include <math.h>
#include <globals.h>

//---------------------------------Initialize Functions---------------------------------
//--------------------------------------------------------------------------------------
void setForwardSpeed(int);
void pinAssignments(void);
double echoTest(int, int);
double doSomeTrig(int, int);
int PIDLeft(double, double);
int PIDRight(double, double);
void PIDForward(void);
void setReverseSpeed(int);
void PIDReverse(void);
void flywheelSpinUp(void);
void shoot(void);
void reload(void);
double calculateAngle(void);
void moveToAngle(double);
void flywheelStop(void);
void stopGame(void);

//-----------------------------------States Definition----------------------------------
//--------------------------------------------------------------------------------------

typedef enum
{
  BLIND_FORWARD,
  ALIGN,
  PID_FORWARD,
  PUSH_BUTTON,
  LOADING,
  MOVE_BACK,
  MOVE_BACK_ADJUST,
  MOVE_BACK_ADJUST2,
  AIMING,
  FIRING,
  ALIGN2,
  STOP
} States_t;

States_t state;

//-----------------------------------------Setup----------------------------------------
//--------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("Serial initialized...");
  state = BLIND_FORWARD;
  Serial.println("STATE: BLIND_FORWARD");
  loader_servo.attach(SERVO_PIN);
  pinAssignments();
  reload();
  setForwardSpeed(BLIND_SPEED);
  GAME_START_TIME = millis();
  gameDurationTimer.begin(stopGame, GAME_DURATION_MICRO);
}

//-----------------------------------------Loop----------------------------------------
//--------------------------------------------------------------------------------------

void loop()
{
  //double poop = echoTest(ECHO_PIN_LF,TRIG_PIN_LF);
  //Serial.println(poop);
  //Serial.println(state);
  if (millis() >= GAME_START_TIME + GAME_DURATION)
  {
    state = STOP;
  }
  switch (state)
  {
  //Moves forward out of area so misses corner befoe PID motion; ends on timer
  case BLIND_FORWARD:
    delay(BLIND_DURATION);
    analogWrite(DCMOT_PIN_LEFT, BLIND_SPEED);
    digitalWrite(DIR1_PIN1, LOW);
    digitalWrite(DIR2_PIN1, HIGH);

    analogWrite(DCMOT_PIN_RIGHT, BLIND_SPEED + BLIND_TRIM); //Replaced set forward speed to set trim
    digitalWrite(DIR1_PIN2, HIGH);
    digitalWrite(DIR2_PIN2, LOW);

    setForwardSpeed(0);
    delay(SHORT_DELAY);
    Serial.println("STATE: ALIGN");
    transitionTime = millis();
    state = ALIGN;
    break;

  case ALIGN:
  {
    double currentAngle = calculateAngle();
    const double targetAngle = 0.0;
    double angleError = currentAngle - targetAngle;
    angleError = abs(angleError);

    moveToAngle(targetAngle);

    if (millis() - transitionTime >= AIM_DURATION && angleError <= ANGLE_ERROR_RANGE)
    {
      setForwardSpeed(0);
      //ALIGN_KI = ALIGN_KI_INITIAL;
      errorSum = 0;
      delay(SHORT_DELAY);
      Serial.println("STATE: PID_FORWARD");
      state = PID_FORWARD;
    }
    break;
  }

  //Finds wall and moves along it to the armory
  case PID_FORWARD:
  {
    double forwardDist = echoTest(ECHO_PIN_FRONT, TRIG_PIN_FRONT);
    PIDForward();
    if (forwardDist <= BUTTON_DIST_THRESH)
    {
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: PUSH_BUTTON");
      setForwardSpeed(PUSH_BUTTON_SPEED);
      state = PUSH_BUTTON;
    }
    break;
  }

  //moves forward until hits button
  case PUSH_BUTTON:
  {
    double forwardDist = echoTest(ECHO_PIN_FRONT, TRIG_PIN_FRONT);

    if (forwardDist <= BUTTON_PRESSED_THRESH)
    {
      setForwardSpeed(0);
      Serial.println("STATE: LOADING");
      state = LOADING;
    }
    break;
  }

  //Waits in armory while load the ball launcher
  case LOADING:
    delay(LOAD_DURATION);
    errorSum = 0;
    //setReverseSpeed(BLIND_SPEED);
    Serial.println("STATE: MOVE_BACK");
    state = MOVE_BACK;
    break;

  //Moves back for a few seconds to get out of armory, spins up flywheels, no aiming
  case MOVE_BACK:
  {
    double forwardDist = echoTest(ECHO_PIN_FRONT, TRIG_PIN_FRONT);
    double backwardDist = echoTest(ECHO_PIN_BACK, TRIG_PIN_BACK);
    PIDReverse();
    if (forwardDist >= REVERSE_FIRE_DIST && currentTarget < 2)
    {
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: MOVE_BACK_ADJUST");
      transitionTime = millis();
      state = MOVE_BACK_ADJUST;
    }
    else if (backwardDist <= REVERSE_FIRE_DIST2)
    {
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: MOVE_BACK_ADJUST2");
      transitionTime = millis();
      state = MOVE_BACK_ADJUST2;
    }
    break;
  }

  case MOVE_BACK_ADJUST:
  {
    double forwardDist = echoTest(ECHO_PIN_FRONT, TRIG_PIN_FRONT);
    double distError = forwardDist - REVERSE_FIRE_DIST;
    distError = abs(distError);

    int motorSpeed = BACK_ADJUST_KP * distError + BACK_ADJUST_KI * errorSum;

    if (forwardDist > REVERSE_FIRE_DIST)
    {
      //move forward
      analogWrite(DCMOT_PIN_LEFT, motorSpeed);
      digitalWrite(DIR1_PIN1, LOW);
      digitalWrite(DIR2_PIN1, HIGH);

      analogWrite(DCMOT_PIN_RIGHT, motorSpeed);
      digitalWrite(DIR1_PIN2, HIGH);
      digitalWrite(DIR2_PIN2, LOW);
    }
    else if (forwardDist < REVERSE_FIRE_DIST)
    {
      //move backward
      analogWrite(DCMOT_PIN_LEFT, motorSpeed);
      digitalWrite(DIR1_PIN1, HIGH);
      digitalWrite(DIR2_PIN1, LOW);

      analogWrite(DCMOT_PIN_RIGHT, motorSpeed);
      digitalWrite(DIR1_PIN2, LOW);
      digitalWrite(DIR2_PIN2, HIGH);
    }
    else
    {
      //stop
      setForwardSpeed(0);
    }

    errorSum = errorSum + distError;
    //BACK_ADJUST_KI = BACK_ADJUST_KI + BACK_ADJUST_KI_INITIAL;

    if ((distError < REVERSE_DIST_ERROR_RANGE) && (millis() - transitionTime > BACK_ADJUST_DURATION))
    {
      //transition to next state
      //BACK_ADJUST_KI = BACK_ADJUST_KI_INITIAL;
      errorSum = 0;
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: AIMING");
      transitionTime = millis();
      state = AIMING;
    }
    break;
  }

  case MOVE_BACK_ADJUST2:
  {
    double backwardDist = echoTest(ECHO_PIN_BACK, TRIG_PIN_BACK);
    double distError = backwardDist - REVERSE_FIRE_DIST2;
    distError = abs(distError);

    int motorSpeed = BACK_ADJUST_KP * distError + BACK_ADJUST_KI * errorSum;

    if (backwardDist < REVERSE_FIRE_DIST)
    {
      //move forward
      analogWrite(DCMOT_PIN_LEFT, motorSpeed);
      //analogWrite(DCMOT_PIN_LEFT, 35);
      digitalWrite(DIR1_PIN1, LOW);
      digitalWrite(DIR2_PIN1, HIGH);

      analogWrite(DCMOT_PIN_RIGHT, motorSpeed);
      //analogWrite(DCMOT_PIN_RIGHT, 35);
      digitalWrite(DIR1_PIN2, HIGH);
      digitalWrite(DIR2_PIN2, LOW);
    }
    else if (backwardDist > REVERSE_FIRE_DIST)
    {
      //move backward
      analogWrite(DCMOT_PIN_LEFT, motorSpeed);
      //analogWrite(DCMOT_PIN_LEFT, 35);
      digitalWrite(DIR1_PIN1, HIGH);
      digitalWrite(DIR2_PIN1, LOW);

      analogWrite(DCMOT_PIN_RIGHT, motorSpeed);
      //analogWrite(DCMOT_PIN_RIGHT, 35);
      digitalWrite(DIR1_PIN2, LOW);
      digitalWrite(DIR2_PIN2, HIGH);
    }
    else
    {
      //stop
      setForwardSpeed(0);
    }

    //BACK_ADJUST_KI = BACK_ADJUST_KI + BACK_ADJUST_KI_INITIAL;
    errorSum = errorSum + distError;

    if ((distError < REVERSE_DIST_ERROR_RANGE) && (millis() - transitionTime > BACK_ADJUST_DURATION))
    {
      //transition to next state
      //BACK_ADJUST_KI = BACK_ADJUST_KI_INITIAL;
      errorSum = 0.0;
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: AIMING");
      transitionTime = millis();
      state = AIMING;
    }
    break;
  }

  case AIMING:
  {
    double currentAngle = calculateAngle();
    double currentTargetAngle = TARGET_ANGLES[currentTarget];
    double angleError = currentAngle - currentTargetAngle;
    angleError = abs(angleError);

    moveToAngle(currentTargetAngle);

    if (millis() - transitionTime >= AIM_DURATION && angleError <= ANGLE_ERROR_RANGE)
    {
      if (currentTarget < (TARGET_AMT - 1))
      {
        currentTarget = currentTarget + 1;
      }
      else
      {
        currentTarget = 0;
      }
      setForwardSpeed(0);
      //ALIGN_KI = ALIGN_KI_INITIAL;
      delay(SHORT_DELAY);
      flywheelSpinUp();
      delay(FLYWHEEL_SPINUP_TIME);
      Serial.println("STATE: FIRING");
      state = FIRING;
      //TESTING
      //  transitionTime = millis();
      //  Serial.println("STATE: AIMING");
      //  state = AIMING;
      //----
    }
    break;
  }

  //shoots and reloads until out of ammo
  case FIRING:
    for (int i = 1; i <= BARRAGE_AMT; i++)
    {
      shoot();
      reloadCount = reloadCount - 1;
      delay(SHOOTING_DELAY);
      reload();
      delay(SHOOTING_DELAY);
    }

    flywheelStop();

    if (reloadCount <= 0)
    {
      reload();
      delay(SHORT_DELAY);
      reloadCount = DEFAULT_RELOAD_COUNT;
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: ALIGN2");
      state = ALIGN2;
    }
    else
    {
      setForwardSpeed(0);
      delay(SHORT_DELAY);
      Serial.println("STATE: AIMING");
      transitionTime = millis();
      state = AIMING;
    }
    break;

  case ALIGN2:
  {
    double currentAngle = calculateAngle();
    const double targetAngle = 0.0;
    double angleError = currentAngle - targetAngle;
    angleError = abs(angleError);

    moveToAngle(targetAngle);

    if (millis() - transitionTime >= AIM_DURATION && angleError <= ANGLE_ERROR_RANGE)
    {
      setForwardSpeed(0);
      //ALIGN_KI = ALIGN_KI_INITIAL;
      errorSum = 0;
      delay(SHORT_DELAY);
      Serial.println("STATE: PID_FORWARD");
      state = PID_FORWARD;
    }
    break;
  }

  //stops after fires the balls
  case STOP:
    setForwardSpeed(0);
    flywheelStop();
    break;
  }
}

//---------------------------------------Functions--------------------------------------
//--------------------------------------------------------------------------------------

//-------------------Pin Assignments----------------------

void pinAssignments(void)
{
  //Servo Pin
  pinMode(SERVO_PIN, OUTPUT);

  //Locomotion Motor 1
  pinMode(DCMOT_PIN_LEFT, OUTPUT);
  pinMode(DIR1_PIN1, OUTPUT);
  pinMode(DIR2_PIN1, OUTPUT);

  //Locomotion Motor 2
  pinMode(DCMOT_PIN_RIGHT, OUTPUT);
  pinMode(DIR1_PIN2, OUTPUT);
  pinMode(DIR2_PIN2, OUTPUT);

  //Flywheel  Motors
  pinMode(DCMOT_FW_PIN1, OUTPUT);
  pinMode(DCMOT_FW_PIN2, OUTPUT);
  pinMode(DIR_FW1_PIN1, OUTPUT);
  pinMode(DIR_FW1_PIN2, OUTPUT);
  pinMode(DIR_FW2_PIN1, OUTPUT);
  pinMode(DIR_FW2_PIN2, OUTPUT);

  //Wall Detection Echos
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(ECHO_PIN_LF, INPUT);
  pinMode(TRIG_PIN_LF, OUTPUT);
  pinMode(ECHO_PIN_LB, INPUT);
  pinMode(TRIG_PIN_LB, OUTPUT);
  pinMode(ECHO_PIN_RF, INPUT);
  pinMode(TRIG_PIN_RF, OUTPUT);
  pinMode(ECHO_PIN_RB, INPUT);
  pinMode(TRIG_PIN_RB, OUTPUT);
  pinMode(ECHO_PIN_BACK, INPUT);
  pinMode(TRIG_PIN_BACK, OUTPUT);
}

//---------------------Locomotion------------------------

void setForwardSpeed(int speed)
{
  analogWrite(DCMOT_PIN_LEFT, speed);
  digitalWrite(DIR1_PIN1, LOW);
  digitalWrite(DIR2_PIN1, HIGH);

  analogWrite(DCMOT_PIN_RIGHT, speed);
  digitalWrite(DIR1_PIN2, HIGH);
  digitalWrite(DIR2_PIN2, LOW);
}

void setReverseSpeed(int speed)
{
  analogWrite(DCMOT_PIN_LEFT, speed);
  digitalWrite(DIR1_PIN1, HIGH);
  digitalWrite(DIR2_PIN1, LOW);

  analogWrite(DCMOT_PIN_RIGHT, speed);
  digitalWrite(DIR1_PIN2, LOW);
  digitalWrite(DIR2_PIN2, HIGH);
}

//-------------------Locomotion Helpers ----------------------

double calculateAngle(void)
{
  double l1 = echoTest(ECHO_PIN_LF, TRIG_PIN_LF); //LF_distance_to_wall
  //Serial.println(l1);
  if (l1 >= ULTRASONIC_LIMIT)
  {
    l1 = lastL1Val;
  }

  double l2 = echoTest(ECHO_PIN_LB, TRIG_PIN_LB); //LB_distance_to_wall
  //Serial.println(l2);
  if (l2 >= ULTRASONIC_LIMIT)
  {
    l2 = lastL2Val;
  }

  lastL1Val = l1; //this could be problematic, but prob not
  lastL2Val = l2;

  double theta = atan2((l1 - l2), LEFT_SENSOR_DIST);
  theta = theta * 180.0 / PI;
  return theta;
}

//Aims launcher
void moveToAngle(double targetAngle)
{
  double theta = calculateAngle();

  double angleError = theta - targetAngle;
  angleError = abs(angleError);

  int turnSpeed = NOMINAL_ANGLE_TURN_SPEED;

  if ((angleError * ALIGN_KP) <= NOMINAL_ANGLE_TURN_SPEED)
  {
    turnSpeed = angleError * ALIGN_KP + errorSum * ALIGN_KI;
    //ALIGN_KI = ALIGN_KI + ALIGN_KI_INITIAL;
    errorSum = errorSum + angleError;
  }
  else
  {
    turnSpeed = NOMINAL_ANGLE_TURN_SPEED;
  }

  if (theta > targetAngle)
  {
    //Spin counter clockwise
    //Left Motor
    analogWrite(DCMOT_PIN_LEFT, turnSpeed);
    digitalWrite(DIR1_PIN1, HIGH);
    digitalWrite(DIR2_PIN1, LOW);

    //Right Motor
    analogWrite(DCMOT_PIN_RIGHT, turnSpeed);
    digitalWrite(DIR1_PIN2, HIGH);
    digitalWrite(DIR2_PIN2, LOW);
  }
  else if (theta < targetAngle)
  {
    //Spin clockwise
    //Left Motor
    analogWrite(DCMOT_PIN_LEFT, NOMINAL_ANGLE_TURN_SPEED);
    digitalWrite(DIR1_PIN1, LOW);
    digitalWrite(DIR2_PIN1, HIGH);

    //Right Motor
    analogWrite(DCMOT_PIN_RIGHT, NOMINAL_ANGLE_TURN_SPEED);
    digitalWrite(DIR1_PIN2, LOW);
    digitalWrite(DIR2_PIN2, HIGH);
  }
  else
  {
    setForwardSpeed(0);
  }
}

//send signal out, read it bouncing back
double echoTest(const int ECHO_PIN, const int TRIG_PIN)
{
  // Clears the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2); //originally 2
  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); //originally 10
  digitalWrite(TRIG_PIN, LOW);
  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  double duration = pulseIn(ECHO_PIN, HIGH);
  double distance = duration * 0.034 / 2.0;
  delay(ULTRASONIC_READ_DELAY);
  return distance;
}

//maths
double doSomeTrig(int l1, int l2)
{
  double absolute_distance = (l2 * sin(atan(LEFT_SENSOR_DIST / (l2 - l1))) + l1 * sin(atan(LEFT_SENSOR_DIST / (l2 - l1)))) / 2.0;
  return fabs(absolute_distance);
}

//Proportional control of Left Motor
int PIDLeft(double angle, double absolute_distance)
{
  double desired_angle = (absolute_distance - WALL_BUFFER) * PID_RATE; // desired angle, 30 degrees when 50 centimeters away
  double error = desired_angle - angle;
  double output = error * KP;
  int mot_speed = map(output, -ANGLE_MAJIC_FACTOR * KP, ANGLE_MAJIC_FACTOR * KP, PID_DEFAULT_SPEED + PID_SPEED_RANGE, PID_DEFAULT_SPEED - PID_SPEED_RANGE);
  return mot_speed;
}

//Proportional control of Right Motor
int PIDRight(double angle, double absolute_distance)
{
  double desired_angle = (absolute_distance - WALL_BUFFER) * PID_RATE; // desired angle, 30 degrees when 50 centimeters away
  double error = desired_angle - angle;
  double output = error * KP;
  int mot_speed = map(output, -ANGLE_MAJIC_FACTOR * KP, ANGLE_MAJIC_FACTOR * KP, PID_DEFAULT_SPEED - PID_SPEED_RANGE, PID_DEFAULT_SPEED + PID_SPEED_RANGE);
  return mot_speed;
}

//Move forward using proportional controllers
void PIDForward(void)
{
  double l1 = echoTest(ECHO_PIN_LF, TRIG_PIN_LF); //LF_distance_to_wall
  if (l1 >= ULTRASONIC_LIMIT)
  {
    l1 = lastL1Val;
  }

  double l2 = echoTest(ECHO_PIN_LB, TRIG_PIN_LB); //LB_distance_to_wall
  if (l2 >= ULTRASONIC_LIMIT)
  {
    l2 = lastL2Val;
  }

  lastL1Val = l1;
  lastL2Val = l2;
  double angle = 90.0 - (atan(LEFT_SENSOR_DIST / (l2 - l1)) * 180.0 / PI); //degrees
  if (angle > 90.0)
  {
    angle = angle - 180.0;
  }
  double absolute_distance = doSomeTrig(l1, l2);

  //Left Motor
  analogWrite(DCMOT_PIN_LEFT, PIDLeft(angle, absolute_distance));
  digitalWrite(DIR1_PIN1, LOW);
  digitalWrite(DIR2_PIN1, HIGH);

  //Right Motor
  analogWrite(DCMOT_PIN_RIGHT, PIDRight(angle, absolute_distance));
  digitalWrite(DIR1_PIN2, HIGH);
  digitalWrite(DIR2_PIN2, LOW);
}

//move backward using proportional controllers
void PIDReverse(void)
{
  double l1 = echoTest(ECHO_PIN_LB, TRIG_PIN_LB); //LF_distance_to_wall
  if (l1 >= ULTRASONIC_LIMIT)
  {
    l1 = lastL1Val;
  }

  double l2 = echoTest(ECHO_PIN_LF, TRIG_PIN_LF); //LB_distance_to_wall
  if (l2 >= ULTRASONIC_LIMIT)
  {
    l2 = lastL2Val;
  }

  lastL1Val = l1;
  lastL2Val = l2;

  double angle = 90.0 - (atan(LEFT_SENSOR_DIST / (l2 - l1)) * 180.0 / PI); //degrees
  if (angle > 90.0)
  {
    angle = angle - 180.0;
  }
  double absolute_distance = doSomeTrig(l1, l2);

  //Left (in the forward orienation) Motor
  analogWrite(DCMOT_PIN_LEFT, PIDLeft(angle, absolute_distance));
  digitalWrite(DIR1_PIN1, HIGH);
  digitalWrite(DIR2_PIN1, LOW);

  //Right (in the forward orienation) Motor
  analogWrite(DCMOT_PIN_RIGHT, PIDRight(angle, absolute_distance));
  digitalWrite(DIR1_PIN2, LOW);
  digitalWrite(DIR2_PIN2, HIGH);
}

//------------------------Sieging---------------------------

//spin flywheels before firing
void flywheelSpinUp(void)
{
  digitalWrite(DIR_FW1_PIN1, HIGH);
  digitalWrite(DIR_FW1_PIN2, LOW);
  digitalWrite(DIR_FW2_PIN1, HIGH);
  digitalWrite(DIR_FW2_PIN2, LOW);
  analogWrite(DCMOT_FW_PIN1, FLYWHEEL_SPEED);
  analogWrite(DCMOT_FW_PIN2, FLYWHEEL_SPEED);
}

void flywheelStop(void)
{
  digitalWrite(DIR_FW1_PIN1, HIGH);
  digitalWrite(DIR_FW1_PIN2, LOW);
  digitalWrite(DIR_FW2_PIN1, HIGH);
  digitalWrite(DIR_FW2_PIN2, LOW);
  analogWrite(DCMOT_FW_PIN1, 0);
  analogWrite(DCMOT_FW_PIN2, 0);
}

//shoots one ball
void shoot(void)
{
  loader_servo.write(0);
}

//reloads one ball
void reload(void)
{
  loader_servo.write(150);
}

void stopGame(void)
{
  setForwardSpeed(0);
  flywheelStop();
  state = STOP;
}
