//------------ PIN ASSIGNMENTS -----------

//Locomotion DC Motor 1 LEFT
#define DCMOT_PIN_LEFT 23 //Analog
#define DIR1_PIN1 22      //Digital
#define DIR2_PIN1 21      //Digital

//Locomotion DC Motor 2 RIGHT
#define DCMOT_PIN_RIGHT 20 //Analog
#define DIR1_PIN2 19       //Digital
#define DIR2_PIN2 18       //Digital

//Flywheel Motors (Only one direction)
#define DCMOT_FW_PIN1 30 //Analog
#define DCMOT_FW_PIN2 29 //Analog
#define DIR_FW1_PIN1 28
#define DIR_FW1_PIN2 27
#define DIR_FW2_PIN1 26
#define DIR_FW2_PIN2 25

//Servos
#define SERVO_PIN 14 //PWM
//#define SERVO_PIN_TOP 3 //PWM

//Sensors (PWM for TRIG_PIN, others digital)
#define TRIG_PIN_FRONT 2 //front
#define ECHO_PIN_FRONT 3 //front

#define TRIG_PIN_LF 4 //left front side
#define ECHO_PIN_LF 5 //left front side

#define TRIG_PIN_LB 6 //left back side
#define ECHO_PIN_LB 7 //left back side

#define TRIG_PIN_RF 10 //right side
#define ECHO_PIN_RF 11 //right side

#define TRIG_PIN_RB 0 //right side
#define ECHO_PIN_RB 0 //right side

#define TRIG_PIN_BACK 8 //back
#define ECHO_PIN_BACK 9 //back

//------------ CONSTANTS ------------
const int BLIND_DURATION = 2500;
const int LOAD_DURATION = 5000;
const int PUSH_BUTTON_DURATION = 500; //not currently in use
const int BLIND_SPEED = 50;
const int BLIND_TRIM = 5;
const int PUSH_BUTTON_SPEED = 35;
const int NOMINAL_ANGLE_TURN_SPEED = 35; // was 30

//WITH KI
const double ALIGN_KP = 6.0;       //8
const double BACK_ADJUST_KP = 1.5; //2

//WITHOUT KI
//const double ALIGN_KP = 12.0;
//const double BACK_ADJUST_KP = 5.0;

const double ALIGN_KI = .04; //.05
//const double ALIGN_KI = 0;
double errorSum = 0;
//double ALIGN_KI = ALIGN_KI_INITIAL;
const double BACK_ADJUST_KI = .02; //.07
//const double BACK_ADJUST_KI = 0.0;
//double BACK_ADJUST_KI = BACK_ADJUST_KI_INITIAL;

const double BUTTON_DIST_THRESH = 14.0; //was 10.0 - triggers before button is hit - momentum carries robot into button
const double BUTTON_PRESSED_THRESH = 7.0;
const double LEFT_SENSOR_DIST = 18.8;
const double PID_DEFAULT_SPEED = 52.5; //was 50
const double WALL_BUFFER = 7.5;
const double PID_RATE = 0.8;
const double KP = .08;
const double ANGLE_MAJIC_FACTOR = 30.0;
const double PID_SPEED_RANGE = 15.0; //was 10
const double ULTRASONIC_LIMIT = 100.0;
const double REVERSE_FIRE_DIST = 60.0;
const double REVERSE_FIRE_DIST2 = 60.0;
const int FLYWHEEL_SPEED = 255;
const int ULTRASONIC_READ_DELAY = 3; //necessary delay between reading sensors at short distances to prevent interference
const double ANGLE_ERROR_RANGE = 1.0;
const double REVERSE_DIST_ERROR_RANGE = 0.5;
const int AIM_DURATION = 750;         //was 500
const int BACK_ADJUST_DURATION = 750; //was 500
const int TARGET_AMT = 4;
const double TARGET_ANGLES[TARGET_AMT] = {-17.0, -7.25, -8.0, 9.0}; //3/9/19 {-17.5,-7.25,18.0,25.0};
const int BARRAGE_AMT = 3;                                          //HAS TO BE 3 RIGHT NOW - amount of balls fired in a single barrage
const int FLYWHEEL_SPINUP_TIME = 1250;
const int FLYWHEEL_SPINDOWN_TIME = 500;
const int SHOOTING_DELAY = 500; //was 1000
const int DEFAULT_RELOAD_COUNT = 6;
const int SHORT_DELAY = 250;
const unsigned int GAME_DURATION = (2 * 60) * 1000; //2 min ten seconds
const unsigned int GAME_DURATION_MICRO = (2 * 60 + 1) * 1000000;
unsigned long GAME_START_TIME;

//------------ GLOBAL VARS ------------
double lastFrontDistVal = 0;
double lastBackDistVal = 0;
double lastL1Val = 0;
double lastL2Val = 0;
int reloadCount = DEFAULT_RELOAD_COUNT;
int currentTarget = 0; //vals are 0-3
unsigned long transitionTime;

Servo loader_servo;
IntervalTimer gameDurationTimer;
