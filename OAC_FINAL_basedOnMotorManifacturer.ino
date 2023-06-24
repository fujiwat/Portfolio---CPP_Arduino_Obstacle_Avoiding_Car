#include <Servo.h>
// Hardware settings
const int   PinServo1 =  10;
const int   PinServo2 =  9;
const int   PinMeterFL = A0;
const int   PinMeterFR = A1;
const int   PinMeterRL = A0;
const int   PinMeterRR = A1;
const int   PinIrL = A0;
const int   PinIrR = A1;
const int   DriveVolt = 9;  /* motor drive max voltage */
const int   PowerVolt = 9;  /* external power voltage */
const int   MaxSensorValue = 1023;
const float MaxMotorSpeedInitial = 1.0;
const float MaxMotorSpeedStrait = 0.35;  /* real max speed = 1.0 you can saturate with this value 0.3 to 1.0 */
const float MaxMotorSpeedCurve  = 0.75;

const int   MOTORLATCH = 12;
const int   MOTORCLK = 4;
const int   MOTORENABLE = 7;
const int   MOTORDATA = 8;
// environment settings
const int   ObstacleDistanceCm = 15;
const int   servoInitAngle = 34;
unsigned int StopTimerSec = 40;             // StopTimer in seconds.  0=non stop.  DEBUG_DISABLE_MOTOR=override it to 0
// debug mode
#define DEBUG 1                             // debug=1 then Serial.print is enabled for all.
#define DEBUG_MSG_MOTOR 0                   // debug=1, no debug=0
#define DEBUG_MSG_SONAR 0                   // debug=1, no debug=0
#define DEBUG_MSG_IR    0                   // debug=1, no debug=0
#define DEBUG_MSG_DRIVE_DIRECTION       1   // debug=1, no debug=0
#define DEBUG_DISABLE_LINE_DETECT       0   // no line detect mode = 1, (regular) detect mode = 0
#define DEBUG_DISABLE_OBSTACLE_DETECT   0   // no obstacle detect mode = 1, (regular) detect mode = 0
#define DEBUG_DISABLE_MOTOR             0   // no motor spin mode = 1, spin (regular) mode = 0
#define DEBUG_USB                       0   // insert delay in loop() for too much messages

/* Support multiple arguments for Serial.print(), Serial.println() -- max 10 args, needs () for each arg.  (no arg not allowed -- _PRINTLN(("")) is okay.) */
/* usage: _PRINTLN( ("16 in hex is="), (16, HEX), (" in Decimal="), (16), (" in Binary="), (16, BIN), (" 0.4567 with 3 decimal places="), (0.4567, 3) ); */
#if DEBUG == 1
  #define _PRINT(...)     { _FNAME(__VA_ARGS__, _Fa, _F9, _F8, _F7, _F6, _F5, _F4, _F3, _F2, _F1)(__VA_ARGS__); }
  #define _PRINTLN(...)   { _PRINT(__VA_ARGS__); Serial.println(); }
  #define _FNAME(_1,_2,_3,_4,_5,_6, _7, _8, _9, _a, _fx,...) _fx
  #define _SP             Serial.print
  #define _F1(a)            _SP a
  #define _F2(a,b)          _SP a; _SP b
  #define _F3(a,b,c)          _SP a; _SP b; _SP c
  #define _F4(a,b,c,d)        _SP a; _SP b; _SP c; _SP d
  #define _F5(a,b,c,d,e)        _SP a; _SP b; _SP c; _SP d; _SP e
  #define _F6(a,b,c,d,e,f)      _SP a; _SP b; _SP c; _SP d; _SP e; _SP f
  #define _F7(a,b,c,d,e,f,g)      _SP a; _SP b; _SP c; _SP d; _SP e; _SP f; _SP g
  #define _F8(a,b,c,d,e,f,g,h)    _SP a; _SP b; _SP c; _SP d; _SP e; _SP f; _SP g; _SP h
  #define _F9(a,b,c,d,e,f,g,h,i)    _SP a; _SP b; _SP c; _SP d; _SP e; _SP f; _SP g; _SP h; _SP i
  #define _Fa(a,b,c,d,e,f,g,h,i,j)  _SP a; _SP b; _SP c; _SP d; _SP e; _SP f; _SP g; _SP h; _SP i; _SP j
#else
  #define _PRINT(...)
  #define _PRINTLN(...)
#endif
#if DEBUG_MSG_MOTOR
  #define _PRINT_MOTOR _PRINT
  #define _PRINTLN_MOTOR _PRINTLN
#else
  #define _PRINT_MOTOR(...)
  #define _PRINTLN_MOTOR(...)
#endif
#if DEBUG_MSG_SONAR
  #define _PRINT_SONAR _PRINT
  #define _PRINTLN_SONAR _PRINTLN
#else
  #define _PRINT_SONAR(...)
  #define _PRINTLN_SONAR(...)
#endif
#if DEBUG_MSG_IR
  #define _PRINT_IR _PRINT
  #define _PRINTLN_IR _PRINTLN
#else
  #define _PRINT_IR(...)
  #define _PRINTLN_IR(...)
#endif
#if DEBUG_MSG_DRIVE_DIRECTION
  #define _PRINT_DRIVE _PRINT
  #define _PRINTLN_DRIVE _PRINTLN
#else
  #define _PRINT_DRIVE(...)
  #define _PRINTLN_DRIVE(...)
#endif


/****************************************************\
  SAMPLE PROGRRAM PROVIDED BY MANUFACTURER
\****************************************************/
enum _MOTOR { RR=1, FR=2, FL=3, RL=4 };
enum _DIREC { FORWARD=1, BACKWARD=2, BRAKE=3, RELEASE=4 };
enum _PIN { MOTOR1_A=2, MOTOR1_B=3, MOTOR2_A=1, MOTOR2_B=4, MOTOR3_A=5, MOTOR3_B=7, MOTOR4_A=0, MOTOR4_B=6,
            MOTOR1_PWM=11, MOTOR2_PWM=3, MOTOR3_PWM=6, MOTOR4_PWM=5, SERVO1_PWM=10, SERVO2_PWM=9 };
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;
  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
        motorA = MOTOR1_A;
        motorB = MOTOR1_B;
        break;
      case 2:
        motorA = MOTOR2_A;
        motorB = MOTOR2_B;
        break;
      case 3:
        motorA = MOTOR3_A;
        motorB = MOTOR3_B;
        break;
      case 4:
        motorA = MOTOR4_A;
        motorB = MOTOR4_B;
        break;
      default:
        break;
    }
    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1); // -1: no PWM set
        break;
      case BRAKE:
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      case RELEASE:
        motor_output (motorA, LOW, 0); // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      default:
        break;
    }
  }
}
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;
  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
      break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
      break;
    default:
      speed = -3333;
      break;
  }
  if (speed != -3333)
  {
    shiftWrite(output, high_low);
    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;
  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);
    // start with all outputs (of the shift register) low
    latch_copy = 0;
    shift_register_initialized = true;
  }
  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}


/****************************************************\
  class: Motor
\****************************************************/
enum Direction { Left = -1, Stop = 0, Right = 1, Forward =2 };

/****************************************************\
  class: IR Sensor
\****************************************************/
class IrSensor {
  protected:
    char *name;
    int pin, maxValue;
  public:
    /****************************************************\
      Initialize
    \****************************************************/
    IrSensor(char *name, int pin, int maxValue) {
      this->name = name;
      this->pin = pin;
      pinMode(pin, INPUT);    /* IR Sensor Left */
    }
    /****************************************************\
      get IR value in analog 0. to 1.
    \****************************************************/
    float getAnalogValue() {
      int meter = analogRead(pin);  /* 0 to maxSensorValue */
      float value = ((float)meter) / ((float)maxValue);
      _PRINTLN_IR( (" Sensor ") , (name), (": "), (meter) , (", value="), (value) );
      return value;  /* value = 0. to +1. */
    }
    /****************************************************\
      get IR value in digital LOW/HIGH
    \****************************************************/
    int digitalRead() {
      int value = ::digitalRead(pin);       /* Low(0): obstacle, High(1): no obstacle */
      _PRINTLN_IR( (" Sensor ") , (name), (": "), (" value="), (value) );
      #if DEBUG_DISABLE_LINE_DETECT
        return LOW;
      #else
        return value;
      #endif
    }
};
IrSensor sensorL("SL", PinIrL, MaxSensorValue);
IrSensor sensorR("SR", PinIrR, MaxSensorValue);

/****************************************************\
  class: Ultra Sonic Sensor
  https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
  Ranging Distance : 2cm – 400 cm/1″ – 13ft
  resolution: 0.3cm
  measuring Angle: 30degree
  Effectual angle: <15degree
  distance to an object = ((speed of sound in the air)*time)/2
  speed of sound in the air at 20ºC (68ºF) = 343m/s
\****************************************************/
Servo servo1;
Servo servo2; // compatible to motor driver sample program
class Sonar {
  protected:
    char *name;
    int pinTrig, pinEcho, lastMm;
    int lastAngle;
  public:
    enum Dir { Left = -1, Stop = 0, Right = 1 };
    /****************************************************\
      Initialize
    \****************************************************/
    Sonar(char *name, int pinTrig, int pinEcho) {
      this->pinTrig = pinTrig;
      this->pinEcho = pinEcho;
      pinMode(pinTrig, OUTPUT);
      pinMode(pinEcho, INPUT);
      lastAngle = servoInitAngle;
    }
    /****************************************************\
      Get Distance in mm
    \****************************************************/
    int getDistanceMm() {
      unsigned long duration, mm;
      /*In the loop(), trigger the sensor by sending a HIGH pulse of 10 microseconds. */
      /* But, before that, give a short LOW pulse to ensure you’ll get a clean HIGH pulse: */
      digitalWrite(pinTrig, LOW);   /* clear the trigger */
      delayMicroseconds(5);       /* wait for stable */
      digitalWrite(pinTrig, HIGH);    /* send the trigger */
      delayMicroseconds(10);      /* weit for stable  */
      digitalWrite(pinTrig, LOW);   /* turn off the trigger */
      duration = pulseIn(pinEcho, HIGH);
      /* distance = (traveltime/2) x speed of sound */
      /* The speed of sound is: 343m/s = 0.0343 cm/uS = 1/29.1 cm/uS */
      /* distance(cm) = (duration/2) / 29.1; */
      /* mm = (float)duration/2. / 2.91; */
      mm = (float)duration / 2. / 2.85;   /* calculated by the TINKERCAD real values */
      if ( mm == 0 ) {            /* can not get the distance = too far */
        mm = 9999;
      }
      _PRINTLN_SONAR( ("DISTANCE="), (mm/10), ("cm"), (" duration="), (duration) );
      return lastMm = mm;
    }
    /****************************************************\
      Getdistance in mm (direction Left or Right)
    \****************************************************/
    int getDistanceMmLR(Sonar::Dir dir) {
      //lock left and get distance
      int distanceMm;
      SetAngle( (dir == (Sonar::Dir::Left)) ? (servoInitAngle + 70) : (servoInitAngle - 70) );
      distanceMm = getDistanceMm();
      delay(100);  /* needed??? */
      SetAngle(servoInitAngle);
      if (dir == (Sonar::Dir::Left)) {
        _PRINTLN_SONAR( ("L="), (distanceMm) );
      } else {
        _PRINTLN_SONAR( ("R="), (distanceMm) );
      }
      return distanceMm;
    }
    /****************************************************\
      Set Angle
    \****************************************************/
    void SetAngle(int angle) {
      _PRINTLN_SONAR( ("  SetAngle( "), (angle), (" )") );
      servo1.write(angle);
      delay( (abs(lastAngle - angle) % 360) * 15 ); /* wait until finish the rotation */
      lastAngle = angle;
    }
};
Sonar sonar("Sonar", A2, A3); /* name, pinTrig, pinEcho, angle */

/***********************************************************************************************************\
 ***********************************************************************************************************  
 *  void setup()
 ***********************************************************************************************************
\***********************************************************************************************************/
void setup() {        /* Initialize */
  Serial.begin(9600);       /* default value = 9600 */
  _PRINTLN( ("***** Arduino setup() start *****") );
  #if DEBUG_DISABLE_LINE_DETECT
    _PRINTLN( ("***** DISABLE LINE DETECT *****") );
  #else
    _PRINTLN( ("***** ENABLE  LINE DETECT *****") );
  #endif
  #if DEBUG_DISABLE_OBSTACLE_DETECT
    _PRINTLN( ("***** DISABLE OBSTACLE DETECT *****") );
  #else
    _PRINTLN( ("***** ENABLE  OBSTANCE DETECT *****") );
  #endif
  #if DEBUG_DISABLE_MOTOR
    _PRINTLN( ("***** DISABLE MOTOR *****") );
    StopTimerSec = 0;       /* always no stop mode, in caseof NO MOTOR */
  #else
    _PRINTLN( ("***** ENABLE  MOTOR *****") );
  #endif
  /* initialize Servo motor */
  servo2.attach(PinServo2);
  servo1.attach(PinServo1);
  servo1.write(servoInitAngle-10);  /* set initial angle to the front */
  delay(15 * 180);
  servo1.write(servoInitAngle);  /* set initial angle to the front */
  /* set motor direction stroingly */
  motor(RR, FORWARD, +MaxMotorSpeedInitial*255 );
  motor(FR, FORWARD, +MaxMotorSpeedInitial*255 );
  motor(FL, FORWARD, +MaxMotorSpeedInitial*255 );
  motor(RL, FORWARD, +MaxMotorSpeedInitial*255 );
  delay( 10 );
  motor(RR, RELEASE, 0);
  motor(FR, RELEASE, 0);
  motor(FL, RELEASE, 0);
  motor(RL, RELEASE, 0);
  CarStop(0); 
}

/****************************************************\
  void loop()
\****************************************************/
void loop() {
  static unsigned long startMillis = millis();
  static Direction lastDirection = Direction::Stop;
  _PRINTLN_DRIVE( ("loop() ") );
  // *******************************
  // ** Black Line detection
  // *******************************
  if ( StopTimerSec==0 || (millis() < (startMillis + StopTimerSec*1000)) ) {
    int sL = sensorL.digitalRead();         /* Line Sensor: Low(0)=WHITE, High(1)=BLACK */
    int sR = sensorR.digitalRead();         /* Line Sensor: Low(0)=WHITE, High(1)=BLACK */
    if ( sL == LOW && sR == LOW ) {
      /* Left=WHITE, Right=WHITE:  both are good, move forwoard */
      _PRINTLN_DRIVE( ("[MOVE FORWARD]") );
      obstacleAvoidAndGo();
      moveForward( 0 );
      lastDirection = Direction::Forward;      
    } else if ( sL == LOW  && sR == HIGH ) {
      /* Left=WHITE, Right=BLACK:  more right is good, turn right */
      _PRINTLN_DRIVE( ("[TR] Black on the right.  More right!") );
      if ( lastDirection != Direction::Right ) {
        CarStop(100);
      }
      moveRight(20);
      lastDirection = Direction::Right;
      _PRINTLN_DRIVE( ("[TR]") );
      obstacleAvoidAndGo();
    } else if ( sL == HIGH  && sR == LOW ) {
      /* Left=BLACK, Right=WHITE:  more left is good, turn left */
      _PRINTLN( ("[TL] Black on the left.  More left!") );
      if ( lastDirection != Direction::Left ) {
        CarStop(100);
      }
      moveLeft(20);
      lastDirection = Direction::Left;
      _PRINTLN( ("[TL]") );
      obstacleAvoidAndGo();
    } else if ( sL == HIGH && sR == HIGH ) {
      /* Left=Black, Right=Black:  stop */
      CarStop(100);
      lastDirection = Direction::Stop;
    }
  } else {
    // Stop Timer comes. then stop.
    _PRINTLN( ("***** StopTimer!!  No more move.  StopTimerSec="), (StopTimerSec) );
    CarStop(1000*60);// delay 1 minute
    lastDirection = Direction::Stop;
  }
  #if DEBUG_USB
    _PRINTLN( (" wait 500msec") );
    delay( 500 );      
  #endif
}

/****************************************************\
  obstacleAvoidAndGo
  return true: 
\****************************************************/
void obstacleAvoidAndGo() {
  int frontCm, leftCm, rightCm;
  
  // *******************************
  // ** Obstacle detection
  // *******************************
  _PRINT_DRIVE( ("obstacleAvoidAndGo()  ") );  
  #if DEBUG_DISABLE_OBSTACLE_DETECT
    frontCm = 1000;                               /* always no obstacle */
  #else
    frontCm = sonar.getDistanceMm() / 10;         /* mm -> cm remove the resolution=0.3ch */
  #endif  
  if ( frontCm <= ObstacleDistanceCm ) {
    /* too close to the obstacle. stop and check */
    _PRINTLN( ("[OBSTACLE FOUND!]") );
    CarStop(100);            /* stop */
    leftCm  = sonar.getDistanceMmLR(Sonar::Dir::Left) / 10; /* mm -> cm remove the resolution=0.3cm */
    rightCm = sonar.getDistanceMmLR(Sonar::Dir::Right) / 10; /* mm -> cm remove the resolution=0.3cm */
//    delay(100); /* needed??? */
    /* check the direction */
    if ( leftCm < rightCm ) {
      /* closer object is in the left -- turn right */
      CarTurnRight();
    } else if ( leftCm > rightCm ) {
      /* closer object is in the right -- turn left */
      CarTurnLeft();
    } else {
      /* closer object in in the front -- anyway turn right */
      CarTurnRight();
    }
  } else {
    /* no object -- move forward */
//    moveForward( 0 );
  }
}

/****************************************************\
  void CarTurnLeft
\****************************************************/
void CarTurnLeft() {
  _PRINT_DRIVE( ("-- CarTurnLeft") );
  moveLeft( 700 );    /* try to turn left */
  CarStop(0);
  moveForward( 900 );   /* move forward, means move left */
  CarStop(0);
  moveRight( 800 );   /* try to turn right, means turn to forward direction */
  CarStop(0);
  moveForward( 700 );   /* move forward */
  CarStop(0);
  moveRight( 700 );   /* try to turn right, means direction back to the line */
  CarStop(0);
  moveForward( 1000 );   /* move forward */
  moveLeft( 400 );    /* try to turn left */
  if (sensorL.digitalRead() == HIGH) {  /* Left is Black */
    loop();
  } else {
    moveForward( 0 );
  }
}

/****************************************************\
  void CarTurnRight
\****************************************************/
void CarTurnRight() {
  _PRINT_DRIVE( ("-- CarTurnRight") );
  moveRight( 700 );   /* try to turn right */
  CarStop(0);
  moveForward( 900 );   /* move forward, means go right direction*/
  CarStop(0);
  moveLeft( 800 );    /* try to turn left, means turn forward direction */
  CarStop(0);
  moveForward( 700 );   /* move forward */
  CarStop(0);
  moveLeft( 700 );    /* try to turn left, means direction back to the line */
  CarStop(0);
  moveForward( 1000 );   /* move forward, means back to the line*/
  CarStop(0);
  moveRight( 400 );   /* try to turn right, means turn direction forward */
  if (sensorR.digitalRead() == HIGH) {
    loop();
  } else {
    moveForward( 0 );
  }
}

/****************************************************\
  void CarStop
\****************************************************/
void CarStop(int mSecDelayAfter) {
  _PRINTLN_DRIVE( ("[STOP] CarStop()") );
  motor(RR, RELEASE, 0);
  motor(FR, RELEASE, 0);
  motor(FL, RELEASE, 0);
  motor(RL, RELEASE, 0);
  delay( mSecDelayAfter );
}

/****************************************************\
  void moveForward( mSec )
\****************************************************/
void moveForward(int mSecDelayAfter) {
  _PRINTLN_DRIVE( ("[FORWARD] moveForward( " ), (mSecDelayAfter), (" )") );
  motor(RR, FORWARD, MaxMotorSpeedStrait*255 );
  motor(FR, FORWARD, MaxMotorSpeedStrait*255 );
  motor(FL, FORWARD, MaxMotorSpeedStrait*255 );
  motor(RL, FORWARD, MaxMotorSpeedStrait*255 );
  delay( mSecDelayAfter );
}

/****************************************************\
  void moveLeft( mSec )
\****************************************************/
void moveLeft(int mSecDelayAfter) {
  _PRINTLN_DRIVE( ("---- moveLeft( "), (mSecDelayAfter), (" )") );
  motor(RR, FORWARD, MaxMotorSpeedCurve*255 );
  motor(FR, FORWARD, MaxMotorSpeedCurve*255 );
  motor(FL, BACKWARD, MaxMotorSpeedCurve*255 );
  motor(RL, BACKWARD, MaxMotorSpeedCurve*255 );
  delay( mSecDelayAfter );
}

/****************************************************\
  void moveRight( mSec )
\****************************************************/
void moveRight(int mSecDelayAfter) {
  _PRINTLN_DRIVE( ("---- moveRight( "), (mSecDelayAfter), (" )") );
  motor(RR, BACKWARD, MaxMotorSpeedCurve*255 );
  motor(FR, BACKWARD, MaxMotorSpeedCurve*255 );
  motor(FL, FORWARD, MaxMotorSpeedCurve*255 );
  motor(RL, FORWARD, MaxMotorSpeedCurve*255 );
  delay( mSecDelayAfter );
}
