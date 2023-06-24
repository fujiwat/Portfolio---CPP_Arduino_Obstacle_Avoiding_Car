# Portfolio---CPP_Arduino_Obstacle_Avoiding_Car

C++ Arduino program, 600 lines
2 weeks work.
```
## SAMPLE PROGRRAM PROVIDED BY MANUFACTURER
void motor(int nMotor, int command, int speed)
void motor_output (int output, int high_low, int speed)
void shiftWrite(int output, int high_low)

## Class IrSensor
    IrSensor(char *name, int pin, int maxValue)   // constructor
    float getAnalogValue()                        // get IR value in analog 0.0 to 1.0
    int digitalRead()                             // get IR value in digital LOW/HIGH
## class Sonar
    Sonar(char *name, int pinTrig, int pinEcho)   // constructor
    int getDistanceMm()                           // get Distance in mm
    int getDistanceMmLR(Sonar::Dir dir)           // get distance in mm (Direction Left or Right)
    void SetAngle(int angle)                      // Set Sonner Angle

## Initialize
void setup()

## loop
void loop()

## functions
void obstacleAvoidAndGo()                         // Obstacle detection, avoid it, go
void CarTurnLeft()                                // Car Turn Left
void CarTurnRight()                               // Car Turn Right
void CarStop(int mSecDelayAfter)                  // Car Stop
void moveForward(int mSecDelayAfter)              // basic function Move Forward
void moveLeft( mSec )                             // basic function Move Left
void moveRight( mSec )                            // basic function Move Right
```
