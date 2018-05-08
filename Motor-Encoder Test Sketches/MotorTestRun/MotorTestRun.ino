/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>
#include <Encoder.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define PWMA 32
#define AIN1 20
#define AIN2 33

#define PWMB 25
#define BIN1 30
#define BIN2 27

Encoder knobLeft(29, 31);
Encoder knobRight(28, 26);

#define STBY 9

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
 //Nothing here
   Serial.begin(57600);
  Serial.println(" Arduino Encoder Test:");
}

long positionLeft  = 0;
long positionRight = 0;

void loop()
{

  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();

   Serial.print("Start Wheel Left = ");
   Serial.print(newLeft);
   Serial.print("; Start Wheel Right = ");
   Serial.print(newRight);
   Serial.println();

   forward(motor1, motor2, 100);
   delay(5000);

   
   brake(motor1, motor2);

   delay(2000);
   
   newLeft = knobLeft.read();
   newRight = knobRight.read();
  
   Serial.print("Forward Position Wheel Left = ");
   Serial.print(newLeft);
   Serial.print("; Forward Position Wheel Right = ");
   Serial.print(newRight);
   Serial.println();
   
   delay(3000);
   
   back(motor1, motor2, -100);
   delay(5000);
   
   brake(motor1, motor2);
   delay(2000);
   
   newLeft = knobLeft.read();
   newRight = knobRight.read();
   
   Serial.print("Back Position Wheel Left = ");
   Serial.print(newLeft);
   Serial.print("; Back Position Wheel Right = ");
   Serial.print(newRight);
   Serial.println();
   delay(3000);

}
