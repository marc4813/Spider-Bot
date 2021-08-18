#include <Arduino.h>
#include <PS4Controller.h>
#include <AccelStepper.h>

#define MAX_STEP_SPEED 1000 // Max steps per second
#define MIN_STEP_SPEED 500 // Min steps per second
#define STEPPER_ACCEL 200 // Max steps per second per second

#define DEADZONE_SIZE 15

#define MOTOR_R_DIRECTION 1 // Set to -1 to reverse motor direction, instead of rewiring
#define MOTOR_L_DIRECTION 1 

#define DIR_R 14 // Direction pin for Right motors
#define DIR_L 32 // Direction pin for Left motors
#define STEP_R 27 // Step pin for Right motors
#define STEP_L 26 // Step pin for Left motors

#define LED_CHANNEL_A 0
#define LED_CHANNEL_B 1

int leftStickY;
int leftStepSpeed;

int rightStickY;
int rightStepSpeed;

void setup()
{
  
  delay(1000);

  AccelStepper stepperL(AccelStepper::DRIVER,STEP_L, DIR_L);
  AccelStepper stepperR(AccelStepper::DRIVER,STEP_R, DIR_R);

  stepperL.setMaxSpeed(MAX_STEP_SPEED);
  stepperR.setMaxSpeed(MAX_STEP_SPEED);
  
  Serial.begin(115200);
  PS4.begin("01:02:03:04:05:06");
  Serial.println("Waiting for controller to connect to 01:02:03:04:05:06");

}

void loop()
{
  if(PS4.isConnected())
  {
   
    // Get raw y values from both input sticks
    leftStickY = PS4.data.analog.stick.ly;
    rightStickY = PS4.data.analog.stick.ry;
    
    Serial.print("LStick: ");
    Serial.print(leftStickY, DEC);
    Serial.print("\t RStick: ");
    Serial.print(rightStickY, DEC);

    // Prevent values from being -128
    if (leftStickY < 0)
      leftStickY = leftStickY + 1;

    if (rightStickY < 0)
      rightStickY = rightStickY + 1; 

    // Establish motor directions
    // "* MOTOR_X_DIRECTION" allows motor to be reversed through code instead of wiring

    if (leftStickY * MOTOR_L_DIRECTION < 0)
    {
       // Direction A (Clockwise)
       digitalWrite(DIR_L, HIGH);
    }
   
    else
    {
      // Direction B (Counterclockwise)
      digitalWrite(DIR_L, LOW);
    }

   
    if (rightStickY * MOTOR_R_DIRECTION < 0) 
    {
      // Direction A (Clockwise)
      digitalWrite(DIR_R, LOW);

    }
   
    else
    {
      // Direction B (Counterclockwise)
      digitalWrite(DIR_R, LOW);
    }
    
    // Convert stick position to magnitude
    leftStickY = abs(leftStickY);
    rightStickY = abs(rightStickY);

    // Map stick positions to stepper speed
    leftStepSpeed = map(leftStickY, DEADZONE_SIZE, 127, MIN_STEP_SPEED, MAX_STEP_SPEED);
    rightStepSpeed = map(rightStickY, DEADZONE_SIZE, 127, MIN_STEP_SPEED, MAX_STEP_SPEED);

    // Overwrite if stick is within deadzone
    if (leftStickY < DEADZONE_SIZE)
      leftStepSpeed = 0;

    if (rightStickY < DEADZONE_SIZE)
      rightStepSpeed = 0;

    
    // Write stepper speeds to motors
    stepperL.setSpeed(leftStepSpeed);
    stepperL.runSpeed();

    stepperR.setSpeed(rightStepSpeed);
    stepperR.runSpeed();


    Serial.print("\tLeft Steps/Sec:\t");
    Serial.print(leftStepSpeed);
    Serial.print("\tRight Steps/Sec:\t");
    Serial.print(rightStepSpeed);
  }
  else
  {
    Serial.println("Controller not connected...");
    delay(1000);
  }
  
  delay(100);

}