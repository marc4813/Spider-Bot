#include <Arduino.h>
#include <PS4Controller.h>


TaskHandle_t Rstep;
TaskHandle_t Lstep;
void WriteStepsL(void *pvParameters);
void WriteStepsR(void *pvParameters);

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
  
  // Connect PS4 controller
  Serial.begin(115200);
  char add[] = "01:02:03:04:05:06";
  PS4.begin(add);

  xTaskCreatePinnedToCore(
        WriteStepsR, 
        "Write Right steps", 
        10000,
        NULL,
        1,
        &Rstep,
        0);

  xTaskCreatePinnedToCore(
        WriteStepsL, 
        "Write Left steps", 
        10000,
        NULL,
        1,
        &Lstep,
        1);

}

void WriteStepsL(void *pvParameters)
{
  for(;;)
  {
    if(PS4.isConnected())
    {
   
      // Get raw y values from left stick
      leftStickY = PS4.data.analog.stick.ly;
    
      Serial.print("LStick: ");
      Serial.print(leftStickY, DEC);

      // Prevent values from being -128
      if (leftStickY < 0)
        leftStickY = leftStickY + 1;

   
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

      // Convert stick position to magnitude
      leftStickY = abs(leftStickY);

      // Map stick positions to stepper speed
      leftStepSpeed = map(leftStickY, DEADZONE_SIZE, 127, MAX_STEP_SPEED, MIN_STEP_SPEED);
   
      // Overwrite if stick is within deadzone
      if (leftStickY < DEADZONE_SIZE)
        leftStepSpeed = 0;

      if (rightStickY < DEADZONE_SIZE)
        rightStepSpeed = 0;

    
      // Write stepper speeds to motors
      if(leftStepSpeed != 0)
      {  
        digitalWrite(STEP_L,HIGH);
        delayMicroseconds(leftStepSpeed);
        digitalWrite(STEP_L, LOW);
        delayMicroseconds(leftStepSpeed);

      }

      Serial.print("\tLeft Steps/Sec:\t");
      Serial.print(leftStepSpeed);
    }

    else
    {
      Serial.println("Controller not connected...");
      delay(1000);
    }
  }
}

void WriteStepsR(void *pvParameters)
{
  for(;;)
  {
    if(PS4.isConnected())
    {
   
      // Get raw y values from left stick
      rightStickY = PS4.data.analog.stick.ry;
    
      Serial.print("RStick: ");
      Serial.print(rightStickY, DEC);

      // Prevent values from being -128
      if (rightStickY < 0)
        rightStickY = rightStickY + 1;

   
      // Establish motor directions
      // "* MOTOR_X_DIRECTION" allows motor to be reversed through code instead of wiring

      if (rightStickY * MOTOR_L_DIRECTION < 0)
      {
         // Direction A (Clockwise)
        digitalWrite(DIR_R, HIGH);
      }
   
      else
      {
        // Direction B (Counterclockwise)
        digitalWrite(DIR_R, LOW);
      }

      // Convert stick position to magnitude
      rightStickY = abs(rightStickY);

      // Map stick positions to stepper speed
      rightStepSpeed = map(rightStickY, DEADZONE_SIZE, 127, 0, MIN_STEP_SPEED);
   
      // Overwrite if stick is within deadzone
      if (rightStickY < DEADZONE_SIZE)
        rightStepSpeed = 0;

    
      // Write stepper speeds to motors
      if(rightStepSpeed != 0)
      {  
        digitalWrite(STEP_L,HIGH);
        delayMicroseconds(rightStepSpeed);
        digitalWrite(STEP_L, LOW);
        delayMicroseconds(rightStepSpeed);

      }

      Serial.print("\tLeft Steps/Sec:\t");
      Serial.print(leftStepSpeed);
    }

    else
    {
      Serial.println("Controller not connected...");
      delay(1000);
    }
  }
}

void loop()
{
  
}