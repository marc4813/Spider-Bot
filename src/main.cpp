#include <Arduino.h>
#include <PS4Controller.h>

// Define stepper motor connections and steps per revolution:
#define R_DIR 14
#define R_STEP 27
#define L_DIR 32
#define L_STEP 26
#define stepsPerRevolution 200
#define DEADZONE_SIZE 15
#define MAX_STEP_DELAY 2500 // Max microseconds per step (slowest speed)
#define MIN_STEP_DELAY 300 // Min microseconds per step (fastest speed)
#define MOTOR_R_DIRECTION 1
#define MOTOR_L_DIRECTION 1

int delayTime = 500;
int dir = 0;

TaskHandle_t Rstep;
TaskHandle_t Lstep;
void WriteStepsL(void *pvParameters);
void WriteStepsR(void *pvParameters);
void writeSteps(int direction, int stepPin, int dirPin);


void setup() {
  // Declare pins as output:
  pinMode(R_DIR, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(L_STEP, OUTPUT);
  // Connect to PS4
  Serial.begin(115200);
  char bluetoothMAC[] = "01:02:03:04:05:06";
  PS4.begin(bluetoothMAC);

  // Each core on the esp-32 is responsible for each side of the robot
  xTaskCreatePinnedToCore(WriteStepsR, "Write Right steps", 
    10000, NULL, 1, &Rstep, 0);

  xTaskCreatePinnedToCore(WriteStepsL, "Write Left steps", 
    10000, NULL, 1, &Lstep, 1);
}

void loop(){}

void WriteStepsL(void *pvParameters) {
  writeSteps(0, L_STEP, L_DIR);
}

void WriteStepsR(void *pvParameters) {
  writeSteps(1, R_STEP, R_DIR);
}

void writeSteps(int direction, int stepPin, int dirPin) {
  while(true) {
    if(!PS4.isConnected()) {
      Serial.print("PS4 Disconnected");
      delay(100);
    }
    else {
      int stickVal = 0;
      // Get input from ps4 controller
      if(direction)
        stickVal = PS4.data.analog.stick.ry;
      else
        stickVal = PS4.data.analog.stick.ly;

      if (stickVal * (direction ? MOTOR_R_DIRECTION : MOTOR_L_DIRECTION) > 0)
        digitalWrite(dirPin, HIGH);
      else
        digitalWrite(dirPin, LOW);

      if (stickVal < 0)
         stickVal = (int) abs(stickVal + 1);
      
     // Calculate deadzone
      if (stickVal < DEADZONE_SIZE)
         stickVal = 0;

     // Map stick magnitudes to stepper delays (high stick value -> low delay, low stick value -> high delay)
      delayTime = map(max(stickVal, DEADZONE_SIZE), DEADZONE_SIZE, 128, MAX_STEP_DELAY, MIN_STEP_DELAY);
      if(stickVal != 0) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(delayTime);
      }
      
    }
    vTaskDelay(1);
  }
  
}