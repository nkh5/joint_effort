#include <Stepper.h>

// 2048 steps = 1 full revolution for the 28BYJ-48
const int stepsPerRevolution = 2048;

// Initialize the stepper library on pins 8 through 11
// Remember: pins 9 and 10 are swapped for this specific motor
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  // Set the motor speed to 15 RPM (close to max speed for this motor)
  myStepper.setSpeed(15);
  Serial.begin(9600);
}

void loop() {

  myStepper.step(-4 * stepsPerRevolution);
  
  // A brief pause is highly recommended so the sudden reverse 
  // doesn't strip the delicate plastic gears inside the motor
  delay(500); 

  myStepper.step(4 * stepsPerRevolution);
  
  delay(1000); 
}