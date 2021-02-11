// Include the library
#include <SharpIR.h>

// Comment out to turn off debug
#define DEBUG 0

// Pin declaration
#define IR_PIN_1 A0
#define IR_PIN_2 A1

// Define the model used in the SharpIR library
#define model 430

// Varibales for the distances
uint8_t distanceIR1;
uint8_t distanceIR2;

// Create instances of the sensors
SharpIR sensor1 = SharpIR(IR_PIN_1, model);
SharpIR sensor2 = SharpIR(IR_PIN_2, model);
  
void setup() {
  Serial.begin(9600);
}

void loop() {
  distanceIR1 = sensor1.distance();
  distanceIR2 = sensor2.distance();
  Serial.print("Distance sensor 1: ");
  Serial.print(distanceIR1);
  Serial.println(" cm");
  Serial.print("Distance sensor 2: ");
  Serial.print(distanceIR2);
  Serial.println(" cm");
  delay(500);
}
