/*
* This code gives the orientation of an Adafruit BNO055 9-DOF IMU sensor. 
* The code has been made for a solution to our bachelor thesis. Any credits to guides and references can
* be found the in README.md file.
*/


// Include the libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Comment out to turn off debug
//#define DEBUG 0
// Comment out to turn off data values printing
//#define PRINT_DATA_VALUES 0

// Define number of samples per second
#define SAMPLES_PER_SECOND 20
// Define the sample rate of the IMU in milliseconds
#define IMU_SAMPLE_RATE_MS 1000/SAMPLES_PER_SECOND
// Size of dataFrame array
#define DATA_FRAME_SIZE 9

Adafruit_BNO055 IMU_SENSOR = Adafruit_BNO055();

int16_t velocity_x = 0;
int16_t velocity_y = 0;
int16_t velocity_z = 0;

int16_t current_placement_x = 0;
int16_t current_placement_y = 0;
int16_t current_placement_z = 0;

unsigned long nextTimeout = 0;

int i = DATA_FRAME_SIZE;

void setup() {
  // Start serial monitor with baud rate = 115200
  Serial.begin(115200);
  // Initialize the IMU
  IMU_SENSOR.begin();
  // Delay for 1000 milliseconds to ensure that the IMU has started
  delay(1000);
  // Get the temperature from the IMU Sensor
  int8_t temp = IMU_SENSOR.getTemp();
  // Tell the IMU to use the external crystal on the board instead of the crystal in the chip.
  IMU_SENSOR.setExtCrystalUse(true);
#ifdef DEBUG
  Serial.println("DEBUG: Setup complete..");
  Serial.print("DEBUG: Started the IMU with a sample rate of ");
  Serial.print(SAMPLES_PER_SECOND);
  Serial.println("samples per second..");
#endif
}

void loop() {
  startTimer(IMU_SAMPLE_RATE_MS);
  while (!timerHasExpired()) {
    
  }
  imu::Vector<3> acc = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
#ifdef DEBUG
  Serial.println("DEBUG: Successfully extracted IMU values");
#endif
#ifdef PRINT_DATA_VALUES
  Serial.println("***** ACCELERATION *****");
  Serial.print("x: ");
  Serial.println(acc.x());
  Serial.print("y: ");
  Serial.println(acc.y());
  Serial.print("z: ");
  Serial.println(acc.z());

  Serial.println("***** GYROSCOPE *****");
  Serial.print("x: ");
  Serial.println(gyro.x());
  Serial.print("y: ");
  Serial.println(gyro.y());
  Serial.print("z: ");
  Serial.println(gyro.z());

  Serial.println("***** MAGNETOMETER *****");
  Serial.print("x: ");
  Serial.println(mag.x());
  Serial.print("y: ");
  Serial.println(mag.y());
  Serial.print("z: ");
  Serial.println(mag.z());
#endif

float dataFrame[DATA_FRAME_SIZE] = {acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z(), mag.x(), mag.y(), mag.z()};
for(int i = 0; i < DATA_FRAME_SIZE; i++) {
  Serial.print(dataFrame[i]);
  Serial.print(" "); 
}
Serial.println(" ");

#ifdef PRINT_DATA_VALUES
  Serial.println("***** VELOCITY *****");
  Serial.print("x: ");
  Serial.println(velocity_x);
  Serial.print("y: ");
  Serial.println(velocity_y);
  Serial.print("z: ");
  Serial.println(velocity_z);

  Serial.println("***** POSITION *****");
  Serial.print("x: ");
  Serial.println(current_placement_x);
  Serial.print("y: ");
  Serial.println(current_placement_y);
  Serial.print("z: ");
  Serial.println(current_placement_z);
#endif
  
  //delay(IMU_SAMPLE_RATE_MS);
}

//----- Timer functions ----

/**
   Checks if the timer has expired. If so, true is returned.
   If the timer has not yet "expired", false is returned.

   @return true if timer expired, false if not.
*/

boolean timerHasExpired()
{
  boolean result = false;
  if (millis() > nextTimeout)
  {
    result = true;
  }
  else
  {
    result = false;
  }
  return result;
}


/**
   Starts the timer and set the timer to expire after
   the number of milliseconds given by the parameter timeout.

   @param timeout The number of milliseconds until the timer will expire.
*/
void startTimer(int timeout)
{
  nextTimeout = millis() + timeout;
}
