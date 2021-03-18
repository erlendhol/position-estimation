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

#define TRANSMIT_RAW_DATA 1

Adafruit_BNO055 IMU_SENSOR = Adafruit_BNO055();

#define DATA_FRAME_SIZE 13
unsigned long startTime = 0;
unsigned long currentTime = 0;

double x_pos = 0.000000;
double y_pos = 0.000000;
double z_pos = 0.000000;
double lin_acc_x_offset = 0.000000;
double lin_acc_y_offset = 0.000000;
double lin_acc_z_offset = 0.000000;
int n_offset_iterations = 1000;
bool run_offset = true;

char lin_acc_str[] = "Linear Accelereation: ";
char ang_vel_str[] = "Angular Velocity: ";
char eul_ang_str[] = "Euler Angles: ";
char trans_pos_str[] = "Translatoric Position: ";



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

  currentTime = millis();
  double dt = currentTime - startTime;
  startTime = currentTime;
  
  imu::Vector<3> acc = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> lin_acc = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> euler = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_EULER);
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

if(run_offset) {
  for(int N = 0; N < n_offset_iterations; N++) {
    imu::Vector<3> lin_acc = IMU_SENSOR.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    lin_acc_x_offset = lin_acc_x_offset + lin_acc.x();
    lin_acc_y_offset = lin_acc_y_offset + lin_acc.y();
    lin_acc_z_offset = lin_acc_z_offset + lin_acc.z();
    delay(10);
  }

  lin_acc_x_offset = lin_acc_x_offset / n_offset_iterations;
  lin_acc_y_offset = lin_acc_y_offset / n_offset_iterations;
  lin_acc_z_offset = lin_acc_z_offset / n_offset_iterations;

  Serial.println(lin_acc_x_offset);
  Serial.println(lin_acc_y_offset);
  Serial.println(lin_acc_z_offset);  
  run_offset = false;
}

double dt_in_seconds = dt / 1000;
double dt_squared = dt_in_seconds * dt_in_seconds;



x_pos = x_pos + ((lin_acc.x() - lin_acc_x_offset) * dt_squared);
y_pos = y_pos + ((lin_acc.y() - lin_acc_y_offset) * dt_squared);
z_pos = z_pos + ((lin_acc.z() - lin_acc_z_offset) * dt_squared);

#ifdef TRANSMIT_RAW_DATA
  double dataframe[DATA_FRAME_SIZE] = {currentTime, lin_acc.x(), lin_acc.y(), lin_acc.z(), 
                                                    gyro.x(), gyro.y(), gyro.z(), 
                                                    euler.x(), euler.y(), euler.z(),
                                                    x_pos, y_pos, z_pos};

for(int i = 0; i < DATA_FRAME_SIZE; i++)  {
  Serial.print(dataframe[i]);
  if(i < DATA_FRAME_SIZE - 1) {
    Serial.print(", ");
  }
}
Serial.print('\n');
#endif

// Print values 
//Serial.print(lin_acc_str);
//Serial.print(lin_acc.x());
//Serial.print(", ");
//Serial.print(lin_acc.y());
//Serial.print(", ");
//Serial.println(lin_acc.z());
//
//Serial.print(ang_vel_str);
//Serial.print(gyro.x());
//Serial.print(", ");
//Serial.print(gyro.y());
//Serial.print(", ");
//Serial.println(gyro.z());
//
//Serial.print(eul_ang_str);
//Serial.print(euler.x());
//Serial.print(", ");
//Serial.print(euler.y());
//Serial.print(", ");
//Serial.println(euler.z());
//
//Serial.print(trans_pos_str);
//Serial.print(x_pos);
//Serial.print(", ");
//Serial.print(y_pos);
//Serial.print(", ");
//Serial.println(z_pos);
//
//Serial.print("Delta Time: ");
//Serial.println(dt);
//Serial.println("--------------");
  
delay(500);

}

