#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)
#define NED_AXIS_MAP (0x21)
#define NED_AXIS_SIGN (0x01)

/* Set the values of the configuration registers of the sensors */
#define GYRO_CONFIG0 (0x38)
#define GYRO_CONFIG1 (0x00)
#define ACCEL_CONFIG (0x0F)
#define MAG_CONFIG (0x7D)

const int trigPin = 11;           //connects to the trigger pin on the distance sensor
const int echoPin = 12;           //connects to the echo pin on the distance sensor

float distance = 0;               //stores the distance measured by the distance sensor
float delta_t = 0.05;
float time_stamp = 0;
float last_time_stamp = 0;

boolean magnetometer = false;
boolean gyroscope = false;
boolean linear_acceleration = false;
boolean accelerometer = false;
boolean accel_cal = false;
boolean euler_meas = false;
boolean gravity_meas = false;
boolean acc_mag_gyro = true;
boolean display_offsets = false;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  //pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity
  //pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor
  //pinMode(13, OUTPUT);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  /* Update the sensor config registers */
  bno.setMode(0x00);
  delay(100);
  //bno.write8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR, 0x80);
  bno.setAxisRemap(0x24);
  bno.setAxisSign(0x02);
  //delay(25);
  //Serial.println(bno.read8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR));
  delay(25);
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 0x01);
  delay(100);
  //bno.write8(Adafruit_BNO055::MAG_CONFIG_ADDR, MAG_CONFIG);
  bno.write8(Adafruit_BNO055::ACCEL_CONFIG_ADDR, ACCEL_CONFIG);
  bno.write8(Adafruit_BNO055::GYRO_CONFIG0_ADDR, GYRO_CONFIG0);
  bno.write8(Adafruit_BNO055::GYRO_CONFIG1_ADDR, GYRO_CONFIG1);
  
 

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  
  /* Set BNO055 to AMG mode (accel-mag-gyro) */
  bno.setMode(0x07);
  //bno.setMode(0x0C);
  
  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");

  bno.setExtCrystalUse(false);

//  if(magnetometer)
//  {
//    Serial.println("time, Magnetometer X, Magnetometer Y, Magnetometer Z");
//  }
//
//  if(gyroscope)
//  {
//    Serial.println("time, Gyro X, Gyro Y, Gyro Z");
//  }
//
//  if(accelerometer)
//  {
//    Serial.println("time, Acceleration X, Acceleration Y, Acceleration Z");
//  }
//
//  if(linear_acceleration)
//  {
//    Serial.println("time, LinearAcc X, LinearAcc Y, LinearAcc Z, distance");
//  }
//
//  if(euler_meas)
//  {
//    Serial.println("time, Euler X, Euler Y, Euler Z");
//  }
//
//  if(gravity_meas)
//  {
//    Serial.println("time, Gravity X, Gravity Y, Gravity Z");
//  }
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  //Serial.println("time, Acceleration x, Acceleration y, acceleration z, distance");
  //Serial.println("time, Acceleration x, Acceleration y, Acceleration z, Magnetometer x, Magnetometer y, Magnetometer z, Euler x, Euler y, Euler z");
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
//  time_stamp = millis();
//  delta_t = (time_stamp - last_time_stamp)/1000;
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - degrees/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> acc_lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  double xm_off, ym_off, zm_off, xm_cal, ym_cal, zm_cal, xa_cal, ya_cal, za_cal;


  xm_off = -magneto.x() + 75.81;
  ym_off = -magneto.y() + 124.53;
  zm_off = -magneto.z() - 96.595;

  //Scale factor x:  1.0636341343
  //Scale factor y:  1.175314997967755
  //Scale factor z:  0.8271357742181541

  xm_cal = 0.8928725038*xm_off;
  ym_cal = 1.3351249540*ym_off;
  zm_cal = 0.8841532049*zm_off;

  float zerog_x = -0.1415890312988175;
  float zerog_y = 0.16061132197367467;
  float zerog_z = -0.020036616161616116;
  float scale_x = 10.013536596841888;
  float scale_y = 9.9447985140919;
  float scale_z = 9.911741161616163;
  float g_const = 9.81;

  xa_cal = (acc.x() - zerog_x) * g_const / scale_x;
  ya_cal = (acc.y() - zerog_y) * g_const / scale_y;
  za_cal = (acc.z() - zerog_z) * g_const / scale_z;
  
  //distance = getDistance();   //variable to store the distance measured by the sensor


   if(magnetometer)
  {
    Serial.println(millis() + String(",") + (-magneto.x()) + String(",") + (-magneto.y()) + String(",") + (-magneto.z()));
  }

  if(gyroscope)
  {
    Serial.println(millis() + String(",") + gyro.x() + String(",") + gyro.y() + String(",") + gyro.z());
  }

  if(accelerometer)
  {
    Serial.println(millis() + String(",") + acc.x() + String(",") + acc.y() + String(",") + acc.z());
  }

  if(accel_cal)
  {
    Serial.println(millis() + String(",") + xa_cal + String(",") + ya_cal + String(",") + za_cal);
  }

  if(linear_acceleration)
  {
    Serial.println(millis() + String(",") + acc_lin.x() + String(",") + acc_lin.y() + String(",") + acc_lin.z() + String(",") + distance);
  }

  if(euler_meas)
  {
    Serial.println(millis() + String(",") + euler.x() + String(",") + euler.y() + String(",") + euler.z());
  }

  if(gravity_meas)
  {
    Serial.println(millis() + String(",") + gravity.x() + String(",") + gravity.y() + String(",") + gravity.z());
  }

  if(acc_mag_gyro)
  {
    Serial.println(delta_t + String(",") + (xa_cal) + String(",") + (ya_cal) + String(",") + (za_cal) + String(",") + (xm_cal) + String(",") + (ym_cal) + String(",") + (zm_cal) + String(",") + (gyro.x()) + String(",") + (gyro.y()) + String(",") + (gyro.z()) + String(",") + 10);
    //Serial.println(delta_t + String(",") + (acc.x()) + String(",") + (acc.y()) + String(",") + (acc.z()) + String(",") + (xm_cal) + String(",") + (ym_cal) + String(",") + (zm_cal) + String(",") + (gyro.x()) + String(",") + (gyro.y()) + String(",") + (gyro.z()) + String(",") + 10);
    //Serial.write(delta_t + String(",") + (acc.x()) + String(",") + (acc.y()) + String(",") + (acc.z()) + String(",") + (xm_cal) + String(",") + (ym_cal) + String(",") + (zm_cal) + String(",") + (gyro.x()) + String(",") + (gyro.y()) + String(",") + (gyro.z()) + String(",") + 10);
  }

//  if(acc_mag_gyro)
//  {
//    Serial.println(millis() + String(",") + (acc.x()+0.34) + String(",") + (acc.y()+0.46) + String(",") + (acc.z()+0.3) + String(",") + (magneto.x()+33.25) + String(",") + (magneto.y()+2.9375) + String(",") + (magneto.z()+51.6875) + String(",") + (gyro.x()+0.125) + String(",") + (gyro.y()+0.125) + String(",") + (gyro.z()+0.125) + String(",") + euler.z() + String(",") + euler.y() + String(",") + euler.x());
//  }
  //Serial.println(millis() + String(",") + acc.x() + String(",") + acc.y() + String(",") + acc.z() + String(",") + magneto.x() + String(",") + magneto.y() + String(",") + magneto.z() + String(",") + euler.x() + String(",") + euler.y() + String(",") + euler.z());
  //Serial.println(gyro.x());
  //Linear Acceleration
  //Serial.println(millis() + String(",") + acc_lin.x() + String(",") + acc_lin.y() + String(",") + acc_lin.z() + String(",") + distance);
  //Gyroscope
  //Serial.println(millis() + String(",") + gyro.x() + String(",") + gyro.y() + String(",") + gyro.z());
  //Magnetometer
  //Serial.println(millis() + String(",") + magneto.x() + String(",") + magneto.y() + String(",") + magneto.z());
  //Gravity
  //Serial.println(millis() + String(",") + gravity.x() + String(",") + gravity.y() + String(",") + gravity.z());

//  Serial.print("Distance: " + String(distance));     //print the distance that was measured
//  Serial.println(" cm");      //print units after the distance
  
  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(acc.x());
//  Serial.print(" Y: ");
//  Serial.print(acc.y());
//  Serial.print(" Z: ");
//  Serial.print(acc.z());
//  Serial.print("\t\t");

  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
//  last_time_stamp = time_stamp;
  //Serial.flush();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime * 2.54 / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calculatedDistance;              //send back the distance that was calculated
}
