// For reading IMU data
// Note: only use ElectronicCats version of MPU6050, 
//       to avoid conflict with the i2cdevlib version.
#include "I2Cdev.h"   // must be installed as lib
#include "MPU6050.h"  // use IMU_Zero example from MPU6050 to set offset values onto device
#include "math.h"

// For EKF
#include "BasicLinearAlgebra.h"
#include "math_utils.h"
#include "ImuEncEKF.h"

// Arduino Wire library is required if I2Cdev
// I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define M_PI 3.14159265358979323846
#define GRAV 9.81

// Initialize class objects
ImuEncEKF filter;
MPU6050 accelgyro;
unsigned long time_f;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  /* AFS_SEL | Full Scale Range | LSB Sensitivity
   * --------+------------------+----------------
   * 0       | +/- 2g           | 16384 LSB/g
   * 1       | +/- 4g           |  8192 LSB/g
   * 2       | +/- 8g           |  4096 LSB/g
   * 3       | +/- 16g          |  2048 LSB/g
   */
  accelgyro.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_4);
  
  /* FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131.0 LSB/deg/s
   * 1      | +/- 500 degrees/s  |  65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s |  32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s |  16.4 LSB/deg/s
   */
  accelgyro.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  time_f = micros();
}

void loop() {
  // read raw accel/gyro measurements from device
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate dt
  unsigned long time_i = time_f;
  time_f = micros();
  float dt = (float)(time_f - time_i) / 1000000.0f;  //microseconds to seconds
  //Serial.println(dt,4); // check dt

  float a_scale = GRAV / 8192.0f;                              // scale factor to convert accel into units of m/s^2
  float g_scale = M_PI / (180.0f * 65.5f);                     // scale factor to convert gyro into units of rad/s

  filter.setIMUmeas(ax, ay, az, gx, gy, gz, a_scale, g_scale);
  filter.propagateImuState(dt);
  BLA::Matrix<16,1,float> X_est = filter.getState();
  printQuaternion(X_est);
}

void printQuaternion(BLA::Matrix<16,1,float> X_est) {
  float qx, qy, qz, qw;
  qx = X_est(0);
  qy = X_est(1);
  qz = X_est(2);
  qw = X_est(3);
  Serial.print(qx, 4);
  Serial.print(",");
  Serial.print(qy, 4);
  Serial.print(",");
  Serial.print(qz, 4);
  Serial.print(",");
  Serial.print(qw, 4);
  Serial.println(",");
}
