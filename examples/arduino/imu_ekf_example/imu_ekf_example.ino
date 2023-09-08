// For reading IMU data
// Note: only use ElectronicCats version of MPU6050, 
//       to avoid conflict with the i2cdevlib version.
#include "I2Cdev.h"
#include "MPU6050.h" // use IMU_Zero example from MPU6050 to set offset values onto device
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

/* AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 16384 LSB/g
 * 1       | +/- 4g           |  8192 LSB/g
 * 2       | +/- 8g           |  4096 LSB/g
 * 3       | +/- 16g          |  2048 LSB/g
 */

/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131.0 LSB/deg/s
 * 1      | +/- 500 degrees/s  |  65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s |  32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s |  16.4 LSB/deg/s
 */

#define AFS_SEL 1 // 0,1,2 or 3 (see above chart)
#define FS_SEL 1  // 0,1,2 or 3 (see above chart)
#define M_PI 3.14159265358979323846
#define GRAV 9.81

// Initialize class objects
ImuEncEKF filter;
MPU6050 accelgyro;
unsigned long time_f;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float a_scl;
float g_scl;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize serial communication
  Serial.begin(38400);

  // Initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // Sensitivity settings
  accelgyro.setFullScaleAccelRange(AFS_SEL);
  accelgyro.setFullScaleGyroRange(FS_SEL);
  float a_sens[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f }; // sensitivity options
  float g_sens[4] = { 131.0f, 65.5f, 32.8f, 16.4f };
  a_scl = GRAV / a_sens[AFS_SEL];           // scale factor to convert accel into units of m/s^2
  g_scl = M_PI / (180.0f * g_sens[FS_SEL]); // scale factor to convert gyro into units of rad/s

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Start timer
  time_f = micros();
}

void loop() {
  // Read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //printMeasurements();

  // Calculate dt
  unsigned long time_i = time_f;
  time_f = micros();
  float dt = (float)(time_f - time_i) / 1000000.0f;  //microseconds to seconds
  //Serial.println(dt,4); // check dt

  filter.setIMUmeas(ax*a_scl, ay*a_scl, az*a_scl, gx*g_scl, gy*g_scl, gz*g_scl);
  filter.propagateImuState(dt);
  BLA::Matrix<4,1,float> q_est = filter.getQuat();
  printQuaternion(q_est);
}

void printQuaternion(BLA::Matrix<4,1,float> q) {
  // use smaller precision (e.g. 2) for
  // better performance on Arduino nano
  Serial.print(q(0), 2); // qx
  Serial.print(",");
  Serial.print(q(1), 2); // qy
  Serial.print(",");
  Serial.print(q(2), 2); // qz
  Serial.print(",");
  Serial.print(q(3), 2); // qw
  Serial.println(",");
}

void printMeasurements() {
  Serial.print(F("ax "));
  Serial.print(ax * a_scl);
  Serial.print(F(" ay "));
  Serial.print(ay * a_scl);
  Serial.print(F(" az "));
  Serial.print(az * a_scl);
  Serial.print(F(" gx "));
  Serial.print(gx * g_scl);
  Serial.print(F(" gy "));
  Serial.print(gy * g_scl);
  Serial.print(F(" gz "));
  Serial.print(gz * g_scl);
  Serial.println();
}
