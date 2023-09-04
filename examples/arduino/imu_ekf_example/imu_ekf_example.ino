// For reading IMU data
#include "I2Cdev.h"   // must be installed as lib
#include "MPU6050.h"  // must be installed as lib
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
int16_t ax, ay, az;
int16_t gx, gy, gz;
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
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // Options for accel are +/- 2,4,8,16
                                                         // Accel has sensitivity scale factors of 16384,8192,4096,2048 respectively
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // Options fo gyro are +/- 250,500,1000,2000
                                                         // Gyro has sensitivity scale factors of 131,65.5,32.8,16.4, respectively

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  time_f = micros();
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate dt
  unsigned long time_i = time_f;
  time_f = micros();
  float dt = (float)(time_f - time_i) / 1000000.0f;  //microseconds to seconds
  //Serial.println(dt,4); // check dt

  float a_scale = GRAV / 8192.0f;                              // scale factor to convert accel into units of m/s^2
  float g_scale = M_PI / (180.0f * 65.5f);                     // scale factor to convert gyro into units of rad/s
  gx = gx + 0.13f/g_scale; // hack bias
  //printMeasurements(ax,ay,az,gx,gy,gz,a_scale,g_scale);

  filter.setIMUmeas((float)(ax * a_scale), (float)(ay * a_scale), (float)(az * a_scale), (float)(gx * g_scale), (float)(gy * g_scale), (float)(gy * g_scale));
  filter.RK4(dt);
  //BLA::Matrix<16,1,float> X_est = filter.getState();
  filter.printState();
}

inline void printMeasurements(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float a_scale, float g_scale) {
  Serial.print("ax ");
  Serial.print(ax * a_scale);
  Serial.print(" ay ");
  Serial.print(ay * a_scale);
  Serial.print(" az ");
  Serial.print(az * a_scale);
  Serial.print(" gx ");
  Serial.print(gx * g_scale);
  Serial.print(" gy ");
  Serial.print(gy * g_scale);
  Serial.print(" gz ");
  Serial.print(gz * g_scale);
  Serial.println();
}