// For reading IMU data
// Note: only use ElectronicCats version of MPU6050,
//       to avoid conflict with the i2cdevlib version.
#include "I2Cdev.h"
#include "MPU6050.h"
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
#define FS_SEL 2  // 0,1,2 or 3 (see above chart)
#define M_PI 3.14159265358979323846
#define GRAV 9.81
#define TO_DEG 57.2957795131 // 180/pi
#define TO_RAD 0.01745329252 // pi/180

// Initialize class objects
ImuEncEKF filter;
MPU6050 accelgyro;
unsigned long time_f, time_start;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float a_scl;
float g_scl;

// Calibration parameters
const float SC_[9] = {0.9970304311860567,-0.014837314285154361,0.013707847451761646,0.014037313965050303,0.9953090836487588,-0.006179516542020768,-0.01134369448348627,0.004215263393860424,0.9894563653949066}; // scale-factor/off-axis accel calib matrix
const float bias_[6] = {0.4182762357568723,-0.09544905735395014,-0.46627117454304434, 0.11861324999999993,0.10363050000000004,-0.03186133333333337}; // imu bias: b_ax, b_ay, b_az, b_gx, b_gy, b_gz

//PID gains
// float kp = 60.0f;
// float kd = 1.4f;
// float k_dist = 5.0f;
// float k_vel = 2.0f;
float k_deg = 70.0f;
float k_rate = 1.6f;
float k_dist = 550.0f;
float k_vel = 150.0f;
float k_int = 0.0f;
bool PID_started = false;
bool PID_restart = false;
float deg_err = 0.0f;
float dist_err = 0.0f;
float dist_err_integral = 0.0f;
float dist = 0.0f;
float vel_hist[3] = {0.0, 0.0, 0.0};

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
const byte c1_ma = 2; // for encoder
const byte c2_ma = 4;
// Motor B connections
int enB = 10;
int in3 = 6;
int in4 = 5;
const byte c1_mb = 3; //Pin 2 will be the interrupt pin
const byte c2_mb = A2; //c1 -> encoder 1, c2 -> encoder 2

// Encoder vars
// /*
int c1_state_ma;
int c2_state_ma;
int c1_state_mb;
int c2_state_mb;
bool CCW_ma;
bool CCW_mb;
float rpm_ma = 0;
float rpm_mb = 0;
float deg_chng_ma = 0;
float deg_chng_mb = 0;
int intrp_ma = 0;     // number of times entering the interrupt
int intrp_mb = 0; 
int ax_rev_ma = 0;    // number of axle revolutions
int ax_rev_mb = 0;
float ax_ang_ma = 0;  // angle in degrees of d-shaft
float ax_ang_mb = 0;
float prev_ax_ang_ma;
float prev_ax_ang_mb;

// wheel/encoder data
const float gr = 62.39f;  // gear ratio -> Motor 1 gr: 62.39 | Motor 2 gr: 62.39
const int I = 22;        // Interrupts per revolution of encoder wheel -> original is 22
                         // In this case, an interrupt happens when c1 goes from 0 -> 1 or 1 -> 0
const float radius = 0.0475f; // wheel radius (meters)
// */

#define OUTPUT_READABLE_ACCELGYRO

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // Sensitivity settings
  accelgyro.setFullScaleAccelRange(AFS_SEL);
  accelgyro.setFullScaleGyroRange(FS_SEL);
  float a_sens[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f }; // sensitivity options
  float g_sens[4] = { 131.0f, 65.5f, 32.8f, 16.4f };
  a_scl = GRAV / a_sens[AFS_SEL];           // scale factor to convert accel into units of m/s^2
  g_scl = M_PI / (180.0f * g_sens[FS_SEL]); // scale factor to convert gyro into units of rad/s

  // Set calibration bias values (bax, bay, baz, bgx, bgy, bgz)
  // filter.applyImuBias(0.40652113f, -0.12045737f, -0.46506827f, 0.11012042f, 0.10504308f, -0.03172367f);
  filter.applyImuBias(bias_[0], bias_[1], bias_[2], bias_[3], bias_[4], bias_[5]);

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Encoder setup
  // /*
  CCW_ma = true;
  CCW_mb = true;
  pinMode(c1_ma, INPUT);
  pinMode(c2_ma, INPUT);
  pinMode(c1_mb, INPUT);
  pinMode(c2_mb, INPUT);
  // */

  // interrupts if c1 rises 0 -> 1
  attachInterrupt(digitalPinToInterrupt(c1_ma), angular_test_ma, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c1_mb), angular_test_mb, CHANGE);
  
  // Start timer
  time_start = micros();
  time_f = time_start;
}

void loop() {
  // Read raw data
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate dt
  unsigned long time_i = time_f;
  time_f = micros();
  float dt = (float)(time_f - time_i) / 1000000.0f;  // loop time (sec)
  float dur = (float)(time_i - time_start) / 1000000.0f;  // total time duration (sec)
  //Serial.println(dt,4); // check dt

  // RKF & Madgwick method
  filter.processImuMeas(ax*a_scl, ay*a_scl, az*a_scl, gx*g_scl, gy*g_scl, gz*g_scl, SC_);
  filter.propagateImuState(dt, dur);
  BLA::Matrix<4,1,float> q = filter.getQuat();
  float sq2_2 = sqrt(2)/2;
  float q_rot[4] = {sq2_2*q(0) - sq2_2*q(2), -sq2_2*q(3) + sq2_2*q(1), sq2_2*q(0) + sq2_2*q(2), -sq2_2*q(3) - sq2_2*q(1)}; // rot q to body frame
  float pitch = -asin(2*(q_rot[3]*q_rot[1] - q_rot[0]*q_rot[2]));
  // Serial.println(pitch);

  // Encoder calculations
  deg_chng_ma = (ax_ang_ma - prev_ax_ang_ma);
  deg_chng_mb = (ax_ang_mb - prev_ax_ang_mb);
  wrap_deg_chng(deg_chng_ma);
  wrap_deg_chng(deg_chng_mb);
  // rpm_ma = (deg_chng_ma*500)/(3*dt);
  prev_ax_ang_ma = ax_ang_ma;
  // rpm_mb = (deg_chng_mb*500)/(3*dt);
  prev_ax_ang_mb = ax_ang_mb;
  float prev_dist = dist;
  dist += 0.5f*radius*(-deg_chng_ma + deg_chng_mb)*TO_RAD;
  float vel_meas = (dist - prev_dist)/dt;
  float vel = 0.4f*vel_meas + 0.3f*vel_hist[2] + 0.2f*vel_hist[1] + 0.1f*vel_hist[0]; // weighted average
  vel_hist[0] = vel_hist[1];
  vel_hist[1] = vel_hist[2];
  vel_hist[2] = vel_meas;

  // wait for filter to initialize
  if (dur > 3) {
    if (PID_started) {
      dist_err_integral += dist_err*dt;
    }
    if (PID_restart)
      dist_err_integral = 0;
      PID_restart = false;
    float deg = pitch*TO_DEG;
    // Serial.println(deg);
    float prev_deg_err = deg_err;
    deg_err = deg;
    float rate_err = (deg_err - prev_deg_err)/dt;
    float dist_ref = 0.0f;
    dist_err = dist - dist_ref;
    float spd = k_deg*deg_err + k_rate*rate_err + k_dist*constrain(dist_err, -0.035, 0.035)/(constrain(abs(vel), 0.0, 0.5) + 0.5) + k_vel*constrain(vel, -1, 1) + k_int*dist_err_integral;
    if (abs(spd) < 5.0f) {
      spd = 0.0f;
    } else if (abs(deg) > 50 || isnan(deg)) {
      spd = 0.0f;
      PID_restart = true;
    }
    spd = constrain(spd, -255.0f, 255.0f);

    if (spd < 0) {
      // Motors backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    } else {
      // Motors forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    int abs_spd = (int)round(abs(spd));
    analogWrite(enA, abs_spd);
    analogWrite(enB, abs_spd);
    PID_started = true;
  }
}

void angular_test_ma() {
  c1_state_ma = digitalRead(c1_ma);
  c2_state_ma = digitalRead(c2_ma);
  if((c1_state_ma == 1 && c2_state_ma == 0) || (c1_state_ma == 0 && c2_state_ma == 1)){
    // Enter if motor is going CW -> see encoder value chart at top
    CCW_ma = false;
    intrp_ma -= 1;
  } else if((c1_state_ma == 1 && c2_state_ma == 1) || (c1_state_ma == 0 && c2_state_ma == 0)){
    // Enter if motor is going CCW -> see encoder value chart at top
    CCW_ma = true;
    intrp_ma += 1;
  }
  ax_ang_ma = float(intrp_ma)*360.0/(gr*float(I));
  if ((int) ax_ang_ma == 360){
    ax_rev_ma += 1;
    intrp_ma = 0;
    ax_ang_ma = 0;
  } else if ((int) ax_ang_ma == -360){
    ax_rev_ma -= 1;
    intrp_ma = 0;
    ax_ang_ma = 0;
  }
}

void angular_test_mb() {
  c1_state_mb = digitalRead(c1_mb);
  c2_state_mb = digitalRead(c2_mb);
  if((c1_state_mb == 1 && c2_state_mb == 0) || (c1_state_mb == 0 && c2_state_mb == 1)){
    // Enter if motor is going CW -> see encoder value chart at top
    CCW_mb = false;
    intrp_mb -= 1;
  } else if((c1_state_mb == 1 && c2_state_mb == 1) || (c1_state_mb == 0 && c2_state_mb == 0)){
    // Enter if motor is going CCW -> see encoder value chart at top
    CCW_mb = true;
    intrp_mb += 1;
  }
  ax_ang_mb = float(intrp_mb)*360.0/(gr*float(I));
  if ((int) ax_ang_mb == 360){
    ax_rev_mb += 1;
    intrp_mb = 0;
    ax_ang_mb = 0;
  } else if ((int) ax_ang_mb == -360){
    ax_rev_mb -= 1;
    intrp_mb = 0;
    ax_ang_mb = 0;
  }
}

void wrap_deg_chng(float& deg_chng) {
  if (deg_chng < -200) {
    deg_chng = 360 + deg_chng;
  } else if (deg_chng > 200) {
    deg_chng = 360 - deg_chng;
  }
}
