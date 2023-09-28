/*
  ImuEncEKF.cpp - Library to apply an EKF to an IMU (accel. and gyro.)
  Author - Sequoyah Walters
  Note - Uses notation from "Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight"

  State = [q, bw, v, ba, p]
        = [q_x, q_y, q_z, q_w, bw_x, bw_y, bw_z, v_x, v_y, v_z, ba_x, ba_y, ba_z, p_x, p_y, p_z]
  q:  rotation from world frame to IMU body frame
  bw: bias of angular velocity measurements
  v:  velocity of IMU body frame in world frame
  ba: bias of linear acceleration measurements
  p:  position of IMU body frame in world frame

  Error State = [th_err, bw_err, v_err, ba_err, p_err]
              = [th_x_err, th_y_err, th_z_err, bw_x_err, bw_y_err, bw_z_err, v_x_err, v_y_err, v_z_err, ba_x_err, ba_y_err, ba_z_err, p_x_err, p_y_err, p_z_err]
  th_err: q_err ~= (0.5*th_err, 1)^T, where th_err represents a small angle rotation
  bw_err: bw_err = bw - bw_estimate
*/

#include <BasicLinearAlgebra.h>
#include "ImuEncEKF.h"

#include "math_utils.h"

// ----- Public Functions ----- //

/*
 * @brief Constructor for ImuEKF
 * @note Initializes state est, error state est
 */
ImuEncEKF::ImuEncEKF()
{
}

/*
 * @brief Apply IMU calibration bias values
 */
void ImuEncEKF::applyImuBias(float ba_x, float ba_y, float ba_z,
                             float bw_x, float bw_y, float bw_z)
{
  X_est_.ba = {ba_x, ba_y, ba_z};
  X_est_.bw = {bw_x, bw_y, bw_z};
}

/*
 * @brief Set IMU measurements
 */
void ImuEncEKF::processImuMeas(float a_x_raw, float a_y_raw, float a_z_raw,
                               float w_x_raw, float w_y_raw, float w_z_raw, const float* SC_)
{
  BLA::Matrix<3,1,float> a_raw = {a_x_raw, a_y_raw, a_z_raw};
  BLA::Matrix<3,1,float> a_est = {SC_[0]*a_x_raw + SC_[1]*a_y_raw + SC_[2]*a_z_raw - X_est_.ba(0), SC_[3]*a_x_raw + SC_[4]*a_y_raw + SC_[5]*a_z_raw - X_est_.ba(1), SC_[6]*a_x_raw + SC_[7]*a_y_raw + SC_[8]*a_z_raw - X_est_.ba(2)}; // linear acceleration estimate

  BLA::Matrix<3,1,float> w_raw = {w_x_raw, w_y_raw, w_z_raw};
  BLA::Matrix<3,1,float> w_est = w_raw - X_est_.bw; // angular velocity estimate

  // Acceleration error compensation
  // See ch. 7 in AHRS algorithms and calibration solutions to facilitate new applications using low-cost MEMS, by Sebastian O. H. Madgwick
  float k_init = 10.0f;
  float k_normal = 0.5f;
  float t_init = 3.0f; // initialization time
  float k = t_curr_ < t_init ? k_normal + (k_init - k_normal)*(t_init-t_curr_)/t_init : k_normal;
  BLA::Matrix<3,3,float> K = {k, 0.0f, 0.0f,
                              0.0f, k, 0.0f,
                              0.0f, 0.0f, k}; // acceleration error gain
  BLA::Matrix<3,1,float> grav_dir; // direction of gravity assumed by X_est_.q
  grav_dir(0) = 2.0f*X_est_.q(0)*X_est_.q(2) - 2.0f*X_est_.q(3)*X_est_.q(1);
  grav_dir(1) = 2.0f*X_est_.q(1)*X_est_.q(2) + 2.0f*X_est_.q(3)*X_est_.q(0);
  grav_dir(2) = 2.0f*X_est_.q(3)*X_est_.q(3) - 1.0f + 2.0f*X_est_.q(2)*X_est_.q(2);
  BLA::Matrix<3,1,float> a_norm = math_utils::vectNorm(a_est);
  BLA::Matrix<3,3,float> a_skew = math_utils::skewSym(a_norm);
  BLA::Matrix<3,1,float> e_a = a_skew * grav_dir;
  w_est = w_est + K*e_a;

  // Construct Omega matrix
  BLA::Matrix<4,4,float> Omega;
  Omega.Submatrix<3,3>(0,0) = -math_utils::skewSym(w_est);
  Omega.Submatrix<1,3>(3,0) = -(~w_est);
  Omega.Submatrix<3,1>(0,3) = w_est;
  Omega(3,3) = 0.0f;

  IMU_proc_.a = a_est;
  IMU_proc_.w = w_est;
  IMU_proc_.Omega = Omega;
}

void ImuEncEKF::propagateImuState(float dt, float dur)
{
  // Simulate forward the IMU state
  ImuEncEKF::state X_pred = ImuEncEKF::rk4(dt, X_est_);
  X_est_ = X_pred;
  //X_est_ = ImuEncEKF::rk4(dt, X_est_);

  // Set current time
  t_curr_ = dur;
}

/*
 * @brief Get estimated state as a single BLA vector
 */
BLA::Matrix<16,1,float> ImuEncEKF::getState()
{
  BLA::Matrix<16,1,float> X_est = {X_est_.q(0),  X_est_.q(1),  X_est_.q(2), X_est_.q(3),
                                   X_est_.bw(0), X_est_.bw(1), X_est_.bw(2),
                                   X_est_.v(0),  X_est_.v(1),  X_est_.v(2),
                                   X_est_.ba(0), X_est_.ba(1), X_est_.ba(2),
                                   X_est_.p(0),  X_est_.p(1),  X_est_.p(2)};
  return X_est;
}

/*
 * @brief Get estimated quaternion
 */
BLA::Matrix<4,1,float> ImuEncEKF::getQuat()
{
  return X_est_.q;
}

/*
 * @brief Get angular velocity bias
 */
BLA::Matrix<3,1,float> ImuEncEKF::getAngVelBias()
{
  return X_est_.bw;
}

/*
 * @brief Get estimated velocity
 */
BLA::Matrix<3,1,float> ImuEncEKF::getVel()
{
  return X_est_.v;
}

/*
 * @brief Get acceleration bias
 */
BLA::Matrix<3,1,float> ImuEncEKF::getAccelBias()
{
  return X_est_.ba;
}

/*
 * @brief Get estimated position
 */
BLA::Matrix<3,1,float> ImuEncEKF::getPos()
{
  return X_est_.p;
}

/*
 * @brief Get estimated error state as a single BLA vector
 */
BLA::Matrix<15,1,float> ImuEncEKF::getErrState()
{
  BLA::Matrix<15,1,float> X_err = {X_err_.th_err(0), X_err_.th_err(1), X_err_.th_err(2),
                                   X_err_.bw_err(0), X_err_.bw_err(1), X_err_.bw_err(2),
                                   X_err_.v_err(0),  X_err_.v_err(1),  X_err_.v_err(2),
                                   X_err_.ba_err(0), X_err_.ba_err(1), X_err_.ba_err(2),
                                   X_err_.p_err(0),  X_err_.p_err(1),  X_err_.p_err(2)};
  return X_err;
}

/*
 * @brief Print the state
 */
void ImuEncEKF::printState()
{
  Serial.println("X_est:");
  Serial.print("  q: "); Serial.print(X_est_.q(0));  Serial.print(" "); Serial.print(X_est_.q(1));  Serial.print(" "); Serial.print(X_est_.q(2)); Serial.print(" "); Serial.println(X_est_.q(3));
  Serial.print(" bw: "); Serial.print(X_est_.bw(0)); Serial.print(" "); Serial.print(X_est_.bw(1)); Serial.print(" "); Serial.println(X_est_.bw(2));
  Serial.print("  v: "); Serial.print(X_est_.v(0));  Serial.print(" "); Serial.print(X_est_.v(1));  Serial.print(" "); Serial.println(X_est_.v(2));
  Serial.print(" ba: "); Serial.print(X_est_.ba(0)); Serial.print(" "); Serial.print(X_est_.ba(1)); Serial.print(" "); Serial.println(X_est_.ba(2));
  Serial.print("  p: "); Serial.print(X_est_.p(0));  Serial.print(" "); Serial.print(X_est_.p(1));  Serial.print(" "); Serial.println(X_est_.p(2));
}

// ----- Private Functions ----- //

/*
 * @brief Differential imu state dynamics
 * @note Input is state
 */
ImuEncEKF::state ImuEncEKF::imuDyn(const ImuEncEKF::state& X_in)
{
  state Xdot;
  Xdot.q = IMU_proc_.Omega * X_in.q * 0.5f;
  Xdot.bw = {0.0f, 0.0f, 0.0f};
  BLA::Matrix<3,1,float> grav = {0.0f, 0.0f, -9.81f};
  Xdot.v = ( ~(math_utils::quat2Rot(X_in.q)) ) * IMU_proc_.a + grav;
  Xdot.v(0) = abs(Xdot.v(0)) > 0.2f ? Xdot.v(0) : 0.0f;
  Xdot.v(1) = abs(Xdot.v(1)) > 0.2f ? Xdot.v(1) : 0.0f;
  Xdot.v(2) = abs(Xdot.v(2)) > 0.2f ? Xdot.v(2) : 0.0f;
  Xdot.ba = {0.0f, 0.0f, 0.0f};
  Xdot.p = X_in.v;

  return Xdot;
}

/*
 * @brief Simulation step using 4-th order Runge-Kutta
 */
ImuEncEKF::state ImuEncEKF::rk4(float dt, const ImuEncEKF::state& X_in)
{
  // RK4
  ImuEncEKF::state k1 = ImuEncEKF::imuDyn(X_in);
  ImuEncEKF::state k2 = ImuEncEKF::imuDyn(X_in + k1 * (dt/2.0f));
  ImuEncEKF::state k3 = ImuEncEKF::imuDyn(X_in + k2 * (dt/2.0f));
  ImuEncEKF::state k4 = ImuEncEKF::imuDyn(X_in + k3 * dt);
  ImuEncEKF::state k = (k1 + k2 * 2.0f + k3 * 2.0f + k4) / 6.0f;

  ImuEncEKF::state X_est;
  X_est = X_in + k * dt;

  // Normalize quaternion
  math_utils::quatNorm(X_est.q);

  return X_est;
}
