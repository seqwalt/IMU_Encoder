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

/*
 * @brief Constructor for ImuEKF
 * @note Initializes state est, error state est
 */
ImuEncEKF::ImuEncEKF()
{
}

/*
 * @brief Set IMU measurements
 */
void ImuEncEKF::setIMUmeas(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z)
{
  IMU_meas_.a(0) = a_x; IMU_meas_.a(1) = a_y; IMU_meas_.a(2) = a_z;
  IMU_meas_.w(0) = w_x; IMU_meas_.w(1) = w_y; IMU_meas_.w(2) = w_z;
}

/*
 * @brief Differential state dynamics
 * @note Input is state
 */
ImuEncEKF::state ImuEncEKF::Dyn(const ImuEncEKF::state& X)
{
  BLA::Matrix<3,1,float> grav = {0.0f, 0.0f, -9.81f};
  BLA::Matrix<3,1,float> a_est = IMU_meas_.a - X.ba; // linear acceleration estimate
  BLA::Matrix<3,1,float> w_est = IMU_meas_.w - X.bw; // angular velocity estimate

  BLA::Matrix<4,4,float> Omega;
  Omega.Submatrix<3,3>(0,0) = -math_utils::skewSym(w_est);
  Omega.Submatrix<1,3>(3,0) = -(~w_est);
  Omega.Submatrix<3,1>(0,3) = w_est;
  Omega(3,3) = 0.0f;

  state Xdot;
  Xdot.q = Omega * X.q * 0.5f;
  Xdot.bw = {0.0f, 0.0f, 0.0f};
  Xdot.v = ( ~(math_utils::quat2Rot(X.q)) ) * a_est + grav;
  Xdot.ba = {0.0f, 0.0f, 0.0f};
  Xdot.p = X.v;

  return Xdot;
}

/*
 * @brief Simulation step using 4-th order Runge-Kutta
 */
void ImuEncEKF::RK4(float dt)
{
  // RK4
  ImuEncEKF::state k1 = ImuEncEKF::Dyn(X_est_);
  ImuEncEKF::state k2 = ImuEncEKF::Dyn(X_est_ + k1 * (dt/2.0f));
  ImuEncEKF::state k3 = ImuEncEKF::Dyn(X_est_ + k2 * (dt/2.0f));
  ImuEncEKF::state k4 = ImuEncEKF::Dyn(X_est_ + k3 * dt);
  ImuEncEKF::state k = (k1 + k2 * 2.0f + k3 * 2.0f + k4) / 6.0f;
  X_est_ = X_est_ + k * dt;

  // Normalize quaternion
  math_utils::quatNorm(X_est_.q);
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
