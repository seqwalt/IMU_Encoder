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
#include "common.h"

#include <vector>

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
  IMU_meas_.a = {a_x, a_y, a_z};
  IMU_meas_.w = {w_x, w_y, w_z};
}

/*
 * @brief Differential state dynamics
 * @note Input is state
 */
ImuEncEKF::state ImuEncEKF::Dyn(const ImuEncEKF::state& X)
{
  BLA::Matrix<3> grav = {0.0, 0.0, -9.81};
  BLA::Matrix<3> a_est = IMU_meas_.a - X.ba; // linear acceleration estimate
  BLA::Matrix<3> w_est = IMU_meas_.w - X.bw; // angular velocity estimate

  BLA::Matrix<4,4> Omega(4,4);
  Omega.Submatrix<3,3>(0,0) = -math_utils::skewSym(w_est);
  Omega.Submatrix<1,3>(3,0) = -(~w_est);
  Omega.Submatrix<3,1>(0,3) = w_est;
  Omega(3,3) = 0.0;

  state Xdot;
  Xdot.q = Omega * X.q * 0.5f;
  Xdot.bw = {0.0, 0.0, 0.0};
  Xdot.v = ( ~(math_utils::quat2Rot(X.q)) ) * a_est + grav;
  Xdot.ba = {0.0, 0.0, 0.0};
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
  ImuEncEKF::state k2 = ImuEncEKF::Dyn(X_est_ + k1 * (dt/2.0));
  ImuEncEKF::state k3 = ImuEncEKF::Dyn(X_est_ + k2 * (dt/2.0));
  ImuEncEKF::state k4 = ImuEncEKF::Dyn(X_est_ + k3 * dt);
  ImuEncEKF::state k = (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0;
  X_est_ = X_est_ + k * dt;

  // Normalize quaternion
  math_utils::quatNorm(X_est_.q);
}

/*
 * @brief Print state estimate
 */
void ImuEncEKF::printState()
{
  common::printState_(X_est_.q, X_est_.bw, X_est_.v, X_est_.ba, X_est_.p);
}

/*
 * @brief Print error state estimate
 */
void ImuEncEKF::printErrState()
{
  common::printErrState_(X_err_.th_err, X_err_.bw_err, X_err_.v_err, X_err_.ba_err, X_err_.p_err);
}
