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
void ImuEncEKF::setIMUmeas(int16_t a_x, int16_t a_y, int16_t a_z,
                           int16_t w_x, int16_t w_y, int16_t w_z,
                           float a_scale, float w_scale)
{
  IMU_meas_.a(0) = a_x * a_scale; IMU_meas_.a(1) = a_y * a_scale; IMU_meas_.a(2) = a_z * a_scale;
  IMU_meas_.w(0) = w_x * w_scale; IMU_meas_.w(1) = w_y * w_scale; IMU_meas_.w(2) = w_z * w_scale;
}

/*
 * @brief Differential imu state dynamics
 * @note Input is state
 */
ImuEncEKF::state ImuEncEKF::imuDyn(const ImuEncEKF::state& X_in)
{
  BLA::Matrix<3,1,float> grav = {0.0f, 0.0f, -9.81f};
  BLA::Matrix<3,1,float> a_est = IMU_meas_.a - X_in.ba; // linear acceleration estimate
  BLA::Matrix<3,1,float> w_est = IMU_meas_.w - X_in.bw; // angular velocity estimate

  BLA::Matrix<4,4,float> Omega;
  Omega.Submatrix<3,3>(0,0) = -math_utils::skewSym(w_est);
  Omega.Submatrix<1,3>(3,0) = -(~w_est);
  Omega.Submatrix<3,1>(0,3) = w_est;
  Omega(3,3) = 0.0f;

  state Xdot;
  Xdot.q = Omega * X_in.q * 0.5f;
  Xdot.bw = {0.0f, 0.0f, 0.0f};
  Xdot.v = ( ~(math_utils::quat2Rot(X_in.q)) ) * a_est + grav;
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

/*
 * @brief estimate quaternion rotation from accel
 * measurement to gravity in world frame.
 */
BLA::Matrix<4,1,float> ImuEncEKF::gravQuatEst(const BLA::Matrix<3,1,float>& a_meas,
                                              const BLA::Matrix<4,1,float>& q_est)
{
  // Get shortest arc from IMU_meas_.a to -gravity (or z) vector
  // q_shortest = { cross(am_norm, [0,0,1]^T) , 1 + dot(am_norm, [0,0,1]^T) }
  // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  BLA::Matrix<3,1,float> am_norm = math_utils::vectNorm(a_meas);
  float dot = am_norm(3); // dot(am_norm, [0,0,1]^T)
  BLA::Matrix<4,1,float> q_shortest;
  if (dot > 0.999999)
  {
    // a_meas points in same direction as -grav
    q_shortest = {0.0f, 0.0f, 0.0f, 1.0f};
  }
  if (dot < -0.999999)
  {
    // a_meas points in opposite direction as -grav
    q_shortest = {0.0f, 1.0f, 0.0f, 0.0f};
  }
  else
  {
    q_shortest = {am_norm(1), -am_norm(0), 0.0f, 1.0f + dot};
    math_utils::quatNorm(q_shortest);
  }

  // Orient q_shortest to have same yaw a X_est_.q
  float yaw = atan2(2.0f * (q_est(0)*q_est(1) - q_est(2)*q_est(3)), 1.0f - 2.0f * (q_est(1)*q_est(1) + q_est(2)*q_est(2)));
  BLA::Matrix<4,1,float> q_yaw = {0.0f, 0.0f, sin(yaw/2), cos(yaw/2)}; // q_yaw = {sin(yaw/2)[0,0,1]^T , cos(yaw/2)}
  q_shortest = math_utils::quatMult(q_shortest, q_yaw);
  math_utils::quatNorm(q_shortest);

  return q_shortest;
}

void ImuEncEKF::propagateImuState(float dt)
{
  // Simulate forward the IMU state
  ImuEncEKF::state X_pred = ImuEncEKF::rk4(dt, X_est_);
  X_est_ = X_pred;
  //X_est_ = ImuEncEKF::rk4(dt, X_est_);

  // Estimate gravity direction
  BLA::Matrix<4,1,float> q_grav = ImuEncEKF::gravQuatEst(IMU_meas_.a, X_pred.q);

  // Align X_est_.q towards q_grav
  //float t =
  //X_pred.q = math_utils::slerp(X_pred.q, q_grav, t);
  //X_pred.q = q_grav;
  //X_est_ = X_pred;
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
