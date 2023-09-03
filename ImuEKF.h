/*
  ImuEKF.cpp - Library to apply an EKF to an IMU (accel. and gyro.) - description
  Author - Sequoyah Walters
  Note - Uses notation from "Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight"
*/

#pragma once

#include <string>
#include <Eigen/Dense>

// library interface description
class ImuEKF
{
  public:
    // Constructor
    ImuEKF();

    // Functions
    void printState();
    void printErrState();
    void setIMUmeas(double, double, double, double, double, double);
    void RK4(double);

  private:
    // IMU measurement struct
    struct meas {
      Eigen::Vector3d a; // linear acceleration
      Eigen::Vector3d w; // angular velocity
    };

    // State struct
    struct state {
      Eigen::Vector4d q;  // rotation from world frame to IMU body frame
      Eigen::Vector3d bw; // bias of angular velocity measurements
      Eigen::Vector3d v;  // velocity of IMU body frame in world frame
      Eigen::Vector3d ba; // bias of linear acceleration measurements
      Eigen::Vector3d p;  // position of IMU body frame in world frame
      // Constructor
      state(Eigen::Vector4d q={0.0, 0.0, 0.0, 1.0},
            Eigen::Vector3d bw={0.0, 0.0, 0.0},
            Eigen::Vector3d v={0.0, 0.0, 0.0},
            Eigen::Vector3d ba={0.0, 0.0, 0.0},
            Eigen::Vector3d p={0.0, 0.0, 0.0})
        : q(q), bw(bw), v(v), ba(ba), p(p)
      {
      }
      // add operator (doesn't modify object, therefor const)
      state operator+(const state& A) const
      {
        return state(q+A.q, bw+A.bw, v+A.v, ba+A.ba, p+A.p);
      }
      // multiply operator (doesn't modify object, therefor const)
      state operator*(double val) const
      {
        return state(q*val, bw*val, v*val, ba*val, p*val);
      }
      // divide operator (doesn't modify object, therefor const)
      state operator/(double val) const
      {
        return state(q/val, bw/val, v/val, ba/val, p/val);
      }
    };

    // Error state struct
    struct err_state {
      Eigen::Vector3d th_err; // small angle rotation error q_err ~= (0.5*th_err, 1)^T
      Eigen::Vector3d bw_err; // bw_err = bw_true - bw_estimate
      Eigen::Vector3d v_err;
      Eigen::Vector3d ba_err;
      Eigen::Vector3d p_err;
      // Constructor
      err_state(Eigen::Vector3d th_err={0.0, 0.0, 0.0},
                Eigen::Vector3d bw_err={0.0, 0.0, 0.0},
                Eigen::Vector3d v_err={0.0, 0.0, 0.0},
                Eigen::Vector3d ba_err={0.0, 0.0, 0.0},
                Eigen::Vector3d p_err={0.0, 0.0, 0.0})
        : th_err(th_err), bw_err(bw_err), v_err(v_err), ba_err(ba_err), p_err(p_err)
      {
      }
      // add operator (doesn't modify object, therefor const)
      err_state operator+(const err_state& A) const
      {
        return err_state(th_err+A.th_err, bw_err+A.bw_err, v_err+A.v_err, ba_err+A.ba_err, p_err+A.p_err);
      }
      // multiply operator (doesn't modify object, therefor const)
      err_state operator*(double val) const
      {
        return err_state(th_err*val, bw_err*val, v_err*val, ba_err*val, p_err*val);
      }
      // divide operator (doesn't modify object, therefor const)
      err_state operator/(double val) const
      {
        return err_state(th_err/val, bw_err/val, v_err/val, ba_err/val, p_err/val);
      }
    };

    // Class variables
    meas IMU_meas_;   // IMU measurement
    state X_est_;     // state estimate
    err_state X_err_; // error state estimate
    Eigen::Matrix<double, 15, 15> P_k; // error state covariance

    // Functions
    state Dyn(const ImuEKF::state&); // state dynamics
};
