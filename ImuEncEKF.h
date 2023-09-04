/*
  ImuEncEKF.h - Library to apply an EKF to an IMU (accel. and gyro.) and wheel encoder
  Author - Sequoyah Walters
  Note - Uses notation from "Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight"
*/

#pragma once

#include <string>
#include <BasicLinearAlgebra.h>

// library interface description
class ImuEncEKF
{
  public:
    // Constructor
    ImuEncEKF();

    // Functions
    void printState();
    void printErrState();
    void setIMUmeas(float, float, float, float, float, float);
    void RK4(float);

  private:
    // IMU measurement struct
    struct meas {
      BLA::Matrix<3> a; // linear acceleration
      BLA::Matrix<3> w; // angular velocity
    };

    // State struct
    struct state {
      BLA::Matrix<4> q;  // rotation from world frame to IMU body frame
      BLA::Matrix<3> bw; // bias of angular velocity measurements
      BLA::Matrix<3> v;  // velocity of IMU body frame in world frame
      BLA::Matrix<3> ba; // bias of linear acceleration measurements
      BLA::Matrix<3> p;  // position of IMU body frame in world frame
      // Constructor
      state(BLA::Matrix<4> q={0.0, 0.0, 0.0, 1.0},
            BLA::Matrix<3> bw={0.0, 0.0, 0.0},
            BLA::Matrix<3> v={0.0, 0.0, 0.0},
            BLA::Matrix<3> ba={0.0, 0.0, 0.0},
            BLA::Matrix<3> p={0.0, 0.0, 0.0})
        : q(q), bw(bw), v(v), ba(ba), p(p)
      {
      }
      // add operator (doesn't modify object, therefor const)
      state operator+(const state& A) const
      {
        return state(q+A.q, bw+A.bw, v+A.v, ba+A.ba, p+A.p);
      }
      // multiply operator (doesn't modify object, therefor const)
      state operator*(float val) const
      {
        return state(q*val, bw*val, v*val, ba*val, p*val);
      }
      // divide operator (doesn't modify object, therefor const)
      state operator/(float val) const
      {
        return state(q/val, bw/val, v/val, ba/val, p/val);
      }
    };

    // Error state struct
    struct err_state {
      BLA::Matrix<3> th_err; // small angle rotation error q_err ~= (0.5*th_err, 1)^T
      BLA::Matrix<3> bw_err; // bw_err = bw_true - bw_estimate
      BLA::Matrix<3> v_err;
      BLA::Matrix<3> ba_err;
      BLA::Matrix<3> p_err;
      // Constructor
      err_state(BLA::Matrix<3> th_err={0.0, 0.0, 0.0},
                BLA::Matrix<3> bw_err={0.0, 0.0, 0.0},
                BLA::Matrix<3> v_err={0.0, 0.0, 0.0},
                BLA::Matrix<3> ba_err={0.0, 0.0, 0.0},
                BLA::Matrix<3> p_err={0.0, 0.0, 0.0})
        : th_err(th_err), bw_err(bw_err), v_err(v_err), ba_err(ba_err), p_err(p_err)
      {
      }
      // add operator (doesn't modify object, therefor const)
      err_state operator+(const err_state& A) const
      {
        return err_state(th_err+A.th_err, bw_err+A.bw_err, v_err+A.v_err, ba_err+A.ba_err, p_err+A.p_err);
      }
      // multiply operator (doesn't modify object, therefor const)
      err_state operator*(float val) const
      {
        return err_state(th_err*val, bw_err*val, v_err*val, ba_err*val, p_err*val);
      }
      // divide operator (doesn't modify object, therefor const)
      err_state operator/(float val) const
      {
        return err_state(th_err/val, bw_err/val, v_err/val, ba_err/val, p_err/val);
      }
    };

    // Class variables
    meas IMU_meas_;   // IMU measurement
    state X_est_;     // state estimate
    err_state X_err_; // error state estimate
    BLA::Matrix<15, 15> P_k; // error state covariance

    // Functions
    state Dyn(const ImuEncEKF::state&); // state dynamics
};
