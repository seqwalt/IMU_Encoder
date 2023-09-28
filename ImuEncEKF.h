/*
  ImuEncEKF.h - Library to apply an EKF to an IMU (accel. and gyro.) and wheel encoder
  Author - Sequoyah Walters
  Note - Uses notation from "Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight"
*/

#pragma once

#include <BasicLinearAlgebra.h>

// library interface description
class ImuEncEKF
{
  public:
    // Constructor
    ImuEncEKF();

    // Functions
    void applyImuBias(float, float, float, float, float, float);
    void processImuMeas(float, float, float, float, float, float, const float*);
    void propagateImuState(float, float);
    BLA::Matrix<16,1,float> getState();
    BLA::Matrix<4,1,float> getQuat();
    BLA::Matrix<3,1,float> getAngVelBias();
    BLA::Matrix<3,1,float> getVel();
    BLA::Matrix<3,1,float> getAccelBias();
    BLA::Matrix<3,1,float> getPos();
    BLA::Matrix<15,1,float> getErrState();
    void printState();

  private:
    // IMU measurement struct
    struct meas {
      BLA::Matrix<3,1,float> a; // linear acceleration
      BLA::Matrix<3,1,float> w; // angular velocity
      BLA::Matrix<4,4,float> Omega; // Omega matrix from "Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight"
    };

    // State struct
    struct state {
      BLA::Matrix<4,1,float> q;  // rotation from world frame to IMU body frame, q = [qx, qy, qz, qw]
      BLA::Matrix<3,1,float> bw; // bias of angular velocity measurements
      BLA::Matrix<3,1,float> v;  // velocity of IMU body frame in world frame
      BLA::Matrix<3,1,float> ba; // bias of linear acceleration measurements
      BLA::Matrix<3,1,float> p;  // position of IMU body frame in world frame
      // Constructor
      state(BLA::Matrix<4,1,float> q={0.0f, 0.0f, 0.0f, 1.0f},
            BLA::Matrix<3,1,float> bw={0.0f, 0.0f, 0.0f},
            BLA::Matrix<3,1,float> v={0.0f, 0.0f, 0.0f},
            BLA::Matrix<3,1,float> ba={0.0f, 0.0f, 0.0f},
            BLA::Matrix<3,1,float> p={0.0f, 0.0f, 0.0f})
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
      BLA::Matrix<3,1,float> th_err; // small angle rotation error q_err ~= (0.5*th_err, 1)^T
      BLA::Matrix<3,1,float> bw_err; // bw_err = bw_true - bw_estimate
      BLA::Matrix<3,1,float> v_err;
      BLA::Matrix<3,1,float> ba_err;
      BLA::Matrix<3,1,float> p_err;
      // Constructor
      err_state(BLA::Matrix<3,1,float> th_err={0.0f, 0.0f, 0.0f},
                BLA::Matrix<3,1,float> bw_err={0.0f, 0.0f, 0.0f},
                BLA::Matrix<3,1,float> v_err={0.0f, 0.0f, 0.0f},
                BLA::Matrix<3,1,float> ba_err={0.0f, 0.0f, 0.0f},
                BLA::Matrix<3,1,float> p_err={0.0f, 0.0f, 0.0f})
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
    meas IMU_proc_;   // processed IMU measurement
    state X_est_;     // state estimate
    err_state X_err_; // error state estimate
    float t_curr_; // time since start of filter

    //BLA::Matrix<15,15,float> P_k; // error state covariance

    // Functions
    state imuDyn(const ImuEncEKF::state&); // imu state dynamics
    state rk4(float, const ImuEncEKF::state&); // 4-th order Runge-Kutta
};
