#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "ImuEncEKF.h"

namespace common
{

/*
 * @brief Print state estimate
 */
void printState_(BLA::Matrix<4> q, BLA::Matrix<3> bw, BLA::Matrix<3> v, BLA::Matrix<3> ba, BLA::Matrix<3> p)
{
  // Serial << "X_est:\n";
  // Serial << "  q: " << ~q << "\n";
  // Serial << " bw: " << ~bw << "\n";
  // Serial << "  v: " << ~v << "\n";
  // Serial << " ba: " << ~ba << "\n";
  // Serial << "  p: " << ~p << "\n";
  std::cout << "X_est:\n";
  std::cout << "  q: " << q(0) << ", " << q(1) << ", " << q(2) << ", " << q(3) << "\n";
  std::cout << " bw: " << bw(0) << ", " << bw(1) << ", " << bw(2) << "\n";
  std::cout << "  v: " << v(0) << ", " << v(1) << ", " << v(2) << "\n";
  std::cout << " ba: " << ba(0) << ", " << ba(1) << ", " << ba(2) << "\n";
  std::cout << "  p: " << p(0) << ", " << p(1) << ", " << p(2) << "\n";
}

/*
 * @brief Print error state estimate
 */
void printErrState_(BLA::Matrix<3> th_err, BLA::Matrix<3> bw_err, BLA::Matrix<3> v_err, BLA::Matrix<3> ba_err, BLA::Matrix<3> p_err)
{
  // Serial << "X_err:\n";
  // Serial << " th_err: " << ~th_err << "\n";
  // Serial << " bw_err: " << ~bw_err << "\n";
  // Serial << "  v_err: " << ~v_err << "\n";
  // Serial << " ba_err: " << ~ba_err << "\n";
  // Serial << "  p_err: " << ~p_err << "\n";
  std::cout << "X_err:\n";
  std::cout << " th_err: " << th_err(0) << ", " << th_err(1) << ", " << th_err(2) << "\n";
  std::cout << " bw_err: " << bw_err(0) << ", " << bw_err(1) << ", " << bw_err(2) << "\n";
  std::cout << "  v_err: " << v_err(0) << ", " << v_err(1) << ", " << v_err(2) << "\n";
  std::cout << " ba_err: " << ba_err(0) << ", " << ba_err(1) << ", " << ba_err(2) << "\n";
  std::cout << "  p_err: " << p_err(0) << ", " << p_err(1) << ", " << p_err(2) << "\n";
}

void printMatrix_(BLA::Matrix<3,3> M)
{
  std::cout << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << "\n";
  std::cout << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << "\n";
  std::cout << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << "\n";
}

void printMatrix_(BLA::Matrix<4,4> M)
{
  std::cout << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << ", " << M(0,3) << "\n";
  std::cout << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << ", " << M(1,3) << "\n";
  std::cout << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << ", " << M(2,3) << "\n";
  std::cout << M(3,0) << ", " << M(3,1) << ", " << M(3,2) << ", " << M(3,3) << "\n";
}

void printMatrix_(BLA::Matrix<3> v)
{
  std::cout << v(0) << ", " << v(1) << ", " << v(2) << "\n";
}

void printMatrix_(BLA::Matrix<4> q)
{
  std::cout << q(0) << ", " << q(1) << ", " << q(2) << ", " << q(3) << "\n";
}

} // end namespace common
