#pragma once

#include <BasicLinearAlgebra.h>
#include <iostream>

namespace print
{

void printState(BLA::Matrix<16> X_est)
{
  std::cout << "X_est:\n";
  std::cout << "  q: " << X_est(0) << ", " << X_est(1) << ", " << X_est(2) << ", " << X_est(3) << "\n";
  std::cout << " bw: " << X_est(4) << ", " << X_est(5) << ", " << X_est(6) << "\n";
  std::cout << "  v: " << X_est(7) << ", " << X_est(8) << ", " << X_est(9) << "\n";
  std::cout << " ba: " << X_est(10) << ", " << X_est(11) << ", " << X_est(12) << "\n";
  std::cout << "  p: " << X_est(13) << ", " << X_est(14) << ", " << X_est(15) << "\n";
}

void printErrState(BLA::Matrix<15> X_err)
{
  std::cout << "X_err:\n";
  std::cout << " th_err: " << X_err(0) << ", " << X_err(1) << ", " << X_err(2) << "\n";
  std::cout << " bw_err: " << X_err(3) << ", " << X_err(4) << ", " << X_err(5) << "\n";
  std::cout << "  v_err: " << X_err(6) << ", " << X_err(7) << ", " << X_err(8) << "\n";
  std::cout << " ba_err: " << X_err(9) << ", " << X_err(10) << ", " << X_err(11) << "\n";
  std::cout << "  p_err: " << X_err(12) << ", " << X_err(13) << ", " << X_err(14) << "\n";
}

void printMatrix(BLA::Matrix<3,3> M)
{
  std::cout << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << "\n";
  std::cout << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << "\n";
  std::cout << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << "\n";
}

void printMatrix(BLA::Matrix<4,4> M)
{
  std::cout << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << ", " << M(0,3) << "\n";
  std::cout << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << ", " << M(1,3) << "\n";
  std::cout << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << ", " << M(2,3) << "\n";
  std::cout << M(3,0) << ", " << M(3,1) << ", " << M(3,2) << ", " << M(3,3) << "\n";
}

void printMatrix(BLA::Matrix<3> v)
{
  std::cout << v(0) << ", " << v(1) << ", " << v(2) << "\n";
}

void printMatrix(BLA::Matrix<4> q)
{
  std::cout << q(0) << ", " << q(1) << ", " << q(2) << ", " << q(3) << "\n";
}

} // end namespace common
