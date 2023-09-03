#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "ImuEKF.h"

namespace common
{

/*
 * @brief Print state estimate
 */
void printState_(Eigen::Vector4d q, Eigen::Vector3d bw, Eigen::Vector3d v, Eigen::Vector3d ba, Eigen::Vector3d p)
{
  std::cout << "X_est:\n";
  std::cout << "  q: " << q.transpose() << "\n";
  std::cout << " bw: " << bw.transpose() << "\n";
  std::cout << "  v: " << v.transpose() << "\n";
  std::cout << " ba: " << ba.transpose() << "\n";
  std::cout << "  p: " << p.transpose() << "\n";
}

/*
 * @brief Print error state estimate
 */
void printErrState_(Eigen::Vector3d th_err, Eigen::Vector3d bw_err, Eigen::Vector3d v_err, Eigen::Vector3d ba_err, Eigen::Vector3d p_err)
{
  std::cout << "X_err:\n";
  std::cout << " th_err: " << th_err.transpose() << "\n";
  std::cout << " bw_err: " << bw_err.transpose() << "\n";
  std::cout << "  v_err: " << v_err.transpose() << "\n";
  std::cout << " ba_err: " << ba_err.transpose() << "\n";
  std::cout << "  p_err: " << p_err.transpose() << "\n";
}

} // end namespace common
