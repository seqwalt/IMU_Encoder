#include "../../ImuEncEKF.h"
#include "../../math_utils.h"
#include "print.h"
#include <BasicLinearAlgebra.h>
#include <iostream>

int main(){
    ImuEncEKF filter;
    filter.setIMUmeas(1.0, -3.0, 9.81, 0.0, 0.0, 0.0);
    print::printState(filter.getState());
    filter.RK4(0.05); // run RK4 with 0.05 second time step
    print::printState(filter.getState());

    // Test math_utils.h
    // BLA::Matrix<3> w = {1.0, 2.0, 3.0};
    // BLA::Matrix<3,3> W = math_utils::skewSym(w);
    // print::printMatrix(W);
    // std::cout << "\n";
    //
    // BLA::Matrix<4> q0 = {1.0, 2.0, 3.0, 4.0};
    // math_utils::quatNorm(q0);
    // print::printMatrix(q0);
    // std::cout << (~q0*q0)(0) << "\n\n"; // squared norm (should be 1)
    //
    // BLA::Matrix<4> q1 = {0.5, 0.5, 0.5, 0.5};
    // BLA::Matrix<4> q2 = {0.5, 0.5, 0.5, -0.5};
    // print::printMatrix(math_utils::quatMult(q1, q2));
    // std::cout << "\n";
    //
    // BLA::Matrix<3> dtheta = {0.707, 0, 0.707};
    // print::printMatrix(math_utils::smallAngleQuat(dtheta));
    // std::cout << "\n";
    //
    // print::printMatrix(math_utils::rot2Quat(math_utils::quat2Rot(q1)));
}
