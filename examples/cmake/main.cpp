#include "../../ImuEncEKF.h"
#include "../../math_utils.h"
#include "../../common.h"
#include <BasicLinearAlgebra.h>
#include <iostream>

int main(){
    // ImuEKF filter;
    // filter.setIMUmeas(1.0, -3.0, 9.81, 0.0, 0.0, 0.0);
    // filter.printState();
    // filter.RK4(0.05); // run RK4 with 0.05 second time step
    // filter.printState();

    // Test math_utils.h
    BLA::Matrix<3> w = {1.0, 2.0, 3.0};
    BLA::Matrix<3,3> W = math_utils::skewSym(w);
    common::printMatrix_(W);
    std::cout << "\n";

    BLA::Matrix<4> q0 = {1.0, 2.0, 3.0, 4.0};
    math_utils::quatNorm(q0);
    common::printMatrix_(q0);
    std::cout << (~q0*q0)(0) << "\n\n"; // squared norm (should be 1)

    BLA::Matrix<4> q1 = {0.5, 0.5, 0.5, 0.5};
    BLA::Matrix<4> q2 = {0.5, 0.5, 0.5, -0.5};
    common::printMatrix_(math_utils::quatMult(q1, q2));
    std::cout << "\n";

    BLA::Matrix<3> dtheta = {0.707, 0, 0.707};
    common::printMatrix_(math_utils::smallAngleQuat(dtheta));
    std::cout << "\n";

    common::printMatrix_(math_utils::rot2Quat(math_utils::quat2Rot(q1)));
}
