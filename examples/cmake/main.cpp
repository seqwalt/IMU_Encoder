#include "../../ImuEKF.h"

int main(){
    ImuEKF filter;
    filter.setIMUmeas(1.0, -3.0, 9.81, 0.0, 0.0, 0.0);
    filter.printState();
    filter.RK4(0.05); // run RK4 with 0.05 second time step
    filter.printState();
}
