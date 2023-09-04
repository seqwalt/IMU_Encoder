#include <BasicLinearAlgebra.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#include "common.h"
#include "math_utils.h"
#include "ImuEncEKF.h"

void setup() {
  // put your setup code here, to run once:
  ImuEKF filter;
}

void loop() {
  // put your main code here, to run repeatedly:
  filter.setIMUmeas(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
}
