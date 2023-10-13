# IMU Arduino Library
This library uses a Madgwick complementary filter with RK4 to provide a quaternion estimation given MPU 6050 IMU sensor readings. An example ```.ino``` script implements position control of an inverted pendulum balancing robot with wheel encoders (tested with Arduino Nano).

**Dependency:** [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) is used for matrix math, and can be installed in the Arduino IDE.
