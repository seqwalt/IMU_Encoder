# IMU Arduino Library
This library uses RK4 and a Madgwick correction to provide a quaternion estimation given MPU 6050 sensor readings. An example ```.ino``` script implements position control of an inverted pendulum balancing robot with wheel encoders (tested with Arduino Nano).

**Dependency:** [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) is used for matrix math, and can be installed in the Arduino IDE.
