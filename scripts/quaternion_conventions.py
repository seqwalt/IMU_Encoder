#!/usr/bin/env python3

import sympy as sp
import numpy as np

# Quaternion class
class Quaternion:
    def __init__(self, q0, q1, q2, q3, convention):
        self.__convention = convention; # Hamilton or JPL
        if self.__convention == "Hamilton":
            self.__w = q0; # real part
            self.__x = q1;
            self.__y = q2;
            self.__z = q3;
        elif self.__convention == "JPL":
            self.__x = q0;
            self.__y = q1;
            self.__z = q2;
            self.__w = q3; # real part
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')

    # Quaternion addition
    def add(self, b):
        a = self
        if a.__convention != b.__convention:
            raise NameError('conventions must match')
        if a.__convention == "Hamilton":
            return Quaternion(a.__w + b.__w, a.__x + b.__x, a.__y + b.__y, a.__z + b.__z, a.__convention)
        elif a.__convention == "JPL":
            return Quaternion(a.__x + b.__x, a.__y + b.__y, a.__z + b.__z, a.__w + b.__w, a.__convention)
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')

    # Quaternion multiplication
    #
    # a = (aw + ax*i + ay*j + az*k) and b = (bw + bx*i + by*j + bz*k)
    # a * b = (aw + ax*i + ay*j + az*k) * (bw + bx*i + by*j + bz*k)
    #       = aw*bw*1*1 + aw*bx*1*i + aw*by*1*j + aw*bz*1*k +
    #         ax*bw*i*1 + ax*bx*i*i + ax*by*i*j + ax*bz*i*k +
    #         ay*bw*j*1 + ay*bx*j*i + ay*by*j*j + ay*bz*j*k +
    #         az*bw*k*1 + az*bx*k*i + az*by*k*j + az*bz*k*k
    #
    # Hamilton convention:
    #   i*i = j*j = k*k = -1
    #   i*j = -j*i = k
    #   j*k = -k*j = i
    #   k*i = -i*k = j
    #
    #   Then,
    #   a * b = aw*bw + aw*bx*i + aw*by*j + aw*bz*k +
    #           ax*bw*i - ax*bx + ax*by*k - ax*bz*j +
    #           ay*bw*j - ay*bx*k - ay*by + ay*bz*i +
    #           az*bw*k + az*bx*j - az*by*i - az*bz
    #         = aw*bw - ax*bx - ay*by - az*bz +
    #          (aw*bx + ax*bw + ay*bz - az*by)*i +
    #          (aw*by - ax*bz + ay*bw + az*bx)*j +
    #          (aw*bz + ax*by - ay*bx + az*bw)*k
    #
    # JPL convention:
    #   i*i = j*j = k*k = -1
    #   -i*j = j*i = k
    #   -j*k = k*j = i
    #   -k*i = i*k = j
    #
    #   Then,
    #   a * b = aw*bw + aw*bx*i + aw*by*j + aw*bz*k +
    #           ax*bw*i - ax*bx - ax*by*k + ax*bz*j +
    #           ay*bw*j + ay*bx*k - ay*by - ay*bz*i +
    #           az*bw*k - az*bx*j + az*by*i - az*bz
    #         =(aw*bx + ax*bw - ay*bz + az*by)*i +
    #          (aw*by + ax*bz + ay*bw - az*bx)*j +
    #          (aw*bz - ax*by + ay*bx + az*bw)*k +
    #           aw*bw - ax*bx - ay*by - az*bz
    #
    def mult(self, b):
        a = self
        if a.__convention != b.__convention:
            raise NameError('conventions must match')
        if a.__convention == "Hamilton":
            real = a.__w*b.__w - a.__x*b.__x - a.__y*b.__y - a.__z*b.__z
            i = a.__w*b.__x + a.__x*b.__w + a.__y*b.__z - a.__z*b.__y
            j = a.__w*b.__y - a.__x*b.__z + a.__y*b.__w + a.__z*b.__x
            k = a.__w*b.__z + a.__x*b.__y - a.__y*b.__x + a.__z*b.__w
            return Quaternion(real, i, j, k, a.__convention)
        elif a.__convention == "JPL":
            i = a.__w*b.__x + a.__x*b.__w - a.__y*b.__z + a.__z*b.__y
            j = a.__w*b.__y + a.__x*b.__z + a.__y*b.__w - a.__z*b.__x
            k = a.__w*b.__z - a.__x*b.__y + a.__y*b.__x + a.__z*b.__w
            real = a.__w*b.__w - a.__x*b.__x - a.__y*b.__y - a.__z*b.__z
            return Quaternion(i, j, k, real, a.__convention)
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')

    # Quaternion conjugate
    def conj(self):
        if self.__convention == "Hamilton":
            return Quaternion(self.__w, -self.__x, -self.__y, -self.__z, self.__convention)
        elif self.__convention == "JPL":
            return Quaternion(-self.__x, -self.__y, -self.__z, self.__w, self.__convention)
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')

    # Rotate a 3-vector
    def rotate(self, v):
        if self.__convention == "Hamilton":
            qv = Quaternion(0, v.x, v.y, v.z, self.__convention)
        elif self.__convention == "JPL":
            qv = Quaternion(v.x, v.y, v.z, 0, self.__convention)
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')
        q_temp = self.mult(qv)
        qv_rot = q_temp.mult(self.conj())
        return Vector3(qv_rot.__x, qv_rot.__y, qv_rot.__z)

    # Print the quaternion
    def print(self):
        if self.__convention == "Hamilton":
            print("Convention: " + self.__convention)
            print("[qw, qx, qy, qz]: " + str(self.__w) + ", " + str(self.__x) + ", " + str(self.__y) + ", " + str(self.__z))
        elif self.__convention == "JPL":
            print("Convention: " + self.__convention)
            print("[qx, qy, qz, qw]: " + str(self.__x) + ", " + str(self.__y) + ", " + str(self.__z) + ", " + str(self.__w))
        else:
            raise NameError('convention must be "Hamilton" or "JPL"')

# 3D vector class
class Vector3:
    def __init__(self, x, y, z):
        self.x = x;
        self.y = y;
        self.z = z;

    # Print the vector
    def print(self):
        sp.pprint(sp.Matrix([self.x, self.y, self.z]))
#---------------------------------------------------------------

qw, qx, qy, qz = sp.symbols('qw qx qy qz')
#q = Quaternion(qw, qx, qy, qz, "Hamilton")
q = Quaternion(qx, qy, qz, qw, "JPL")

# multiplication
ang = np.pi/2
qb = Quaternion(0.0, np.sin(ang/2), 0.0, np.cos(ang/2), "JPL")
qc = q.mult(qb)
qc.print()

# Gravity direction in body frame
grav = Vector3(0, 0, 1)
grav_rot = q.rotate(grav)
grav_rot.print()

# Print rotation matrix (v_rot = R * v)
vx, vy, vz = sp.symbols('vx vy vz')
v = Vector3(vx, vy, vz)
v_rot = q.rotate(v)
R = sp.zeros(3,3)
R[0,0] = sp.collect(sp.expand(v_rot.x), vx).coeff(vx,1)
R[0,1] = sp.collect(sp.expand(v_rot.x), vy).coeff(vy,1)
R[0,2] = sp.collect(sp.expand(v_rot.x), vz).coeff(vz,1)
R[1,0] = sp.collect(sp.expand(v_rot.y), vx).coeff(vx,1)
R[1,1] = sp.collect(sp.expand(v_rot.y), vy).coeff(vy,1)
R[1,2] = sp.collect(sp.expand(v_rot.y), vz).coeff(vz,1)
R[2,0] = sp.collect(sp.expand(v_rot.z), vx).coeff(vx,1)
R[2,1] = sp.collect(sp.expand(v_rot.z), vy).coeff(vy,1)
R[2,2] = sp.collect(sp.expand(v_rot.z), vz).coeff(vz,1)
sp.pprint(R)
