/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 *
 * Edits by SW:
 *   08-23-2023:
 *     Change namespace name to math_utils
 *     Use #pragma once
 *     shorten all function names
 *
 *   09-03-23
 *     Use BasicLinearAlgebra instead of Eigen
 *
 * Function list (all input/output float types):
 *   skewSym, vectNorm, quatNorm, quatMult,
 *   smallAngleQuat, quat2Rot, rot2Quat, slerp
 *
 * NOTE: JPL quaternion convention used
 */

#pragma once

#include <BasicLinearAlgebra.h>
#include <math.h>

namespace math_utils
{

/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
inline BLA::Matrix<3,3,float> skewSym ( const BLA::Matrix<3,1,float>& w )
{
    BLA::Matrix<3,3,float> w_hat;
    w_hat ( 0, 0 ) = 0.0f;
    w_hat ( 0, 1 ) = -w ( 2 );
    w_hat ( 0, 2 ) = w ( 1 );
    w_hat ( 1, 0 ) = w ( 2 );
    w_hat ( 1, 1 ) = 0.0f;
    w_hat ( 1, 2 ) = -w ( 0 );
    w_hat ( 2, 0 ) = -w ( 1 );
    w_hat ( 2, 1 ) = w ( 0 );
    w_hat ( 2, 2 ) = 0.0f;
    return w_hat;
}

/*
 * @brief Normalize a 3 vector.
 */
inline BLA::Matrix<3,1,float> vectNorm (
    const BLA::Matrix<3,1,float>& v )
{
    BLA::Matrix<3,1,float> v_norm;
    v_norm = v / (float)sqrt( v(0)*v(0) + v(1)*v(1) + v(2)*v(2) );
    return v_norm;
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline void quatNorm ( BLA::Matrix<4,1,float>& q )
{
    q = q / (float)sqrt( q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3) );
}

/*
 * @brief Perform q1 * q2
 */
inline BLA::Matrix<4,1,float> quatMult (
    const BLA::Matrix<4,1,float>& q1,
    const BLA::Matrix<4,1,float>& q2 )
{
    /*
     * L =  qw,  qz, -qy,  qx
     *     -qz,  qw,  qx,  qy
     *      qy, -qx,  qw,  qz
     *     -qx, -qy, -qz,  qw
     */
    BLA::Matrix<4,4,float> L;
    L(0,0) =  q1(3);
    L(0,1) =  q1(2);
    L(0,2) = -q1(1);
    L(0,3) =  q1(0);
    L(1,0) = -q1(2);
    L(1,1) =  q1(3);
    L(1,2) =  q1(0);
    L(1,3) =  q1(1);
    L(2,0) =  q1(1);
    L(2,1) = -q1(0);
    L(2,2) =  q1(3);
    L(2,3) =  q1(2);
    L(3,0) = -q1(0);
    L(3,1) = -q1(1);
    L(3,2) = -q1(2);
    L(3,3) =  q1(3);

    // q_prod = L * q2
    BLA::Matrix<4,1,float> q_prod;
    q_prod(0) = L(0,0)*q2(0) + L(0,1)*q2(1) + L(0,2)*q2(2) + L(0,3)*q2(3);
    q_prod(1) = L(1,0)*q2(0) + L(1,1)*q2(1) + L(1,2)*q2(2) + L(1,3)*q2(3);
    q_prod(2) = L(2,0)*q2(0) + L(2,1)*q2(1) + L(2,2)*q2(2) + L(2,3)*q2(3);
    q_prod(3) = L(3,0)*q2(0) + L(3,1)*q2(1) + L(3,2)*q2(2) + L(3,3)*q2(3);

    // TODO why norm not work?
    //quatNorm(q_prod);

    //float num;
    //num = q_prod(0)*q_prod(0) + q_prod(1)*q_prod(1) + q_prod(2)*q_prod(2) + q_prod(3)*q_prod(3);
    //Serial.print(q_prod(3)*q_prod(3)); // qx
    // Serial.print(",");
    // Serial.print(q_prod(1)*q_prod(1)); // qy
    // Serial.print(",");
    // Serial.print(q_prod(2)*q_prod(2)); // qz
    // Serial.print(",");
    // Serial.print(q_prod(3)*q_prod(3)); // qw

    // Serial.print(q_prod(0), 1); // qx
    // Serial.print(",");
    // Serial.print(q_prod(1), 1); // qy
    // Serial.print(",");
    // Serial.print(q_prod(2), 1); // qz
    // Serial.print(",");
    // Serial.print(q_prod(3), 1); // qw
    //
    // Serial.println(",");
    return q_prod;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *   full quaternion.
 * @note This function is useful to convert delta quaternion
 *   which is usually a 3x1 vector to a full quaternion.
 *   For more details, check Section 3.2 "Kalman Filter Update" in
 *   "Indirect Kalman Filter for 3D Attitude Estimation:
 *   A Tutorial for quaternion Algebra".
 */
inline BLA::Matrix<4,1,float> smallAngleQuat (
    const BLA::Matrix<3,1,float>& dtheta )
{

    BLA::Matrix<3,1,float> dq = dtheta * 0.5f;
    BLA::Matrix<4,1,float> q;
    float dq_square_norm = dq(0)*dq(0) + dq(1)*dq(1) + dq(2)*dq(2);

    if ( dq_square_norm <= 1 ) {
        q.Submatrix<3,1>(0,0) = dq; // set q(0) = dq(0) ... q(2) = dq(2)
        q ( 3 ) = (float)sqrt ( 1.0f-dq_square_norm );
    } else {
        q.Submatrix<3,1>(0,0) = dq;
        q ( 3 ) = 1.0f;
        q = q / (float)sqrt ( 1.0f+dq_square_norm );
    }

    return q;
}

/*
 * @brief Convert a quaternion to the corresponding rotation matrix
 * @note Pay attention to the convention used. The function follows the
 *   conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *   A Tutorial for Quaternion Algebra", Equation (78).
 *
 *   The input quaternion should be in the form
 *     [q1, q2, q3, q4(scalar)]^T
 */
inline BLA::Matrix<3,3,float> quat2Rot (
    const BLA::Matrix<4,1,float>& q )
{
    const BLA::Matrix<3,1,float>& q_vec = q.Submatrix<3,1>(0,0);
    const float& q4 = q ( 3 );
    BLA::Matrix<3,3,float> Identity3 = {1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f};
    BLA::Matrix<3,3,float> R =
        Identity3 * (2.0f*q4*q4-1.0f) -
        skewSym(q_vec) * 2.0f*q4 +
        q_vec*(~q_vec) * 2.0f;
    //TODO: Is it necessary to use the approximation equation
    //    (Equation (87)) when the rotation angle is small?
    return R;
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *   conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *   A Tutorial for Quaternion Algebra", Equation (78).
 */
inline BLA::Matrix<4,1,float> rot2Quat (
    const BLA::Matrix<3,3,float>& R )
{
    float r00 = R ( 0, 0 );
    float r11 = R ( 1, 1 );
    float r22 = R ( 2, 2 );
    float trace_R = r00 + r11 + r22; // trace of R

    BLA::Matrix<4,1,float> q;
    q.Fill(0.0f);
    if ( r00 >= r11 && r00 >= r22 && r00 >= trace_R ) {
        q ( 0 ) = sqrt ( 1.0f+2.0f*R ( 0, 0 )-trace_R ) / 2.0f;
        q ( 1 ) = ( R ( 0, 1 )+R ( 1, 0 ) ) / ( 4.0f*q ( 0 ) );
        q ( 2 ) = ( R ( 0, 2 )+R ( 2, 0 ) ) / ( 4.0f*q ( 0 ) );
        q ( 3 ) = ( R ( 1, 2 )-R ( 2, 1 ) ) / ( 4.0f*q ( 0 ) );
    } else if ( r11 >= r22 && r11 >= trace_R ) {
        q ( 1 ) = sqrt ( 1.0f+2.0f*R ( 1, 1 )-trace_R ) / 2.0f;
        q ( 0 ) = ( R ( 0, 1 )+R ( 1, 0 ) ) / ( 4.0f*q ( 1 ) );
        q ( 2 ) = ( R ( 1, 2 )+R ( 2, 1 ) ) / ( 4.0f*q ( 1 ) );
        q ( 3 ) = ( R ( 2, 0 )-R ( 0, 2 ) ) / ( 4.0f*q ( 1 ) );
    } else if ( r22 >= trace_R ) {
        q ( 2 ) = sqrt ( 1.0f+2.0f*R ( 2, 2 )-trace_R ) / 2.0f;
        q ( 0 ) = ( R ( 0, 2 )+R ( 2, 0 ) ) / ( 4.0f*q ( 2 ) );
        q ( 1 ) = ( R ( 1, 2 )+R ( 2, 1 ) ) / ( 4.0f*q ( 2 ) );
        q ( 3 ) = ( R ( 0, 1 )-R ( 1, 0 ) ) / ( 4.0f*q ( 2 ) );
    } else {
        q ( 3 ) = sqrt ( 1.0f+trace_R ) / 2.0f;
        q ( 0 ) = ( R ( 1, 2 )-R ( 2, 1 ) ) / ( 4.0f*q ( 3 ) );
        q ( 1 ) = ( R ( 2, 0 )-R ( 0, 2 ) ) / ( 4.0f*q ( 3 ) );
        q ( 2 ) = ( R ( 0, 1 )-R ( 1, 0 ) ) / ( 4.0f*q ( 3 ) );
    }

    if ( q ( 3 ) < 0.0f ) q = -q;
    quatNorm ( q );
    return q;
}

/*
 * @brief Spherical linear interpolation for quaternions
 * @note Input: intial and final quaternions,
 *       and interp parameter t in range [0,1].
 */
inline BLA::Matrix<4,1,float> slerp(
    const BLA::Matrix<4,1,float>& qa,
    const BLA::Matrix<4,1,float>& qb,
    float t )
 {
    BLA::Matrix<4,1,float> qA = qa;
    BLA::Matrix<4,1,float> qB = qb;
    quatNorm(qA);
    quatNorm(qB);

    if (t < 0.000001) return qA;
    else if (t > 0.999999) return qB;
    else
    {
      // (qB*inv(qA))^t * qA
      float dot = qA(0)*qB(0) + qA(1)*qB(1) + qA(2)*qB(2) + qA(3)*qB(3);
      if (dot < 0)
      {
        qA *= -1.0f;
        dot *= -1.0f;
      }
      BLA::Matrix<4,1,float> qA_inv = {-qA(0), -qA(1), -qA(2), qA(3)};
      BLA::Matrix<4,1,float> q_temp1 = quatMult(qB, qA_inv);
      float fCos = q_temp1(3);
      float half_ang = acos(fCos); // half angle of rotation
      BLA::Matrix<3,1,float> v_temp = {q_temp1(0), q_temp1(1), q_temp1(2)};
      BLA::Matrix<3,1,float> v_axis = vectNorm(v_temp);
      BLA::Matrix<4,1,float> q_temp2;
      q_temp2(0) = v_axis(0)*sin(t*half_ang);
      q_temp2(1) = v_axis(1)*sin(t*half_ang);
      q_temp2(2) = v_axis(2)*sin(t*half_ang);
      q_temp2(3) = cos(t*half_ang);

      return quatMult(q_temp2, qA);
    }
 }

} // end namespace math_utils
