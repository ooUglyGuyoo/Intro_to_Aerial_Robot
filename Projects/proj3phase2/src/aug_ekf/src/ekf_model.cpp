#include <iostream>
#include <ekf_model.h>

using namespace std;
using namespace Eigen;

namespace ekf_imu_vision {

Vec15 modelF(const Vec15 &x, const Vec6 &u, const Vec12 &n) {
  
  // DONE: ===========================================================================================
  // return the model xdot = f(x,u,n)
  // x: state vector
  // u: input vector
  // n: noise vector

  Vec15 xdot;

  //  1: x0:2 ~ x, y, z """         postision
  //  2: x3:5 ~ phi theta psi """   orientation
  //  3: x6:8 ~ vx vy vz """        linear velocity
  //  4: x9:11 ~ bgx bgy bgz """    gyro bias
  //  5: x12:14 ~  bax bay baz """  acc bias

  // u0:2 wmx, wmy, wmz             angular velocity
  // u3:5 amx, amy, amz             linear acceleration

  // n0:2 ngx, ngy, ngz             gyro noise
  // n3:5 nax, nay, naz             acc noise
  // n6:8 nbgx, nbgy, nbgz          gyro bias noise
  // n9:11 nbax, nbay, nbaz         acc bias noise

  Vector3d gravity(0, 0, -9.8);
  Vector3d acc(u(0), u(1), u(2));
  Vector3d omega(u(3), u(4), u(5));

  Matrix3d G, R;
  double phi = x(3);
  double theta = x(4);
  double psi = x(5);
  G << cos(theta), 0, -cos(phi) * sin(theta), 0, 1, sin(phi), sin(theta), 0,
      cos(theta) * cos(phi);

  R << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);
  xdot.setZero();
  xdot.block<3, 1>(0, 0) = x.block<3, 1>(6, 0);
  xdot.block<3, 1>(3, 0) = G.inverse() * (omega - Vector3d(x(9), x(10), x(11)));
  xdot.block<3, 1>(6, 0) = gravity + R * (acc - Vector3d(x(12), x(13), x(14)));

  // cout << "xdot: " << xdot << endl;

  return xdot;
}


// ============================================================================================== //
// ============================================================================================== //
/* =================================== model of IMU prediction ================================== */

Mat15x15 jacobiFx(const Vec15 &x, const Vec6 &u, const Vec12 &n) {

  // DONE: ===========================================================================================
  // return the derivative wrt original state df/dx
  // x: state vector
  // u: input vector
  // n: noise vector
  
  Mat15x15 At;
  Vec3 angular_vel, acc;
  Mat3x3 G, G_inverse, R, G_inverse_dot, R_dot;
  double phi, theta, psi;

  angular_vel << u(0), u(1), u(2);
  acc << u(3), u(4), u(5);
  phi = x(3);
  theta = x(4);
  psi = x(5);

  G <<  cos(theta), 0, -cos(phi)*sin(theta),
        0,          1, sin(phi),
        sin(theta), 0, cos(theta)*cos(phi);

  R <<  cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta),  
        -cos(phi)*sin(psi), 
        cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),

        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),   
        cos(phi)*cos(psi),  
        sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),

        -cos(phi)*sin(theta),                               
        sin(phi),           
        cos(phi)*cos(theta);

  G_inverse = G.inverse();

  double x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15;
  double n1, n2, n3, n4, n5, n6, w1, w2, w3;
  x4 = x(3), x5 = x(4), x6 = x(5), x7 = x(6), x8 = x(7), x9 = x(8), x10 = x(9), x11 = x(10), x12 = x(11), x13 = x(12), x14 = x(13), x15 = x(14);
  n1 = 0, n2 = 0, n3 = 0, n4= 0, n5 = 0, n6 = 0;
  w1 = angular_vel(0), w2 = angular_vel(1), w3 = angular_vel(2);
  G_inverse_dot <<  
        0, 
        (sin(x5)*(n1 - w1 + x10))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)) - (cos(x5)*(n3 - w3 + x12))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)), 
        0, 

        (cos(x4)*cos(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x4)*sin(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(5)) + (cos(x5)*sin(x4)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (sin(x4)*sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
        - (sin(x4)*sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*sin(x4)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
        0,

        (sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
        (cos(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) + (sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
        0;

  R_dot <<  
        acc(1)*sin(phi)*sin(psi) + acc(2)*cos(phi)*cos(theta)*sin(psi) - acc(0)*cos(phi)*sin(theta)*sin(psi), 
        acc(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - acc(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), 
        -acc(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - acc(1)*cos(phi)*cos(psi),

        acc(0)*cos(phi)*cos(psi)*sin(theta) - acc(2)*cos(phi)*cos(theta)*cos(psi) - acc(1)*cos(psi)*sin(phi), 
        acc(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acc(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   
        acc(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + acc(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acc(1)*cos(phi)*sin(psi),

        acc(1)*cos(phi) - acc(2)*cos(theta)*sin(phi) + acc(0)*sin(phi)*sin(theta), 
        - acc(0)*cos(phi)*cos(theta) - acc(2)*cos(phi)*sin(theta), 
        0;

  At = Mat15x15::Zero(15,15);
  At.block<3,3>(0,6) = Mat3x3::Identity(3,3);
  At.block<3,3>(3,3) << G_inverse_dot;
  At.block<3,3>(6,3) << R_dot;
  At.block<3,3>(3,9) << -G_inverse;
  At.block<3,3>(6,12) << -R;
  // cout << "At: " << At << endl;
  return At;
}

Mat15x12 jacobiFn(const Vec15 &x, const Vec6 &u, const Vec12 &n) {

  // DONE: ===========================================================================================
  // return the derivative wrt noise df/dn

  Mat15x12 Ut;
  Vec3 angular_vel, acc;
  Mat3x3 G, G_inverse, R;
  double phi, theta, psi;

  angular_vel << u(0), u(1), u(2);
  acc << u(3), u(4), u(5);
  phi = x(3);
  theta = x(4);
  psi = x(5);

  G <<  cos(theta), 0, -cos(phi)*sin(theta),
        0,          1, sin(phi),
        sin(theta), 0, cos(theta)*cos(phi);

  R <<  cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta),  
        -cos(phi)*sin(psi), 
        cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),

        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),   
        cos(phi)*cos(psi),  
        sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),

        -cos(phi)*sin(theta),                               
        sin(phi),           
        cos(phi)*cos(theta);

  G_inverse = G.inverse();

  Ut =Mat15x12::Zero(15,12);
  Ut.block<3,3>(3,0) << -G_inverse;
  Ut.block<3,3>(6,3) << -R;
  Ut.block<6,6>(9,6) << Mat6x6::Identity(6,6);

  // cout << "Ut: " << Ut << endl;
  return Ut;
}



// ============================================================================================== //
// ============================================================================================== //
/* ====================================== model of PnP ========================================== */

Vec6 modelG1(const Vec15 &x, const Vec6 &v) {
  
  // DONE: ===========================================================================================
  // return the model g(x,v), where x = x_origin

  Vec6 zt;

  zt = x.segment(0,6);

  // cout << "zt_pnp: " << zt << endl;
  return zt;
}

Mat6x15 jacobiG1x(const Vec15 &x, const Vec6 &v) {

  // DONE: ===========================================================================================
  // return the derivative wrt original state dz/dx, where x = x_origin

  Mat6x15 Ct;

  Ct.setZero();
  Ct.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

  // cout << "Ct_pnp: " << Ct << endl;
  return Ct;
}

Mat6x6 jacobiG1v(const Vec15 &x, const Vec6 &v) {

  // DONE: ===========================================================================================
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  
  I6.setIdentity();

  // cout << "I6_pnp: " << I6 << endl;
  return I6;
}



// ============================================================================================== //
// ============================================================================================== //
/* ============================== model of stereo VO relative pose ============================== */

Vec6 modelG2(const Vec21 &x, const Vec6 &v) {

  // DONE: ===========================================================================================
  // return the model g(x,v), where x = (x_origin, x_augmented)

  Vec6 zt;
  Mat3x3 R, rotation_matrix, current_rotation_matrix;
  double phi, theta, psi;

  phi = x(18);
  theta = x(19);
  psi = x(20);

  R <<  cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta),  
        -cos(phi)*sin(psi), 
        cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),

        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),   
        cos(phi)*cos(psi),  
        sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),

        -cos(phi)*sin(theta),                               
        sin(phi),           
        cos(phi)*cos(theta);

  current_rotation_matrix <<  
        cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta), 
        -cos(phi) * sin(psi), 
        cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),

        cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta), 
        cos(phi) * cos(psi),  
        sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),

        -cos(phi) * sin(theta),                                   
        sin(phi),             
        cos(phi) * cos(theta);

  rotation_matrix = R.transpose() * current_rotation_matrix;

  zt.block<3,1>(0,0) = R.transpose() * (x.segment(0,3) - x.segment(15,3));
  zt(3) = asin(rotation_matrix(2,1));
  zt(4) = atan2(-rotation_matrix(2,0), rotation_matrix(2,2));
  zt(5) = atan2(-rotation_matrix(0,1), rotation_matrix(1,1));

  // cout << "zt_VO: " << zt << endl;
  return zt;
}

Mat6x21 jacobiG2x(const Vec21 &x, const Vec6 &v) {

  // DONE: ===========================================================================================
  // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)

  Mat6x21 Ct;
  Vec3 diff;
  Mat3x3 R, R_transpose, dRt_dPhi, dRt_dTheta, dRt_dPsi, dz_dR;
  Mat3x3 R_cur, rotate_motion; // middle element of calculating the derivatives
  Mat3x3 dRcurC1_db, dRtR2_dk, dRcurC0_db, dRcurC2_db, dRtR0_dk, dRtR1_dk; // middle element of calculating the derivatives
  double x_b, y_b, z_b, phi_b, theta_b, psi_b;
  double x_k, y_k, z_k, phi_k, theta_k, psi_k;
  double phi_coeff, theta_coeff, psi_coeff;

  // Define state and orientation variables
  x_b = x(0); y_b = x(1); z_b = x(2);
  x_k = x(15); y_k = x(16); z_k = x(17);
  phi_b = x(3); theta_b = x(4); psi_b = x(5); // b for body frame
  phi_k = x(18); theta_k = x(19); psi_k = x(20);
  diff << x(0) - x(15), x(1) - x(16), x(2) - x(17);
  
  // Calculation
  // R << 
  //     cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k),
  //     -cos(phi_k) * sin(psi_k),
  //     cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k),

  //     cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k),
  //     cos(phi_k) * cos(psi_k),
  //     sin(psi_k) * sin(theta_k) - cos(psi_k) * sin(phi_k) * cos(theta_k),

  //     -cos(phi_k) * sin(theta_k), 
  //     sin(phi_k), 
  //     cos(phi_k) * cos(theta_k);
  // R_transpose = R.transpose();

  // // Derivative of rotation matrix transpose base phi:    dRt_dPhi
  // dRt_dPhi <<
  //     -cos(phi_k) * sin(psi_k) * sin(theta_k),
  //     cos(psi_k) * cos(phi_k) * sin(theta_k), 
  //     sin(phi_k) * sin(theta_k),

  //     sin(phi_k) * sin(psi_k), 
  //     -sin(phi_k) * cos(psi_k), 
  //     cos(phi_k),
      
  //     cos(theta_k) * cos(phi_k) * sin(psi_k), 
  //     -cos(psi_k) * cos(theta_k) * cos(phi_k),
  //     -sin(phi_k) * cos(theta_k);

  // // Derivative of ritation matrix transpose base theta:   dRt_dTheta
  // dRt_dTheta << 
  //     -sin(theta_k) * cos(psi_k) - sin(phi_k) * sin(psi_k) * cos(theta_k),
  //     -sin(psi_k) * sin(theta_k) + cos(psi_k) * sin(phi_k) * cos(theta_k),
  //     -cos(phi_k) * cos(theta_k),

  //     0, 0, 0,

  //     cos(psi_k) * cos(theta_k) - sin(theta_k) * sin(phi_k) * sin(psi_k),
  //     sin(psi_k) * cos(theta_k) + cos(psi_k) * sin(theta_k) * sin(phi_k),
  //     -cos(phi_k) * sin(theta_k);

  // // Derivative of ritation matrix transpose base psi:      dRt_dPsi
  // dRt_dPsi << 
  //     -sin(psi_k) * cos(theta_k) - sin(phi_k) * cos(psi_k) * sin(theta_k),
  //     cos(theta_k) * cos(psi_k) - sin(psi_k) * sin(phi_k) * sin(theta_k),
  //     0,

  //     -cos(phi_k) * cos(psi_k),
  //     -cos(phi_k) * sin(psi_k),
  //     0,

  //     -sin(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * cos(psi_k),
  //     cos(psi_k) * sin(theta_k) + sin(psi_k) * cos(theta_k) * sin(phi_k),
  //     0;

  // // Derivative of motion base rotation:    dz_dR
  // dz_dR.block<3,1>(0,0) = dRt_dPhi * diff;
  // dz_dR.block<3,1>(0,1) = dRt_dTheta * diff;
  // dz_dR.block<3,1>(0,2) = dRt_dPsi * diff;

  // R_cur <<  
  //     cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b),
  //     -cos(phi_b) * sin(psi_b),
  //     cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b),

  //     cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b),
  //     cos(phi_b) * cos(psi_b),
  //     sin(psi_b) * sin(theta_b) - cos(psi_b) * sin(phi_b) * cos(theta_b),

  //     -cos(phi_b) * sin(theta_b), 
  //     sin(phi_b), 
  //     cos(phi_b) * cos(theta_b);

  // rotate_motion = R_transpose * R_cur;

  // phi_coeff = 1.0 / sqrt(1 - rotate_motion(2, 1) * rotate_motion(2, 1));
  // theta_coeff = -1.0 / (rotate_motion(2, 0) * rotate_motion(2, 0) + rotate_motion(2, 2) * rotate_motion(2, 2));
  // psi_coeff = -1.0 / (rotate_motion(0, 1) * rotate_motion(0, 1) + rotate_motion(1, 1) * rotate_motion(1, 1));

  // // Derivative of R_cur column 1 base rotation body frame:    dRcurC1_db
  // dRcurC1_db << 
  //     sin(phi_b) * sin(psi_b), 0, -cos(phi_b) * cos(psi_b),
  //     -sin(phi_b) * cos(psi_b), 0, -cos(phi_b) * sin(psi_b), 
  //     cos(phi_b), 0, 0;

  // // Derivative of R_transpose column 2 base keyframe:    dRtR2_dk
  // dRtR2_dk << 
  //     cos(theta_k) * cos(phi_k) * sin(psi_k),
  //     -cos(psi_k) * cos(theta_k) * cos(phi_k),
  //     -sin(phi_k) * cos(theta_k),

  //     cos(psi_k) * cos(theta_k) - sin(theta_k) * sin(phi_k) * sin(psi_k),
  //     sin(psi_k) * cos(theta_k) + cos(psi_k) * sin(theta_k) * sin(phi_k),
  //     -cos(phi_k) * sin(theta_k),

  //     -sin(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * cos(psi_k),
  //     cos(psi_k) * sin(theta_k) + sin(psi_k) * cos(theta_k) * sin(phi_k),
  //     0;

  // // Derivative of R_cur column 0 base rotation body frame:    dRcurC0_db
  // dRcurC0_db << 
  //     -cos(phi_b) * sin(psi_b) * sin(theta_b),
  //     -cos(psi_b) * sin(theta_b) - sin(phi_b) * sin(psi_b) * cos(theta_b),
  //     -sin(psi_b) * cos(theta_b) - sin(phi_b) * cos(psi_b) * sin(theta_b),

  //     cos(psi_b) * cos(phi_b) * sin(theta_b),
  //     -sin(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * cos(theta_b),
  //     cos(theta_b) * cos(psi_b) - sin(psi_b) * sin(phi_b) * sin(theta_b),

  //     sin(phi_b) * sin(theta_b),
  //     -cos(phi_b) * cos(theta_b),
  //     0;

  // // Derivative of R_cur column 2 base rotation body frame:    dRcurC2_db
  // dRcurC2_db << 
  //     cos(theta_b) * cos(phi_b) * sin(psi_b),
  //     cos(psi_b) * cos(theta_b) - sin(theta_b) * sin(phi_b) * sin(psi_b),
  //     -sin(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * cos(psi_b),

  //     -cos(psi_b) * cos(theta_b) * cos(phi_b),
  //     sin(psi_b) * cos(theta_b) + cos(psi_b) * sin(theta_b) * sin(phi_b),
  //     cos(psi_b) * sin(theta_b) + sin(psi_b) * cos(theta_b) * sin(phi_b),

  //     -sin(phi_b) * cos(theta_b), 
  //     -cos(phi_b) * sin(theta_b), 
  //     0;

  // // Derivative of R_transpose column 0 base rotation body frame:    dRtR0_dk
  // dRtR0_dk << 
  //     -cos(phi_k) * sin(psi_k) * sin(theta_k),
  //     cos(psi_k) * cos(phi_k) * sin(theta_k),
  //     sin(phi_k) * sin(theta_k),
      
  //     -cos(psi_k) * sin(theta_k) - sin(phi_k) * sin(psi_k) * cos(theta_k),
  //     -sin(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * cos(theta_k),
  //     -cos(phi_k) * cos(theta_k),

  //     -sin(psi_k) * cos(theta_k) - sin(phi_k) * cos(psi_k) * sin(theta_k),
  //     cos(theta_k) * cos(psi_k) - sin(psi_k) * sin(phi_k) * sin(theta_k),
  //     0;

  // // Derivative of R_transpose column 1 base rotation body frame:    dRtR1_dk
  // dRtR1_dk <<
  //     sin(phi_k) * sin(psi_k),
  //     -sin(phi_k) * cos(psi_k),
  //     cos(18),
  //     0, 0, 0,
  //     -cos(phi_k) * cos(psi_k),
  //     -cos(phi_k) * sin(psi_k),
  //     0;

  // Ct.setZero();
  // Ct.block<3, 3>(0, 0) = R_transpose;
  // Ct.block<3, 3>(0, 15) = -R_transpose;
  // Ct.block<3, 3>(0, 18) = dz_dR;
  // Ct.block<1, 3>(3, 3) = phi_coeff * R_transpose.block<1, 3>(2, 0) * dRcurC1_db;
  // Ct.block<1, 3>(3, 18) =
  //     phi_coeff * (dRtR2_dk * R_cur.block<3, 1>(0, 1)).transpose();
  // Ct.block<1, 3>(4, 3) =
  //     theta_coeff *
  //     (R_transpose.block<1, 3>(2, 0) * dRcurC0_db * rotate_motion(2, 2) -
  //      R_transpose.block<1, 3>(2, 0) * dRcurC2_db * rotate_motion(2, 0));
  // Ct.block<1, 3>(4, 18) =
  //     theta_coeff *
  //     (dRtR2_dk * R_cur.block<3, 1>(0, 0) * rotate_motion(2, 2) -
  //      dRtR2_dk * R_cur.block<3, 1>(0, 2) * rotate_motion(2, 0))
  //         .transpose();
  // Ct.block<1, 3>(5, 3) =
  //     psi_coeff * (R_transpose.block<1, 3>(0, 0) * dRcurC1_db * rotate_motion(1, 1) -
  //                 R_transpose.block<1, 3>(1, 0) * dRcurC1_db * rotate_motion(0, 1));

  // Ct.block<1, 3>(5, 18) =
  //     psi_coeff * (dRtR0_dk * R_cur.block<3, 1>(0, 1) * rotate_motion(1, 1) -
  //                 dRtR1_dk * R_cur.block<3, 1>(0, 1) * rotate_motion(0, 1))
  //                    .transpose();


  // Autograd
  Ct(0, 0) = cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k);
  Ct(0, 1) = cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k);
  Ct(0, 2) = -cos(phi_k) * sin(theta_k);
  Ct(0, 15) = -cos(psi_k) * cos(theta_k) + sin(phi_k) * sin(psi_k) * sin(theta_k);
  Ct(0, 16) = -cos(theta_k) * sin(psi_k) - cos(psi_k) * sin(phi_k) * sin(theta_k);
  Ct(0, 17) = cos(phi_k) * sin(theta_k);
  Ct(0, 18) = sin(phi_k) * sin(theta_k) * (z_b - z_k) + cos(phi_k) * cos(psi_k) * sin(theta_k) * (y_b - y_k) - cos(phi_k) * sin(psi_k) * sin(theta_k) * (x_b - x_k);
  Ct(0, 19) = -(cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) * (x_b - x_k) - (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) * (y_b - y_k) - cos(phi_k) * cos(theta_k) * (z_b - z_k);
  Ct(0, 20) = -(cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) * (x_b - x_k) + (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) * (y_b - y_k);
  Ct(1, 0) = -cos(phi_k) * sin(psi_k);
  Ct(1, 1) = cos(phi_k) * cos(psi_k);
  Ct(1, 2) = sin(phi_k);
  Ct(1, 15) = cos(phi_k) * sin(psi_k);
  Ct(1, 16) = -cos(phi_k) * cos(psi_k);
  Ct(1, 17) = -sin(phi_k);
  Ct(1, 18) = cos(phi_k) * (z_b - z_k) - cos(psi_k) * sin(phi_k) * (y_b - y_k) + sin(phi_k) * sin(psi_k) * (x_b - x_k);
  Ct(1, 20) = -cos(phi_k) * cos(psi_k) * (x_b - x_k) - cos(phi_k) * sin(psi_k) * (y_b - y_k);
  Ct(2, 0) = cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k);
  Ct(2, 1) = sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k);
  Ct(2, 2) = cos(phi_k) * cos(theta_k);
  Ct(2, 15) = -cos(psi_k) * sin(theta_k) - cos(theta_k) * sin(phi_k) * sin(psi_k);
  Ct(2, 16) = -sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_k) * sin(phi_k);
  Ct(2, 17) = -cos(phi_k) * cos(theta_k);
  Ct(2, 18) = -cos(theta_k) * sin(phi_k) * (z_b - z_k) - cos(phi_k) * cos(psi_k) * cos(theta_k) * (y_b - y_k) + cos(phi_k) * cos(theta_k) * sin(psi_k) * (x_b - x_k);
  Ct(2, 19) = (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) * (x_b - x_k) + (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) * (y_b - y_k) - cos(phi_k) * sin(theta_k) * (z_b - z_k);
  Ct(2, 20) = -(sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) * (x_b - x_k) + (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) * (y_b - y_k);
  Ct(3, 3) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(psi_b) * sin(phi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + sin(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_k));
  Ct(3, 5) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
  Ct(3, 18) = -cos(theta_k) * 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k));
  Ct(3, 19) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k));
  Ct(3, 20) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
  Ct(4, 3) = (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) * (-cos(phi_k) * cos(theta_k) * sin(phi_b) - cos(phi_b) * cos(psi_b) * sin(psi_k) * sin(theta_k) + cos(phi_b) * cos(psi_k) * sin(psi_b) * sin(theta_k) + cos(phi_b) * cos(psi_b) * cos(psi_k) * cos(theta_k) * sin(phi_k) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(4, 4) = ((1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0) + 1.0) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(4, 5) = ((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(4, 18) = -(((-cos(phi_k) * cos(psi_k) * cos(theta_k) * (cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) + cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(theta_b)) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b)) * (cos(phi_k) * cos(psi_k) * cos(theta_k) * (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) - cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) + cos(phi_b) * cos(theta_b) * cos(theta_k) * sin(phi_k))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(4, 19) = -((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(4, 20) = -((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
  Ct(5, 3) = (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
  Ct(5, 5) = (((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
  Ct(5, 18) = -((sin(theta_k) - 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_k) * sin(phi_b) + cos(phi_b) * cos(psi_b) * cos(psi_k) * sin(phi_k) + cos(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
  Ct(5, 19) = ((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * (cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b))) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
  Ct(5, 20) = -(((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));

  return Ct;
  
  // cout << "Ct_VO: " << Ct << endl;
  return Ct;
}

Mat6x6 jacobiG2v(const Vec21 &x, const Vec6 &v) {
  
  // DONE: ===========================================================================================
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  
  I6 = Mat6x6::Identity();

  // cout << "I6_VO: " << I6 << endl;
  return I6;
}

}  // namespace ekf_imu_vision