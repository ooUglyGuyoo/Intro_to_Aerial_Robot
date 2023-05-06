#include <ekf_model.h>
using namespace Eigen;
namespace ekf_imu_vision {

///////////////////////////////////////////////////////////////////////////////////////////////////
Vec15 modelF(const Vec15 &x, const Vec6 &u, const Vec12 &n) {

  // TODO
  // return the model xdot = f(x,u,n)

  Vec15 xdot;
  //  1: x0:2 ~ x, y, z """
  //  2: x3:5 ~ phi theta psi """
  //  3: x6:8 ~ vx vy vz """
  //  4: x9:11 ~ bgx bgy bgz """
  //  5: x12:14 ~  bax bay baz """

  // u0:2 wmx, wmy, wmz
  // u3:5 amx, amy, amz

  // n0:2 ngx, ngy, ngz
  // n3:5 nax, nay, naz
  // n6:8 nbgx, nbgy, nbgz
  // n9:11 nbax, nbay, nbaz

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

  return xdot;
}

Mat15x15 jacobiFx(const Vec15 &x, const Vec6 &u, const Vec12 &n) {

  // TODO
  // return the derivative wrt original state df/dx
  Matrix3d G, R, GInvDiv, RDiv;
  double phi = x(3);
  double theta = x(4);
  double psi = x(5);

  Vector3d acc(u(0), u(1), u(2));
  Vector3d omega(u(3), u(4), u(5));

  G << cos(theta), 0, -cos(phi) * sin(theta), 0, 1, sin(phi), sin(theta), 0,
      cos(theta) * cos(phi);
  Matrix3d GInv = G.inverse();

  GInvDiv << 0, omega(2) * cos(theta) - omega(0) * sin(theta), 0,
      omega(0) * sin(theta) - omega(2) * cos(theta) -
          (omega(2) * cos(theta) * sin(phi) * sin(phi)) /
              (cos(phi) * cos(phi)) +
          (omega(0) * sin(phi) * sin(phi) * sin(theta)) / (cos(phi) * cos(phi)),
      (omega(0) * cos(theta) * sin(phi)) / cos(phi) +
          (omega(2) * sin(phi) * sin(theta)) / cos(phi),
      0,
      (omega(2) * cos(theta) * sin(phi)) / (cos(phi) * cos(phi)) -
          (omega(0) * sin(phi) * sin(theta)) / (cos(phi) * cos(phi)),
      -(omega(0) * cos(theta)) / cos(phi) - (omega(2) * sin(theta)) / cos(phi),
      0;

  RDiv << acc(1) * sin(phi) * sin(psi) +
              acc(2) * cos(phi) * cos(theta) * sin(psi) -
              acc(0) * cos(phi) * sin(theta) * sin(psi),
      acc(2) * (cos(theta) * cos(psi) - sin(phi) * sin(theta) * sin(psi)) -
          acc(0) * (cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi)),
      -acc(0) * (cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta)) -
          acc(2) * (sin(theta) * sin(psi) - cos(theta) * cos(psi) * sin(phi)) -
          acc(1) * cos(phi) * cos(psi),
      acc(0) * cos(phi) * cos(psi) * sin(theta) -
          acc(2) * cos(phi) * cos(theta) * cos(psi) -
          acc(1) * cos(psi) * sin(phi),
      acc(2) * (cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta)) -
          acc(0) * (sin(theta) * sin(psi) - cos(theta) * cos(psi) * sin(phi)),
      acc(0) * (cos(theta) * cos(psi) - sin(phi) * sin(theta) * sin(psi)) +
          acc(2) * (cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi)) -
          acc(1) * cos(phi) * sin(psi),
      acc(1) * cos(phi) - acc(2) * cos(theta) * sin(phi) +
          acc(0) * sin(phi) * sin(theta),
      -acc(0) * cos(phi) * cos(theta) - acc(2) * cos(phi) * sin(theta), 0;

  R << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);

  Mat15x15 At;
  At.setZero();
  At.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3);
  At.block<3, 3>(3, 3) << GInvDiv;
  At.block<3, 3>(6, 3) << RDiv;
  At.block<3, 3>(3, 9) << -GInv;
  At.block<3, 3>(6, 12) << -R;
  return At;
}

Mat15x12 jacobiFn(const Vec15 &x, const Vec6 &u, const Vec12 &n) {

  // TODO
  // return the derivative wrt noise df/dn
  Matrix3d G, R, GInvDiv, RDiv;
  double phi = x(3);
  double theta = x(4);
  double psi = x(5);
  G << cos(theta), 0, -cos(phi) * sin(theta), 0, 1, sin(phi), sin(theta), 0,
      cos(theta) * cos(phi);
  Matrix3d GInv = G.inverse();

  R << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);

  Mat15x12 Ut;
  Ut.setZero();
  Ut.block<3, 3>(3, 0) << -GInv;
  Ut.block<3, 3>(6, 3) << -R;
  Ut.block<6, 6>(9, 6) << MatrixXd::Identity(6, 6);
  return Ut;
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15 &x, const Vec6 &v) {

  // TODO
  // return the model g(x,v), where x = x_origin

  Vec6 zt;
  // Aligned version
  zt << x(0), x(1), x(2), x(3), x(4), x(5);
  return zt;
}

Mat6x15 jacobiG1x(const Vec15 &x, const Vec6 &v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = x_origin

  Mat6x15 Ct;
  // Aligned version
  Ct.setZero();
  Ct.block<6, 6>(0, 0) = Mat6x6::Identity();
  return Ct;
}

Mat6x6 jacobiG1v(const Vec15 &x, const Vec6 &v) {

  // TODO;
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  // Aligned version
  I6 = Mat6x6::Identity();
  return I6;
}

/* ============================== model of stereo VO relative pose
 * ============================== */

Vec6 modelG2(const Vec21 &x, const Vec6 &v) {

  // TODO
  // return the model g(x,v), where x = (x_origin, x_augmented)

  Vec6 zt;
  // Aligned version
  Mat3x3 R;
  Vec3 err;
  err << x(0) - x(15), x(1) - x(16), x(2) - x(17);
  double phi = x(18);
  double theta = x(19);
  double psi = x(20);
  R << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);
  Mat3x3 Rot_Cur;
  phi = x(3);
  theta = x(4);
  psi = x(5);
  Rot_Cur << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);
  Mat3x3 Rot;
  Rot = R.transpose() * Rot_Cur;

  //   zt(3) = asin(Rot(1, 2));
  //   zt(4) = atan2(-Rot(1, 0) / cos(zt(3)), Rot(1, 1) / cos(zt(3)));
  //   zt(5) = atan2(-Rot(0, 2) / cos(zt(3)), Rot(2, 2) / cos(zt(3)));
  //   zt(3) = asin(Rot(2, 1));
  //   zt(4) = atan2(-Rot(0, 1) / cos(zt(3)), Rot(1, 1) / cos(zt(3)));
  //   zt(5) = atan2(-Rot(2, 0) / cos(zt(3)), Rot(2, 2) / cos(zt(3)));
  zt.block<3, 1>(0, 0) = R.transpose() * err;
  zt(3) = asin(Rot(2, 1));
  zt(4) = atan2(-Rot(2, 0), Rot(2, 2));
  zt(5) = atan2(-Rot(0, 1), Rot(1, 1));
  return zt;
}

Mat6x21 jacobiG2x(const Vec21 &x, const Vec6 &v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = (x_origin,
  // x_augmented)

  Mat6x21 Ct;
  // Aligned version
  Mat3x3 R;
  double phi = x(18);
  double theta = x(19);
  double psi = x(20);
  R << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);
  Vec3 err;
  err << x(0) - x(15), x(1) - x(16), x(2) - x(17);
  Mat3x3 RTDivPhi;
  RTDivPhi << -cos(phi) * sin(psi) * sin(theta),
      cos(psi) * cos(phi) * sin(theta), sin(phi) * sin(theta),

      sin(phi) * sin(psi), -sin(phi) * cos(psi), cos(phi),

      cos(theta) * cos(phi) * sin(psi), -cos(psi) * cos(theta) * cos(phi),
      -sin(phi) * cos(theta);

  Mat3x3 RTDivTheta;
  RTDivTheta << -sin(theta) * cos(psi) - sin(phi) * sin(psi) * cos(theta),
      -sin(psi) * sin(theta) + cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * cos(theta), 0, 0, 0,
      cos(psi) * cos(theta) - sin(theta) * sin(phi) * sin(psi),
      sin(psi) * cos(theta) + cos(psi) * sin(theta) * sin(phi),
      -cos(phi) * sin(theta);

  Mat3x3 RTDivPsi;
  RTDivPsi << -sin(psi) * cos(theta) - sin(phi) * cos(psi) * sin(theta),
      cos(theta) * cos(psi) - sin(psi) * sin(phi) * sin(theta), 0,
      -cos(phi) * cos(psi), -cos(phi) * sin(psi), 0,
      -sin(psi) * sin(theta) + cos(theta) * sin(phi) * cos(psi),
      cos(psi) * sin(theta) + sin(psi) * cos(theta) * sin(phi), 0;
  Mat3x3 MotionDivAugRot;
  MotionDivAugRot.block<3, 1>(0, 0) = RTDivPhi * err;
  MotionDivAugRot.block<3, 1>(0, 1) = RTDivTheta * err;
  MotionDivAugRot.block<3, 1>(0, 2) = RTDivPsi * err;

  Mat3x3 R_Cur;
  phi = x(3);
  theta = x(4);
  psi = x(5);
  R_Cur << cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
      -cos(phi) * sin(psi),
      cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
      cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
      cos(phi) * cos(psi),
      sin(psi) * sin(theta) - cos(psi) * sin(phi) * cos(theta),
      -cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);
  Mat3x3 RotateMotion, RT;
  RT = R.transpose();
  RotateMotion = RT * R_Cur;
  double phiCoeff;
  phiCoeff = 1.0 / sqrt(1 - RotateMotion(2, 1) * RotateMotion(2, 1));
  Mat3x3 R_CurCol1DotX345, RTRow2DotX890;
  R_CurCol1DotX345 << sin(x(3)) * sin(x(5)), 0, -cos(x(3)) * cos(x(5)),
      -sin(x(3)) * cos(x(5)), 0, -cos(x(3)) * sin(x(5)), cos(x(3)), 0, 0;

  RTRow2DotX890 << cos(x(19)) * cos(x(18)) * sin(x(20)),
      -cos(x(20)) * cos(x(19)) * cos(x(18)), -sin(x(18)) * cos(x(19)),

      cos(x(20)) * cos(x(19)) - sin(x(19)) * sin(x(18)) * sin(x(20)),
      sin(x(20)) * cos(x(19)) + cos(x(20)) * sin(x(19)) * sin(x(18)),
      -cos(x(18)) * sin(x(19)),

      -sin(x(20)) * sin(x(19)) + cos(x(19)) * sin(x(18)) * cos(x(20)),
      cos(x(20)) * sin(x(19)) + sin(x(20)) * cos(x(19)) * sin(x(18)), 0;

  double thetaCoeff;
  thetaCoeff = -1.0 / (RotateMotion(2, 0) * RotateMotion(2, 0) +
                       RotateMotion(2, 2) * RotateMotion(2, 2));
  Mat3x3 R_CurCol0DotX345, R_CurCol2DotX345;
  R_CurCol0DotX345 << -cos(x(3)) * sin(x(5)) * sin(x(4)),
      -cos(x(5)) * sin(x(4)) - sin(x(3)) * sin(x(5)) * cos(x(4)),
      -sin(x(5)) * cos(x(4)) - sin(x(3)) * cos(x(5)) * sin(x(4)),

      cos(x(5)) * cos(x(3)) * sin(x(4)),
      -sin(x(4)) * sin(x(5)) + cos(x(5)) * sin(x(3)) * cos(x(4)),
      cos(x(4)) * cos(x(5)) - sin(x(5)) * sin(x(3)) * sin(x(4)),

      sin(x(3)) * sin(x(4)), -cos(x(3)) * cos(x(4)), 0;
  R_CurCol2DotX345 << cos(x(4)) * cos(x(3)) * sin(x(5)),
      cos(x(5)) * cos(x(4)) - sin(x(4)) * sin(x(3)) * sin(x(5)),
      -sin(x(5)) * sin(x(4)) + cos(x(4)) * sin(x(3)) * cos(x(5)),

      -cos(x(5)) * cos(x(4)) * cos(x(3)),
      sin(x(5)) * cos(x(4)) + cos(x(5)) * sin(x(4)) * sin(x(3)),
      cos(x(5)) * sin(x(4)) + sin(x(5)) * cos(x(4)) * sin(x(3)),

      -sin(x(3)) * cos(x(4)), -cos(x(3)) * sin(x(4)), 0;

  double psiCoeff;
  psiCoeff = -1.0 / (RotateMotion(0, 1) * RotateMotion(0, 1) +
                     RotateMotion(1, 1) * RotateMotion(1, 1));
  Mat3x3 RTRow0DotX890, RTRow1DotX890;
  RTRow0DotX890 << -cos(x(18)) * sin(x(20)) * sin(x(19)),
      cos(x(20)) * cos(x(18)) * sin(x(19)), sin(x(18)) * sin(x(19)),

      -cos(x(20)) * sin(x(19)) - sin(x(18)) * sin(x(20)) * cos(x(19)),
      -sin(x(19)) * sin(x(20)) + cos(x(20)) * sin(x(18)) * cos(x(19)),
      -cos(x(18)) * cos(x(19)),

      -sin(x(20)) * cos(x(19)) - sin(x(18)) * cos(x(20)) * sin(x(19)),
      cos(x(19)) * cos(x(20)) - sin(x(20)) * sin(x(18)) * sin(x(19)), 0;
  RTRow1DotX890 << sin(x(18)) * sin(x(20)), -sin(x(18)) * cos(x(20)), cos(18),
      0, 0, 0, -cos(x(18)) * cos(x(20)), -cos(x(18)) * sin(x(20)), 0;

  Ct.setZero();
  Ct.block<3, 3>(0, 0) = RT;
  Ct.block<3, 3>(0, 15) = -RT;
  Ct.block<3, 3>(0, 18) = MotionDivAugRot;
  Ct.block<1, 3>(3, 3) = phiCoeff * RT.block<1, 3>(2, 0) * R_CurCol1DotX345;
  Ct.block<1, 3>(3, 18) =
      phiCoeff * (RTRow2DotX890 * R_Cur.block<3, 1>(0, 1)).transpose();
  Ct.block<1, 3>(4, 3) =
      thetaCoeff *
      (RT.block<1, 3>(2, 0) * R_CurCol0DotX345 * RotateMotion(2, 2) -
       RT.block<1, 3>(2, 0) * R_CurCol2DotX345 * RotateMotion(2, 0));
  Ct.block<1, 3>(4, 18) =
      thetaCoeff *
      (RTRow2DotX890 * R_Cur.block<3, 1>(0, 0) * RotateMotion(2, 2) -
       RTRow2DotX890 * R_Cur.block<3, 1>(0, 2) * RotateMotion(2, 0))
          .transpose();
  Ct.block<1, 3>(5, 3) =
      psiCoeff * (RT.block<1, 3>(0, 0) * R_CurCol1DotX345 * RotateMotion(1, 1) -
                  RT.block<1, 3>(1, 0) * R_CurCol1DotX345 * RotateMotion(0, 1));

  Ct.block<1, 3>(5, 18) =
      psiCoeff * (RTRow0DotX890 * R_Cur.block<3, 1>(0, 1) * RotateMotion(1, 1) -
                  RTRow1DotX890 * R_Cur.block<3, 1>(0, 1) * RotateMotion(0, 1))
                     .transpose();
  return Ct;
}

Mat6x6 jacobiG2v(const Vec21 &x, const Vec6 &v) {

  // TODO
  // return the derivative wrt noise dz/dv

  Mat6x6 I6 = Mat6x6::Identity();

  return I6;
}
} // namespace ekf_imu_vision