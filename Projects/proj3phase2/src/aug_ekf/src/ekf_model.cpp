#include <ekf_model.h>

namespace ekf_imu_vision {

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {
  
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

  return xdot;
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt original state df/dx

  Mat15x15 At;

  return At;
}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt noise df/dn

  Mat15x12 Ut;

  return Ut;
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15& x, const Vec6& v) {
  
  // TODO
  // return the model g(x,v), where x = x_origin

  Vec6 zt;

  return zt;
}

Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = x_origin

  Mat6x15 Ct;

  return Ct;
}

Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v) {

  // TODO;
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  
  return I6;
}

/* ============================== model of stereo VO relative pose ============================== */

Vec6 modelG2(const Vec21& x, const Vec6& v) {

  // TODO
  // return the model g(x,v), where x = (x_origin, x_augmented)

  Vec6 zt;

  return zt;
}

Mat6x21 jacobiG2x(const Vec21& x, const Vec6& v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)

  Mat6x21 Ct;
  
  return Ct;
}

Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {
  
  // TODO
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;
  
  return I6;
}

}  // namespace ekf_imu_vision