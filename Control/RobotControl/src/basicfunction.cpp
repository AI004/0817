#include "../include/basicfunction.h"
#include <iostream>
// #include "basicfunction.h"
namespace basicfunction {
void MatrixToEuler_ZYX(Eigen::Matrix3d R, Eigen::Vector3d& Euler) { Euler = R.eulerAngles(2, 1, 0); }

void MatrixToEuler_XYZ(Eigen::Matrix3d R, Eigen::Vector3d& Euler) { Euler = R.eulerAngles(0, 1, 2); }

void Euler_XYZToMatrix(Eigen::Matrix3d& R, Eigen::Vector3d euler_a) {
  R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
// add 2022.07.26
void Euler_ZYXToMatrix(Eigen::Matrix3d& R, Eigen::Vector3d euler_a) {
  R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d RotX(double x) { return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d RotY(double y) { return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d RotZ(double z) { return Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
Eigen::Matrix3d VelProjectionMatrix_EulerXYZ(Eigen::Vector3d Euler) {
  Eigen::Matrix3d R;
  R << 1., 0., sin(Euler[1]), 0., cos(Euler[0]), -cos(Euler[1]) * sin(Euler[0]), 0., sin(Euler[0]),
      cos(Euler[0]) * cos(Euler[1]);

  return R;
}

void euleraddoffset(Eigen::Vector3d& euler) {
  // std::cout<<"euler_a_pre: "<<std::endl<<euler.transpose()<<std::endl;
  Eigen::Matrix3d R_a;
  basicfunction::Euler_XYZToMatrix(R_a, euler);
  Eigen::Vector3d delt(0.785, 0.785, 0.785);
  Eigen::Matrix3d R_delt;
  basicfunction::Euler_XYZToMatrix(R_delt, delt);
  R_a = R_a * R_delt;
  basicfunction::MatrixToEuler_XYZ(R_a, euler);
  // std::cout<<"euler_a_after: "<<std::endl<<euler.transpose()<<std::endl;
}

void eulersuboffset(Eigen::Vector3d& euler) {
  // std::cout<<"euler_a_pre: "<<std::endl<<euler.transpose()<<std::endl;
  Eigen::Matrix3d R_a;
  basicfunction::Euler_XYZToMatrix(R_a, euler);
  // std::cout<<"R_a: "<<std::endl<<R_a<<std::endl;
  Eigen::Vector3d delt(0.785, 0.785, 0.785);
  Eigen::Matrix3d R_delt;
  basicfunction::Euler_XYZToMatrix(R_delt, delt);
  R_a = R_a * R_delt.inverse();
  // std::cout<<"R_a: "<<std::endl<<R_a<<std::endl;
  basicfunction::MatrixToEuler_XYZ(R_a, euler);
  // std::cout<<"euler_a_after: "<<std::endl<<euler.transpose()<<std::endl;
}

Eigen::Matrix3d skew(Eigen::Vector3d r) {
  Eigen::Matrix3d skew;
  skew << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0;
  return skew;
}

void matrixtoeulerxyz_(Eigen::Matrix3d R, Eigen::Vector3d& euler) {
  euler.setZero();
  // y (-pi/2 pi/2)
  euler(1) = asin(R(0, 2));
  // z [-pi pi]
  double sinz = -R(0, 1) / cos(euler(1));
  double cosz = R(0, 0) / cos(euler(1));
  euler(2) = atan2(sinz, cosz);
  // x [-pi pi]
  double sinx = -R(1, 2) / cos(euler(1));
  double cosx = R(2, 2) / cos(euler(1));
  euler(0) = atan2(sinx, cosx);
}

double intervaltime(struct timeval ts, struct timeval te) {
  return (te.tv_sec - ts.tv_sec) + (te.tv_usec - ts.tv_usec) / 1000000.0;
}

void quaternionToEuler(double w, double x, double y, double z, Eigen::Vector3d& euler_xyz) {
  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  euler_xyz(0) = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    euler_xyz(1) = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    euler_xyz(1) = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  euler_xyz(2) = std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace basicfunction
