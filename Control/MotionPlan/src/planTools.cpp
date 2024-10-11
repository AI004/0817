#include "planTools.h"
#include <stdlib.h>
#include <iostream>

Eigen::Vector3d matrixtoeulerxyz_(Eigen::Matrix3d R) {
  Eigen::Vector3d euler;
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
  return euler;
}

void quaternionInterp(Eigen::Matrix3d R_start, Eigen::Matrix3d R_end, double totaltime, double currenttime,
                      Eigen::Matrix3d& R_d, Eigen::Vector3d& omiga_d, Eigen::Vector3d& acc_d) {
  RigidBodyDynamics::Math::Quaternion Q_start = Q_start.fromMatrix(R_start).conjugate();
  RigidBodyDynamics::Math::Quaternion Q_end = Q_end.fromMatrix(R_end).conjugate();
  RigidBodyDynamics::Math::Quaternion Q_d;

  RigidBodyDynamics::Math::Quaternion deltQ = Q_start.conjugate() * Q_end;

  Eigen::Vector3d p = (deltQ.block<3, 1>(0, 0));
  if (p.norm() != 0) {
    p = p / p.norm();
  }

  double delttheta = 2 * acos(deltQ(3));

  Eigen::VectorXd p0 = Eigen::VectorXd::Zero(1);  // p0 = p0 p0_dot p0_ddot p1_dot p1_ddot

  Eigen::VectorXd p1 = Eigen::VectorXd::Zero(1);
  p1[0] = delttheta;

  Eigen::VectorXd pd = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd pd_dot = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd pd_ddot = Eigen::VectorXd::Zero(1);

  FifthPoly(p0, p0, p0, p1, p0, p0, totaltime, currenttime, pd, pd_dot, pd_ddot);
  deltQ.block<3, 1>(0, 0) = p * sin(pd[0] / 2.0);
  deltQ[3] = cos(pd[0] / 2.0);
  Q_d = Q_start * deltQ;
  // Q_d = Q_d/sqrt(Q_d.dot(Q_d));

  R_d = QuanteiniontoMatrix(Q_d);
  omiga_d = R_start * p * pd_dot[0];
  acc_d = R_start * p * pd_ddot[0];

  // std::cout<<"R_d:"<<std::endl<<R_d<<std::endl;
  // std::cout<<"Q_end:"<<std::endl<<Q_end<<std::endl;
  // std::cout<<"omiga_d:"<<std::endl<<omiga_d.transpose()<<std::endl;
  // std::cout<<"acc_d:"<<std::endl<<acc_d.transpose()<<std::endl;
}

Eigen::Matrix3d QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q) {
  double x = Q[0];
  double y = Q[1];
  double z = Q[2];
  double w = Q[3];
  Eigen::Matrix3d R;
  R << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,

      2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x,

      2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y;
  return R;
}

void FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot,
               Eigen::VectorXd p0_dotdot,  // start point states
               Eigen::VectorXd p1, Eigen::VectorXd p1_dot,
               Eigen::VectorXd p1_dotdot,  // end point states
               double totalTime,           // total permating time
               double currenttime,         // current time,from 0 to total time
               Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot, Eigen::VectorXd& pd_dotdot) {
  double t = currenttime;
  double time = totalTime;
  if (t < totalTime) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    A << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 / 2, 0, 0, 0, -10 / pow(time, 3), -6 / pow(time, 2),
        -3 / (2 * time), 10 / pow(time, 3), -4 / pow(time, 2), 1 / (2 * time), 15 / pow(time, 4), 8 / pow(time, 3),
        3 / (2 * pow(time, 2)), -15 / pow(time, 4), 7 / pow(time, 3), -1 / pow(time, 2), -6 / pow(time, 5),
        -3 / pow(time, 4), -1 / (2 * pow(time, 3)), 6 / pow(time, 5), -3 / pow(time, 4), 1 / (2 * pow(time, 3));
    Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 1);
    for (int i = 0; i < p0.size(); i++) {
      x0 << p0(i), p0_dot(i), p0_dotdot(i), p1(i), p1_dot(i), p1_dotdot(i);
      a = A * x0;
      pd(i) = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t;
      pd_dot(i) = a(1) + 2 * a(2) * t + 3 * a(3) * t * t + 4 * a(4) * t * t * t + 5 * a(5) * t * t * t * t;
      pd_dotdot(i) = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t * t + 20 * a(5) * t * t * t;
    }
  } else {
    pd = p1;
    pd_dot = p1_dot;
    pd_dotdot = p1_dotdot;
  }
}

bool quintic(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x,
             double dx, double& y, double& dy, double& ddy) {
  // Limit range since curve fit is only valid within range
  x = clamp(x, x1, x2);

  // Declare variables
  double t, t1, t2, deltaY, deltaT, a0, a1, a2, a3, a4, a5;

  // variable substitution
  t = x / dx;
  t1 = x1 / dx;
  t2 = x2 / dx;
  // interpolate
  deltaY = y2 - y1;
  deltaT = t2 - t1;
  a0 = y1;
  a1 = dy1;
  a2 = 1.0 / 2. * ddy1;
  a3 = 1.0 / (2. * deltaT * deltaT * deltaT) *
       (20. * deltaY - (8. * dy2 + 12. * dy1) * deltaT + (ddy2 - 3. * ddy1) * (deltaT * deltaT));
  a4 = 1.0 / (2. * deltaT * deltaT * deltaT * deltaT) *
       (-30. * deltaY + (14. * dy2 + 16. * dy1) * deltaT + (3. * ddy1 - 2. * ddy2) * (deltaT * deltaT));
  a5 = 1.0 / (2. * deltaT * deltaT * deltaT * deltaT * deltaT) *
       (12. * deltaY - 6. * (dy2 + dy1) * deltaT + (ddy2 - ddy1) * (deltaT * deltaT));

  // position
  y = a0 + a1 * myPow((t - t1), 1) + a2 * myPow((t - t1), 2) + a3 * myPow((t - t1), 3) + a4 * myPow(t - t1, 4) +
      a5 * myPow(t - t1, 5);
  // velocity
  dy = a1 + 2. * a2 * myPow((t - t1), 1) + 3. * a3 * myPow((t - t1), 2) + 4. * a4 * myPow(t - t1, 3) +
       5. * a5 * myPow(t - t1, 4);
  // acceleration
  ddy = 2. * a2 + 6. * a3 * myPow((t - t1), 1) + 12. * a4 * myPow(t - t1, 2) + 20. * a5 * myPow(t - t1, 3);

  return true;
}
double clamp(double num, double lim1, double lim2) {
  auto min = std::min(lim1, lim2);
  auto max = std::max(lim1, lim2);

  if (num < min) return min;

  if (max < num) return max;

  return num;
}
double myPow(double x, int n) {
  if (n == 0) return 1.0;
  if (n < 0) return 1.0 / myPow(x, -n);
  double half = myPow(x, n >> 1);

  if (n % 2 == 0)
    return half * half;
  else {
    return half * half * x;
  }
}

void Thirdpoly(double p0, double p0_dot, double p1, double p1_dot,
               double totalTime,    // total permating time
               double currenttime,  // current time,from 0 to total time
               double& pd, double& pd_dot) {
  if (currenttime < totalTime) {
    double a0 = p0;
    double a1 = p0_dot;
    double m = p1 - p0 - p0_dot * totalTime;
    double n = p1_dot - p0_dot;
    double a2 = 3 * m / (totalTime * totalTime) - n / totalTime;
    double a3 = -2 * m / (totalTime * totalTime * totalTime) + n / (totalTime * totalTime);
    pd = a3 * currenttime * currenttime * currenttime + a2 * currenttime * currenttime + a1 * currenttime + a0;
    pd_dot = 3 * a3 * currenttime * currenttime + 2 * a2 * currenttime + a1;
  } else {
    pd = p1;
    pd_dot = p1_dot;
  }
}
double fact(int n) {
  double result = 1;
  if (n == 0)
    result = 1;
  else
    for (int i = 1; i <= n; result *= i, i++);
  return result;
}
bool TriPointsQuintic(double T, double t, double x1, double v1, double x2, double v2, double x3, double v3, double& y,
                      double& dy, double& ddy) {
  double a0, a1, a2, a3, a4, a5;
  a0 = x1;
  a1 = v1;
  a2 = -(23. * x1 - 16. * x2 - 7. * x3 + 6. * T * v1 + 8 * T * v2 + T * v3) / myPow(T, 2);
  a3 = (66. * x1 - 32. * x2 - 34. * x3 + 13. * T * v1 + 32. * T * v2 + 5. * T * v3) / myPow(T, 3);
  a4 = -4. * (17. * x1 - 4. * x2 - 13. * x3 + 3. * T * v1 + 10. * T * v2 + 2. * T * v3) / myPow(T, 4);
  a5 = 4. * (6. * x1 - 6. * x3 + T * v1 + 4. * T * v2 + T * v3) / myPow(T, 5);

  y = a0 + a1 * myPow(t, 1) + a2 * myPow(t, 2) + a3 * myPow(t, 3) + a4 * myPow(t, 4) + a5 * myPow(t, 5);
  dy = a1 + 2. * a2 * myPow(t, 1) + 3. * a3 * myPow(t, 2) + 4. * a4 * myPow(t, 3) + 5. * a5 * myPow(t, 4);
  ddy = 2. * a2 + 6. * a3 * t + 12. * a4 * myPow(t, 2) + 20. * a5 * myPow(t, 3);
  return true;
}
bool TriPointsQuintic(double T, double t, double t_mid, double x1, double v1, double x2, double v2, double x3,
                      double v3, double& y, double& dy, double& ddy) {
  double a0, a1, a2, a3, a4, a5;
  a0 = x1;
  a1 = v1;
  a2 = -(23. * x1 - 16. * x2 - 7. * x3 + 6. * T * v1 + 8 * T * v2 + T * v3) / myPow(T, 2);
  a3 = (66. * x1 - 32. * x2 - 34. * x3 + 13. * T * v1 + 32. * T * v2 + 5. * T * v3) / myPow(T, 3);
  a4 = -4. * (17. * x1 - 4. * x2 - 13. * x3 + 3. * T * v1 + 10. * T * v2 + 2. * T * v3) / myPow(T, 4);
  a5 = 4. * (6. * x1 - 6. * x3 + T * v1 + 4. * T * v2 + T * v3) / myPow(T, 5);

  double ratio1 = 0.5 * T / t_mid;
  double ratio2 = 0.5 * T / (T - t_mid);
  double t1, ratio;
  if (t < t_mid) {
    t1 = ratio1 * t;
    ratio = ratio1;
  } else {
    t1 = 0.5 * T + (t - t_mid) * ratio2;
    ratio = ratio2;
  }
  y = a0 + a1 * myPow(t1, 1) + a2 * myPow(t1, 2) + a3 * myPow(t1, 3) + a4 * myPow(t1, 4) + a5 * myPow(t1, 5);
  dy = (a1 + 2. * a2 * myPow(t1, 1) + 3. * a3 * myPow(t1, 2) + 4. * a4 * myPow(t1, 3) + 5. * a5 * myPow(t1, 4)) * ratio;
  ddy = (2. * a2 + 6. * a3 * t1 + 12. * a4 * myPow(t1, 2) + 20. * a5 * myPow(t1, 3)) * ratio * ratio;
  return true;
};
void TwoPointsCubic(double p0, double p0_dot, double p1, double p1_dot,
                    double totalTime,    // total permating time
                    double currenttime,  // current time,from 0 to total time
                    double& pd, double& pd_dot, double& pd_ddot) {
  double a0 = p0;
  double a1 = p0_dot;
  double m = p1 - p0 - p0_dot * totalTime;
  double n = p1_dot - p0_dot;
  double a2 = 3. * m / (totalTime * totalTime) - n / totalTime;
  double a3 = -2. * m / (totalTime * totalTime * totalTime) + n / (totalTime * totalTime);

  if (currenttime < totalTime) {
    pd = a3 * currenttime * currenttime * currenttime + a2 * currenttime * currenttime + a1 * currenttime + a0;
    pd_dot = 3. * a3 * currenttime * currenttime + 2. * a2 * currenttime + a1;
    pd_ddot = 6. * a3 * currenttime + 2. * a2;
  } else {
    pd = p1;
    pd_dot = p1_dot;
    pd_ddot = 0.0;  // 6.*a3*totalTime + 2.*a2;
  }
}
void airWalk(double t, double vCmd0, double& vCmd, double& tStepPre, int& stIndex, Eigen::VectorXd xStand,
             Eigen::VectorXd& xInit, Eigen::VectorXd& xDotInit, Eigen::VectorXd& xDDotInit, Eigen::VectorXd& xEnd,
             Eigen::VectorXd& xDotEnd, Eigen::VectorXd& xCmd, Eigen::VectorXd& xDotCmd, Eigen::VectorXd& xDDotCmd,
             Eigen::VectorXd& fCmd) {
  double timeStep = 0.0025;
  double tStep, s;
  double T = 0.4;
  double tTrans = 0.2;
  int M = 6, N = 6;
  bool startFlag = false;
  Eigen::VectorXd xPoly(N), xDotPoly(N), xDDotPoly(N), torPoly(N);
  double vZInit = 0.0, vZEnd = -0.0, zMid = 0.08;
  double fZ = -350.;
  double tForce = 0.02;

  tStep = fmod(t, T);
  if (T - tStep < 0.5 * timeStep || tStep < 0.5 * timeStep) {
    tStep = 0.0;
    stIndex = 1 - stIndex;
    startFlag = true;
    vCmd = vCmd0;
    // vCmd += 0.02;
    // if (vCmd > 1.5) {
    //   vCmd = 1.5;
    // }
  }

  tStepPre = tStep;
  s = tStep / T;

  if (startFlag) {
    xInit = xCmd;
    xDotInit = xDotCmd;
    xDDotInit = xDDotCmd;
  }

  if (stIndex == 0) {
    xEnd(0) = xStand(0) - vCmd * T * 0.5;
    xDotEnd(0) = -vCmd;
    xEnd(1) = xStand(1);
    xDotEnd(1) = 0.0;
    xEnd(2) = xStand(2);
    xDotEnd(2) = vZInit;
    xEnd(3) = xStand(3) + vCmd * T * 0.5;
    xDotEnd(3) = -vCmd;
    xEnd(4) = xStand(4);
    xDotEnd(4) = 0.0;
    xEnd(5) = xStand(5);
    xDotEnd(5) = vZEnd;
  } else {
    xEnd(3) = xStand(3) - vCmd * T * 0.5;
    xDotEnd(3) = -vCmd;
    xEnd(4) = xStand(4);
    xDotEnd(4) = 0.0;
    xEnd(5) = xStand(5);
    xDotEnd(5) = vZInit;
    xEnd(0) = xStand(0) + vCmd * T * 0.5;
    xDotEnd(0) = -vCmd;
    xEnd(1) = xStand(1);
    xDotEnd(1) = 0.0;
    xEnd(2) = xStand(2);
    xDotEnd(2) = vZEnd;
  }
  double xTrans = 0.0, xDotTrans = 0.0, xDDotTrans = 0.0;
  for (int i = 0; i < N; ++i) {
    // Thirdpoly(xInit(i), xDotInit(i), xEnd(i), xDotEnd(i), T, tStep+timeStep,
    // xTrans , xDotTrans);
    TwoPointsCubic(xInit(i), xDotInit(i), xEnd(i), xDotEnd(i), T, tStep + timeStep, xTrans, xDotTrans, xDDotTrans);
    xCmd(i) = xTrans;
    xDotCmd(i) = xDotTrans;
    xDDotCmd(i) = xDDotTrans;
    fCmd(i) = 0.0;
  }
  if (stIndex == 0) {
    TriPointsQuintic(T, tStep + timeStep, xInit(5), xDotInit(5), zMid + xStand(5), 0., xStand(5), vZEnd, xTrans,
                     xDotTrans, xDDotTrans);
    xCmd(5) = xTrans;
    xDotCmd(5) = xDotTrans;
    xDDotCmd(5) = xDDotTrans;
    // Thirdpoly(xInit(2), xDotInit(2), xEnd(2), 0.0, 0.1*T, tStep+timeStep,
    // xTrans , xDotTrans);
    TwoPointsCubic(xInit(2), xDotInit(2), xEnd(2), 0.0, 0.1 * T, tStep + timeStep, xTrans, xDotTrans, xDDotTrans);
    xCmd(2) = xTrans;
    xDotCmd(2) = xDotTrans;
    xDDotCmd(2) = xDDotTrans;
  } else {
    TriPointsQuintic(T, tStep + timeStep, xInit(2), xDotInit(2), zMid + xStand(2), 0., xStand(2), vZEnd, xTrans,
                     xDotTrans, xDDotTrans);
    xCmd(2) = xTrans;
    xDotCmd(2) = xDotTrans;
    xDDotCmd(2) = xDDotTrans;
    // Thirdpoly(xInit(5), xDotInit(5), xEnd(5), 0.0, 0.1*T, tStep+timeStep,
    // xTrans , xDotTrans);
    TwoPointsCubic(xInit(5), xDotInit(5), xEnd(5), 0.0, 0.1 * T, tStep + timeStep, xTrans, xDotTrans, xDDotTrans);
    xCmd(5) = xTrans;
    xDotCmd(5) = xDotTrans;
    xDDotCmd(5) = xDDotTrans;
  }

  if (tStep < tForce) {
    fCmd(3 * stIndex + 2) = tStep / tForce * fZ;
    fCmd(3 * (1 - stIndex) + 2) = fZ - tStep / tForce * fZ;
  } else {
    fCmd(3 * stIndex + 2) = fZ;
    fCmd(3 * (1 - stIndex) + 2) = 0.0;
  }
}
Eigen::Matrix3d rotX(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << 1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c;
  return m_ret;
}
Eigen::Matrix3d rotY(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return m_ret;
}
Eigen::Matrix3d rotZ(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return m_ret;
}
Eigen::Matrix3d rotn(double n1, double n2, double n3, double q) {
  Eigen::Matrix3d R;
  double c = std::cos(q);
  double s = std::sin(q);
  R(0, 0) = (1.0 - c) * n1 * n1 + c;
  R(0, 1) = (1.0 - c) * n1 * n2 - n3 * s;
  R(0, 2) = (1.0 - c) * n1 * n3 + n2 * s;
  R(1, 0) = (1.0 - c) * n1 * n2 + n3 * s;
  R(1, 1) = (1.0 - c) * n2 * n2 + c;
  R(1, 2) = (1.0 - c) * n2 * n3 - n1 * s;
  R(2, 0) = (1.0 - c) * n1 * n3 - n2 * s;
  R(2, 1) = (1.0 - c) * n2 * n3 + n1 * s;
  R(2, 2) = (1.0 - c) * n3 * n3 + c;
  return R;
}
void oneLegIK(Eigen::VectorXd Pos, Eigen::VectorXd RPY, Eigen::VectorXd& qIK) {
  double l2 = 0.1305, l3 = 0.36, l4 = 0.34, l6 = 0.039;

  double px = Pos(0), py = Pos(1), pz = Pos(2);
  Eigen::Matrix3d R = rotZ(RPY(2)) * rotY(RPY(1)) * rotX(RPY(0));
  double r11 = R(0, 0), r22 = R(1, 1), r33 = R(2, 2);
  double r12 = R(0, 1), r13 = R(0, 2), r23 = R(1, 2);
  double r21 = R(1, 0), r31 = R(2, 0), r32 = R(2, 1);

  double q1, q2, q3, q4, q5, q6;
  double s1, s2, s3, s4, s5, s6;
  double c1, c2, c3, c4, c5, c6;

  q6 = atan((py * r22 + px * r12 + pz * r32) / (py * r23 + px * r13 + pz * r33 + l6));
  s6 = sin(q6);
  c6 = cos(q6);

  q1 = atan((r32 * c6 - r33 * s6) / (r22 * c6 - r23 * s6));
  s1 = sin(q1);
  c1 = cos(q1);

  q2 = asin(r13 * s6 - r12 * c6);
  s2 = sin(q2);
  c2 = cos(q2);

  Eigen::Vector3d PosHip(0.0, 0.0, -l2);
  Eigen::Vector3d PosFoot(0.0, 0.0, -l6);

  Eigen::Vector3d v1 = Pos - R * PosFoot;
  Eigen::Vector3d v2 = rotX(q1) * PosHip;
  Eigen::Vector3d v3 = v1 - v2;

  double dd = v3.transpose() * v3;
  q4 = M_PI - acos((l3 * l3 + l4 * l4 - dd) / (2 * l3 * l4));
  s4 = sin(q4);
  c4 = cos(q4);

  double A = l4 * r11 + l3 * r11 * c4 + l3 * r13 * c6 * s4 + l3 * r12 * s4 * s6;
  double B = l3 * r11 * s4 - l4 * r13 * c6 - l4 * r12 * s6 - l3 * r13 * c4 * c6 - l3 * r12 * c4 * s6;
  double C = px + l6 * r13;
  double D = sqrt(A * A + B * B - C * C);
  q5 = 2.0 * atan2((A - D), (B + C));
  s5 = sin(q5);
  c5 = cos(q5);

  q3 = asin((r13 * c4 * c5 * c6 - r11 * c5 * s4 - r11 * c4 * s5 + r12 * c4 * c5 * s6 - r13 * c6 * s4 * s5 -
             r12 * s4 * s5 * s6) /
            (c2));

  qIK(0) = q1;
  qIK(1) = q2;
  qIK(2) = q3;
  qIK(3) = q4;
  qIK(4) = q5;
  qIK(5) = q6;

  if (qIK(4) > M_PI) {
    qIK(4) = qIK(4) - 2 * M_PI;
  } else if (qIK(4) < -M_PI) {
    qIK(4) = qIK(4) + 2 * M_PI;
  }
}
void oneLegIK2(Eigen::VectorXd Pos, Eigen::Matrix3d R, Eigen::VectorXd& qIK) {
  double l2 = 0.1305, l3 = 0.36, l4 = 0.34, l6 = 0.039;

  double px = Pos(0), py = Pos(1), pz = Pos(2);

  double r11 = R(0, 0), r22 = R(1, 1), r33 = R(2, 2);
  double r12 = R(0, 1), r13 = R(0, 2), r23 = R(1, 2);
  double r21 = R(1, 0), r31 = R(2, 0), r32 = R(2, 1);

  double q1, q2, q3, q4, q5, q6;
  double s1, s2, s3, s4, s5, s6;
  double c1, c2, c3, c4, c5, c6;

  q6 = atan((py * r22 + px * r12 + pz * r32) / (py * r23 + px * r13 + pz * r33 + l6));
  s6 = sin(q6);
  c6 = cos(q6);

  q1 = atan((r32 * c6 - r33 * s6) / (r22 * c6 - r23 * s6));
  s1 = sin(q1);
  c1 = cos(q1);

  q2 = asin(r13 * s6 - r12 * c6);
  s2 = sin(q2);
  c2 = cos(q2);

  Eigen::Vector3d PosHip(0.0, 0.0, -l2);
  Eigen::Vector3d PosFoot(0.0, 0.0, -l6);

  Eigen::Vector3d v1 = Pos - R * PosFoot;
  Eigen::Vector3d v2 = rotX(q1) * PosHip;
  Eigen::Vector3d v3 = v1 - v2;

  double dd = v3.transpose() * v3;
  q4 = M_PI - acos((l3 * l3 + l4 * l4 - dd) / (2 * l3 * l4));
  s4 = sin(q4);
  c4 = cos(q4);

  double A = l4 * r11 + l3 * r11 * c4 + l3 * r13 * c6 * s4 + l3 * r12 * s4 * s6;
  double B = l3 * r11 * s4 - l4 * r13 * c6 - l4 * r12 * s6 - l3 * r13 * c4 * c6 - l3 * r12 * c4 * s6;
  double C = px + l6 * r13;
  double D = sqrt(A * A + B * B - C * C);
  q5 = 2.0 * atan2((A - D), (B + C));
  s5 = sin(q5);
  c5 = cos(q5);

  q3 = asin((r13 * c4 * c5 * c6 - r11 * c5 * s4 - r11 * c4 * s5 + r12 * c4 * c5 * s6 - r13 * c6 * s4 * s5 -
             r12 * s4 * s5 * s6) /
            (c2));

  qIK(0) = q1;
  qIK(1) = q2;
  qIK(2) = q3;
  qIK(3) = q4;
  qIK(4) = q5;
  qIK(5) = q6;

  if (qIK(4) > M_PI) {
    qIK(4) = qIK(4) - 2 * M_PI;
  } else if (qIK(4) < -M_PI) {
    qIK(4) = qIK(4) + 2 * M_PI;
  }
}
void oneLegIK_new(double leg, Eigen::VectorXd Pos, Eigen::Matrix3d R, Eigen::VectorXd& qIK) {
  double L2 = 0.0185, L31 = 0.0355, L3 = -0.369 - 0.0525, L4 = -0.37, L6 = -0.05;
  double max = 0.79;  // std::sqrt(L2*L2 + L3*L3) + fabs(L4)
  double q1, q2, q3, q4, q5, q6;

  double sign = 2.0 * (leg - 0.5);
  double rotate = 0.61087;
  L31 = sign * L31;
  rotate = sign * rotate;
  R = rotX(rotate).transpose() * R;
  Pos = rotX(rotate).transpose() * Pos;

  Eigen::Vector3d p_f(0.0, 0.0, L6);
  Eigen::Vector3d p_l = Pos - R * p_f;
  double norm_l = p_l.norm();
  if (norm_l > max) {
    std::cout << "Exceed Work Space :" << norm_l << std::endl;
    p_l = p_l / norm_l * max;
    norm_l = max;
  }
  double n1 = -p_l(0) / norm_l;
  double n2 = -p_l(1) / norm_l;
  double n3 = -p_l(2) / norm_l;
  Eigen::Vector3d z0(n1, n2, n3);
  double norm_e = std::sqrt(L2 * L2 + L3 * L3);
  double norm_l2 = std::sqrt(norm_l * norm_l - L31 * L31);
  double alpha = std::acos((norm_e * norm_e + norm_l2 * norm_l2 - L4 * L4) / (2.0 * norm_e * norm_l2));
  double beta = std::asin(-L3 / norm_e);
  double gamma = alpha + beta;
  Eigen::Vector3d p_l0(norm_l2 * std::cos(gamma), L31, -norm_l2 * std::sin(gamma));
  double a = p_l0(2) / p_l0(1);
  double b = p_l(1) / p_l0(1);
  double q2_0 = std::asin((-a * b + std::sqrt(a * a + 1.0 - b * b)) / (a * a + 1.0));
  if (fabs(a * std::sin(q2_0) + b - std::cos(q2_0)) > 1e-5) {
    q2_0 = std::asin((-a * b - std::sqrt(a * a + 1.0 - b * b)) / (a * a + 1.0));
  }
  double a00 = p_l0(0), a01 = p_l0(2) * std::cos(q2_0) + p_l0(1) * std::sin(q2_0), b0 = p_l(0);
  double a10 = p_l0(2) * std::cos(q2_0) + p_l0(1) * std::sin(q2_0), a11 = -p_l0(0), b1 = p_l(2);
  double q1_0 = std::asin((a00 * b1 - a10 * b0) / (a00 * a11 - a01 * a10));
  // q1_0 = -M_PI -q1_0;
  Eigen::Vector3d p_e0(L2, 0.0, L3);
  p_e0 = rotY(q1_0) * rotX(q2_0) * p_e0;
  Eigen::Vector3d p_k0(L2, L31, L3);
  p_k0 = rotY(q1_0) * rotX(q2_0) * p_k0;
  Eigen::Vector3d y0 = p_k0.cross(p_l);
  y0 = y0 / y0.norm();
  Eigen::Vector3d x0 = y0.cross(z0);
  Eigen::Matrix3d R_l;
  R_l << x0, y0, z0;
  Eigen::Matrix3d R_lw = R_l.transpose() * R;

  double q_l = std::atan2(R_lw(1, 0), R_lw(0, 0));
  Eigen::Vector3d p_e = rotn(n1, n2, n3, q_l) * p_e0;
  q2 = std::asin(-p_e(1) / L3);
  q1 = std::asin((L3 * p_e(0) * std::cos(q2) - L2 * p_e(2)) / (L3 * L3 * std::cos(q2) * std::cos(q2) + L2 * L2));
  // q1 = -M_PI -q1;
  Eigen::Vector3d p_2b(L2, 0.0, 0.0);
  Eigen::Vector3d p_2 = rotY(q1) * p_2b;

  Eigen::Vector3d ve_y = (p_2 - p_e).cross(p_l - p_e);
  Eigen::Vector3d ve_x = (p_2 - p_e).cross(ve_y);
  ve_x = ve_x / ve_x.norm();
  Eigen::Vector3d w_proj = (p_l - p_e).dot(ve_x) * ve_x;
  Eigen::Vector3d v_ez = (p_2 - p_e) / (p_2 - p_e).norm();
  double theta = 2 * std::asin(L31 / w_proj.norm());
  Eigen::Vector3d p_k = p_e + 0.5 * w_proj + rotn(v_ez(0), v_ez(1), v_ez(2), theta) * (-0.5 * w_proj);

  Eigen::Vector3d l2e = p_e - p_2;
  Eigen::Vector3d lew = p_l - p_k;
  q4 = std::acos(l2e.dot(lew) / l2e.norm() / lew.norm());
  q3 = std::asin((lew(1) + L4 * std::cos(q4) * std::sin(q2)) / (L4 * std::cos(q2) * std::sin(q4)));

  Eigen::Matrix3d R_56 = (rotY(q1) * rotX(q2) * rotZ(q3) * rotY(q4)).transpose() * R;
  double yaw_v = std::atan2(R_56(1, 0), R_56(0, 0));

  int i = 0;
  double yaw_vg, gradient, q_lg;
  double dq = 0.0001;
  while (fabs(yaw_v) > 1.0e-4) {
    q_lg = q_l + dq;
    p_e = rotn(n1, n2, n3, q_lg) * p_e0;
    q2 = std::asin(-p_e(1) / L3);
    q1 = std::asin((L3 * p_e(0) * std::cos(q2) - L2 * p_e(2)) / (L3 * L3 * std::cos(q2) * std::cos(q2) + L2 * L2));
    // q1 = -M_PI -q1;
    p_2 = rotY(q1) * p_2b;
    ve_y = (p_2 - p_e).cross(p_l - p_e);
    ve_x = (p_2 - p_e).cross(ve_y);
    ve_x = ve_x / ve_x.norm();
    w_proj = (p_l - p_e).dot(ve_x) * ve_x;
    v_ez = (p_2 - p_e) / (p_2 - p_e).norm();
    theta = 2 * std::asin(L31 / w_proj.norm());
    p_k = p_e + 0.5 * w_proj + rotn(v_ez(0), v_ez(1), v_ez(2), theta) * (-0.5 * w_proj);
    l2e = p_e - p_2;
    lew = p_l - p_k;
    q4 = std::acos(l2e.dot(lew) / l2e.norm() / lew.norm());
    q3 = std::asin((lew(1) + L4 * std::cos(q4) * std::sin(q2)) / (L4 * std::cos(q2) * std::sin(q4)));
    R_56 = (rotY(q1) * rotX(q2) * rotZ(q3) * rotY(q4)).transpose() * R;
    yaw_vg = std::atan2(R_56(1, 0), R_56(0, 0));
    gradient = (yaw_vg - yaw_v) / dq;

    q_l = q_l - yaw_v / gradient;
    p_e = rotn(n1, n2, n3, q_l) * p_e0;
    q2 = std::asin(-p_e(1) / L3);
    q1 = std::asin((L3 * p_e(0) * std::cos(q2) - L2 * p_e(2)) / (L3 * L3 * std::cos(q2) * std::cos(q2) + L2 * L2));
    // q1 = -M_PI -q1;
    p_2 = rotY(q1) * p_2b;
    ve_y = (p_2 - p_e).cross(p_l - p_e);
    ve_x = (p_2 - p_e).cross(ve_y);
    ve_x = ve_x / ve_x.norm();
    w_proj = (p_l - p_e).dot(ve_x) * ve_x;
    v_ez = (p_2 - p_e) / (p_2 - p_e).norm();
    theta = 2 * std::asin(L31 / w_proj.norm());
    p_k = p_e + 0.5 * w_proj + rotn(v_ez(0), v_ez(1), v_ez(2), theta) * (-0.5 * w_proj);
    l2e = p_e - p_2;
    lew = p_l - p_k;
    q4 = std::acos(l2e.dot(lew) / l2e.norm() / lew.norm());
    q3 = std::asin((lew(1) + L4 * std::cos(q4) * std::sin(q2)) / (L4 * std::cos(q2) * std::sin(q4)));
    R_56 = (rotY(q1) * rotX(q2) * rotZ(q3) * rotY(q4)).transpose() * R;
    yaw_v = std::atan2(R_56(1, 0), R_56(0, 0));
    // std::cout << "yaw_v:" << yaw_v << std::endl;
    i++;
    if (i > 10) {
      break;
    }
  }
  q5 = std::asin(-R_56(2, 0));
  q6 = std::asin(-R_56(1, 2));
  qIK << q1, q2 + rotate, q3, q4, q5, q6;
}
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd& qCmd, Eigen::VectorXd& qDotCmd, bool& firstFlag) {
  double timeStep = 0.0025;
  Eigen::Vector3d offSetL(0.0, 0.15285, -0.06658), offSetR(0.0, -0.15285, -0.06658);
  Eigen::Vector3d PosL = xCmd.head(3) - offSetL, PosR = xCmd.tail(3) - offSetR;
  Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
  Eigen::Vector3d RPY(0.0, 0.0, 0.0);
  Eigen::Matrix3d rotI = Eigen::Matrix3d::Identity();
  Eigen::VectorXd qCmd_pre = qCmd;
  oneLegIK_new(0.0, PosL, rotI, qIKL);
  oneLegIK_new(1.0, PosR, rotI, qIKR);
  qCmd.head(6) = qIKL;
  qCmd.tail(6) = qIKR;
  if (!firstFlag) qDotCmd = (qCmd - qCmd_pre) / timeStep;
  firstFlag = false;
}
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd, Eigen::VectorXd& qCmd, Eigen::VectorXd& qDotCmd,
                   bool& firstFlag) {
  double timeStep = 0.0025;
  Eigen::Vector3d offSetL(0.0, 0.15285, -0.06658), offSetR(0.0, -0.15285, -0.06658);
  Eigen::Vector3d PosL, PosR;
  Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qCmd_pre = qCmd;
  Eigen::Matrix3d rotR, rotL;
  rotL = (rotX(rpyCmd(0)) * rotY(rpyCmd(1)) * rotZ(rpyCmd(2))).transpose();
  rotR = (rotX(rpyCmd(3)) * rotY(rpyCmd(4)) * rotZ(rpyCmd(5))).transpose();
  PosL = rotL * xCmd.head(3) - offSetL;
  PosR = rotR * xCmd.tail(3) - offSetR;
  oneLegIK_new(0.0, PosL, rotL, qIKL);
  oneLegIK_new(1.0, PosR, rotR, qIKR);
  qCmd.head(6) = qIKL;
  qCmd.tail(6) = qIKR;
  if (!firstFlag) qDotCmd = (qCmd - qCmd_pre) / timeStep;
  firstFlag = false;
}
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd, Eigen::VectorXd rFootCmd, Eigen::VectorXd& qCmd,
                   Eigen::VectorXd& qDotCmd, bool& firstFlag) {
  double timeStep = 0.0025;
  Eigen::Vector3d offSetL(0.0, 0.15285, -0.06658), offSetR(0.0, -0.15285, -0.06658);
  Eigen::Vector3d PosL, PosR;
  Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qCmd_pre = qCmd;
  Eigen::Matrix3d rotR, rotL, rotR_foot, rotL_foot;
  rotL = (rotX(rpyCmd(0)) * rotY(rpyCmd(1)) * rotZ(rpyCmd(2))).transpose();
  rotR = (rotX(rpyCmd(3)) * rotY(rpyCmd(4)) * rotZ(rpyCmd(5))).transpose();
  PosL = rotL * xCmd.head(3) - offSetL;
  PosR = rotR * xCmd.tail(3) - offSetR;
  rotL_foot = rotL * (rotX(rFootCmd(0)) * rotY(rFootCmd(1)) * rotZ(rFootCmd(2)));
  rotR_foot = rotR * (rotX(rFootCmd(3)) * rotY(rFootCmd(4)) * rotZ(rFootCmd(5)));
  oneLegIK_new(0.0, PosL, rotL_foot, qIKL);
  oneLegIK_new(1.0, PosR, rotR_foot, qIKR);
  qCmd.head(6) = qIKL;
  qCmd.tail(6) = qIKR;
  if (!firstFlag) qDotCmd = (qCmd - qCmd_pre) / timeStep;
  firstFlag = false;
}

// arm swing polynominal trajectory
void armPolyJoint(Eigen::VectorXd swingPhase, Eigen::VectorXd& qCmd) {
  Eigen::Matrix<double, 4, 6> swingArmParas;
  swingArmParas << 12.56, -37.76, 43.19, -21.44, 2.107, 0.7214, 2.929, -9.473, 10.91, -5.094, 0.778, -1.897, -47.5,
      145.7, -160.8, 71.49, -6.84, -1.267, -13.84, 37.61, -34.5, 11.37, -1.05, -0.1551;
  Eigen::VectorXd initPos = Eigen::VectorXd::Zero(8);
  initPos << 0.0, -2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0;
  Eigen::VectorXd leftPhasePow = Eigen::VectorXd::Zero(6);
  leftPhasePow << myPow(swingPhase(0), 5), myPow(swingPhase(0), 4), myPow(swingPhase(0), 3), myPow(swingPhase(0), 2),
      myPow(swingPhase(0), 1), myPow(swingPhase(0), 0);

  Eigen::VectorXd rightPhasePow = Eigen::VectorXd::Zero(6);
  rightPhasePow << myPow(swingPhase(1), 5), myPow(swingPhase(1), 4), myPow(swingPhase(1), 3), myPow(swingPhase(1), 2),
      myPow(swingPhase(1), 1), myPow(swingPhase(1), 0);

  qCmd.head(4) = swingArmParas * leftPhasePow;
  qCmd.tail(4) = -swingArmParas * rightPhasePow;
  qCmd(4) *= -1.0;
  qCmd(6) *= -1.0;
  qCmd -= initPos;
  qCmd(2) = 0.0;
  qCmd(6) = 0.0;
  qCmd(7) *= -1.0;
}

// logData
bool dataLog(Eigen::VectorXd& v, std::ofstream& f) {
  for (int i = 0; i < v.size(); i++) {
    f << v[i] << " ";
  }
  f << std::endl;
  return true;
}
