#ifndef PLANTOOLS
#define PLANTOOLS
#include <Eigen/Dense>
#include <fstream>
#include <rbdl/rbdl.h>
Eigen::Vector3d matrixtoeulerxyz_(Eigen::Matrix3d R);
void quaternionInterp(Eigen::Matrix3d R_start, Eigen::Matrix3d R_end,
                      double totaltime, double currenttime,
                      Eigen::Matrix3d &R_d, Eigen::Vector3d &omiga_d,
                      Eigen::Vector3d &acc_d);
Eigen::Matrix3d QuanteiniontoMatrix(RigidBodyDynamics::Math::Quaternion Q);
void FifthPoly(Eigen::VectorXd p0, Eigen::VectorXd p0_dot,
               Eigen::VectorXd p0_dotdot, // start point states
               Eigen::VectorXd p1, Eigen::VectorXd p1_dot,
               Eigen::VectorXd p1_dotdot, // end point states
               double totalTime,          // total permating time
               double currenttime,        // current time,from 0 to total time
               Eigen::VectorXd &pd, Eigen::VectorXd &pd_dot,
               Eigen::VectorXd &pd_dotdot);
bool quintic(double x1, double x2, double y1, double y2, double dy1, double dy2,
             double ddy1, double ddy2, double x, double dx, double &y,
             double &dy, double &ddy);
double clamp(double num, double lim1, double lim2);
double myPow(double x, int n);
void Thirdpoly(double p0, double p0_dot, double p1, double p1_dot,
               double totalTime,   // total permating time
               double currenttime, // current time,from 0 to total time
               double &pd, double &pd_dot);
double fact(int n);
bool TriPointsQuintic(double T, double t, double x1, double v1, double x2,
                      double v2, double x3, double v3, double &y, double &dy,
                      double &ddy);
bool TriPointsQuintic(double T, double t, double t_mid, double x1, double v1,
                      double x2, double v2, double x3, double v3, double &y,
                      double &dy, double &ddy);
void TwoPointsCubic(double p0, double p0_dot, double p1, double p1_dot,
                    double totalTime,   // total permating time
                    double currenttime, // current time,from 0 to total time
                    double &pd, double &pd_dot, double &pd_ddot);
void airWalk(double t, double vCmd0, double &vCmd, double &tStepPre,
             int &stIndex, Eigen::VectorXd xStand, Eigen::VectorXd &xInit,
             Eigen::VectorXd &xDotInit, Eigen::VectorXd &xDDotInit,
             Eigen::VectorXd &xEnd, Eigen::VectorXd &xDotEnd,
             Eigen::VectorXd &xCmd, Eigen::VectorXd &xDotCmd,
             Eigen::VectorXd &xDDotCmd, Eigen::VectorXd &fCmd);
Eigen::Matrix3d rotX(double q);
Eigen::Matrix3d rotY(double q);
Eigen::Matrix3d rotZ(double q);
Eigen::Matrix3d rotn(double n1, double n2, double n3, double q);
void oneLegIK(Eigen::VectorXd Pos, Eigen::VectorXd RPY, Eigen::VectorXd &qIK);
void oneLegIK2(Eigen::VectorXd Pos, Eigen::Matrix3d R, Eigen::VectorXd &qIK);
void oneLegIK_new(double leg, Eigen::VectorXd Pos, Eigen::Matrix3d R,
                  Eigen::VectorXd &qIK);
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd &qCmd,
                   Eigen::VectorXd &qDotCmd, bool &firstFlag);
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd,
                   Eigen::VectorXd &qCmd, Eigen::VectorXd &qDotCmd,
                   bool &firstFlag);
void wkSpace2Joint(Eigen::VectorXd xCmd, Eigen::VectorXd rpyCmd,
                   Eigen::VectorXd rFootCmd, Eigen::VectorXd &qCmd,
                   Eigen::VectorXd &qDotCmd, bool &firstFlag);
void armPolyJoint(Eigen::VectorXd swingPhase, Eigen::VectorXd &qCmd);
bool dataLog(Eigen::VectorXd &v, std::ofstream &f);

#endif // PLANTOOLS
#pragma once