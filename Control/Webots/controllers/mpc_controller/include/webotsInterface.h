
#ifndef WEBOTS_INTERFACE_H
#define WEBOTS_INTERFACE_H

#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "public_parament.h"

#ifndef PI
#  define PI 3.141592654
#endif  // PI

#ifndef TIME_STEP
#  define TIME_STEP (2.5)
#endif  // TIME_STEP

#ifndef SAMPLE_TIME
#  define SAMPLE_TIME (0.0025f)
#endif  // SAMPLE_TIME

#ifndef LEFTFOOT
#  define LEFTFOOT 0
#endif  // LEFTFOOT

#ifndef RIGHTFOOT
#  define RIGHTFOOT 1
#endif  // RIGHTFOOT

using namespace webots;

/**
 * @brief The Derivative class
 * Derivative in S-Domain:
 *            s
 * D(s) = -----------
 *          cs + 1
 *
 * e.g. c = 1e-4
 *
 * Tusting approximation:
 *                    1 - z^-1
 *  S(z) = alpha * ---------------
 *                    1 + z^-1
 */
class Derivative {
 public:
  Derivative();
  Derivative(double dT, double c);
  void init(double dT, double c, double initValue);
  double mSig(double sigIn, double dT);

 private:
  double a0, a1, b0, b1;
  double sigInPrev;
  double sigOutPrev;
};

struct webotState {
  Eigen::VectorXd jointPosAct = Eigen::VectorXd::Zero(actor_num);
  Eigen::VectorXd jointVelAct = Eigen::VectorXd::Zero(actor_num);
  Eigen::VectorXd jointTorAct = Eigen::VectorXd::Zero(actor_num);
  Eigen::VectorXd imu9DAct = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd footGrfAct = Eigen::VectorXd::Zero(12);
  Eigen::Vector3d waistRpyAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistRpyVelAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzVelAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d waistXyzAccAct = Eigen::Vector3d::Zero();
};

/**
 * @brief The WebotsRobot class
 */
class WebotsRobot {
 public:
  Supervisor* robot = new Supervisor();

  void initWebots();
  void deleteRobot();
  bool readData(double simTime, webotState& robotState);
  bool setMotorPos(const Eigen::VectorXd& jointPosTar);
  bool setMotorTau(const Eigen::VectorXd& jointTauTar);
  int nJoint = actor_num;
  std::vector<Motor*> legMotor;

 private:
  Eigen::VectorXd getMotorPos();
  Eigen::VectorXd getMotorTau();
  Eigen::Vector3d getWaistAcc();
  // Eigen::VectorXd getFootForce6D(const int& footFlag);
  // Eigen::VectorXd getFootForce12D();
  Eigen::Vector3d rotm2Rpy(const Eigen::Matrix3d& rotm);
  Eigen::Vector3d rotm2xyz(const Eigen::Matrix3d& R);
  Eigen::Matrix3d rotx(const double theta);

  std::vector<PositionSensor*> legSensor;
  std::vector<Motor*> torqueSensor;
  std::vector<TouchSensor*> forceSensor;
  InertialUnit* imu;
  Gyro* gyro;
  Accelerometer* accelerometer;
  GPS* Waistgps;
  GPS* LFootGps;
  GPS* RFootGps;
  Node* Waist;
  Node* SoleLeft;
  Node* SoleRight;

  std::vector<Derivative> dRpy;
  std::vector<Derivative> dJnt;
};

#endif  // WEBOTS_INTERFACE_H
