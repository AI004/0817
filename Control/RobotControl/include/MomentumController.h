#ifndef MOMENTUMCONTROLLER_H
#define MOMENTUMCONTROLLER_H

#include "../../StateEstimation/include/WalkStateEstimate.h"
#include "Robot_Data.h"

class MomentumController {
 public:
  MomentumController();

  void setInputData(Eigen::MatrixXd input_a,
                    Eigen::MatrixXd input_d,
                    const Eigen::MatrixXd& para,
                    Eigen::MatrixXd IG);

  void controllerRun(Robot_Data* robotdata);

  void getOutputData(Eigen::MatrixXd& output);

 private:
  // input data
  Eigen::MatrixXd input_data_a;
  Eigen::MatrixXd input_data_d;
  // output data
  Eigen::MatrixXd output_data;
  // controller parameters such as kp ki kd,
  Eigen::Matrix<double, Eigen::Dynamic, 1> para_;
  Eigen::MatrixXd IG_;

  // stand CoM
  Eigen::VectorXd standCoM = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ZMPDes = Eigen::VectorXd::Zero(3);

  bool inPrePushPhase = true;
  bool inReflexPhase = false;
  bool inRecoveryPhase = false;
  bool inWalkRecoveryPhase = false;

  double prePushTimer = 0.0;
  double reflexTimer = 0.0;
  double recoveryTimer = 0.0;
  double walkRecoveryTimer = 0.0;
  double preRecoveryPose = 0.0;
  Eigen::Vector2d ZMPThreshold = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd q_factor_init = Eigen::VectorXd::Zero(23);
  Eigen::VectorXd q_dot_factor_init = Eigen::VectorXd::Zero(23);

  Eigen::VectorXd q_factor_prePush = Eigen::VectorXd::Zero(23);
  Eigen::VectorXd q_dot_factor_prePush = Eigen::VectorXd::Zero(23);

  Eigen::VectorXd q_factor_reflex = Eigen::VectorXd::Zero(23);
  Eigen::VectorXd q_dot_factor_reflex = Eigen::VectorXd::Zero(23);

  Eigen::VectorXd q_factor_recovery = Eigen::VectorXd::Zero(23);
  Eigen::VectorXd q_dot_factor_recovery = Eigen::VectorXd::Zero(23);
};

#endif  // MOMENTUMCONTROLLER_H
