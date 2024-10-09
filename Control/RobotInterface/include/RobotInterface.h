/**
 * @file RobotInterface.h
 * @brief
 * @version 1.0
 * @date 2023-06-30
 *
 * @copyright Copyright (c) 2023 PND robotics
 *
 */

#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include <Eigen/Dense>
#include "../../RobotControl/include/public_parament.h"
// #define WEBOTS
// #define parallel_data
// #define STATE_ESTIMATE

struct RobotData {
  // floating base pos rpy + joints
  Eigen::VectorXd q_a_ = Eigen::VectorXd::Zero(generalized_coordinates);
  Eigen::VectorXd q_dot_a_ = Eigen::VectorXd::Zero(generalized_coordinates);
  Eigen::VectorXd tau_a_ = Eigen::VectorXd::Zero(generalized_coordinates);

  Eigen::VectorXd q_d_ = Eigen::VectorXd::Zero(generalized_coordinates);
  Eigen::VectorXd q_dot_d_ = Eigen::VectorXd::Zero(generalized_coordinates);
  Eigen::VectorXd tau_d_ = Eigen::VectorXd::Zero(generalized_coordinates);

  bool pos_mode_ = true;
  int communication_time = 0;
};

class RobotInterface {
public:
  RobotInterface() = default;
  virtual ~RobotInterface() = default;
  virtual void Init() = 0;
  virtual void GetState(double t, Eigen::VectorXd imu_data,
                        RobotData &robot_data) = 0;
  virtual void SetCommand(RobotData &robot_data) = 0;
  virtual void DisableAllJoints() = 0;

  Eigen::VectorXd imu_data_ = Eigen::VectorXd::Zero(9);
  bool error_state_;
};

RobotInterface *get_robot_interface();

#endif // ROBOT_INTERFACE_H_
