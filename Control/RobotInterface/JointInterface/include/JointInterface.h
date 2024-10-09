
/**
 * @file JointInterface.h
 * @brief
 * @version 0.1
 * @date 2023-06-20
 *
 * @copyright Copyright (c) 2023 PND robotics
 *
 */
#ifndef JOINT_INTERFACE_H_
#define JOINT_INTERFACE_H_
#include <Eigen/Dense>
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QStringList>
#include <fstream>
#include <map>

#include "aios.h"
#include "groupCommand.hpp"
#include "groupFeedback.hpp"
#include "lookup.hpp"

class JointInterface {
public:
  //
  explicit JointInterface();
  ~JointInterface();

  void Init(Eigen::VectorXd absolute_pos, Eigen::VectorXd Kp,
            Eigen::VectorXd Kd);
  void SetCommand(Eigen::VectorXd pos_cmd, Eigen::VectorXd vel_cmd,
                  Eigen::VectorXd tor_cmd);
  void GetState(Eigen::VectorXd &pos, Eigen::VectorXd &vel,
                Eigen::VectorXd &tor, double &communication_time);
  // Disable
  void Disable();

  bool joint_error_;

private:
  // group
  std::shared_ptr<Pnd::Group> group;
  // enable_group
  std::shared_ptr<Pnd::Group> enable_group;
  //
  Pnd::GroupFeedback *feedback{};
  //
  Pnd::GroupCommand *group_command{};
  Pnd::GroupCommand *enable_group_cmd{};
  // joint num
  int joint_num_;
  // iplist
  std::vector<std::string> ip_list_;
  std::map<std::string, int> joint_ip_index_;
  //-----------absolute encoder--------------//
  Eigen::VectorXd absolute_pos_;
  //-----------------------------------------//
  // gear ratio
  Eigen::VectorXd joint_gear_ratio_;
  // joint direction
  Eigen::VectorXi joint_dir_;
  // joint current-torque scale
  Eigen::VectorXd c_t_scale_;
  // set linear count, same sort with group
  std::vector<float> linear_count_;
  // set motor config
  std::vector<MotionControllerConfig *> control_config_;
  // start enabling Devices
  std::vector<float> enable_status_;
  // start enabling Devices
  std::vector<float> enable_group_status_;
  // control command
  std::vector<PosPtInfo> control_command_;

  // data
  Eigen::VectorXd q_a_;
  Eigen::VectorXd qd_a_;
  Eigen::VectorXd current_a_;
  Eigen::VectorXd torque_a_;

  // communication error
  std::vector<int> lose_error_count_;
  int tolerance_count_ = 3; // default 1;
};
#endif // JOINTINTERFACE_H_
