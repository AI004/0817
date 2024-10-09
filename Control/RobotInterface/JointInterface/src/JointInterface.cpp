#include "../include/JointInterface.h"
#include "../../../RobotControl/include/public_parament.h"

#include <chrono>
#include <iostream>
#include <thread>
#include "../../../../RobotControl/include/public_parament.h"
JointInterface::JointInterface() : joint_num_(actor_num) { joint_error_ = false; }

JointInterface::~JointInterface() = default;

void JointInterface::Init(Eigen::VectorXd absolute_pos, Eigen::VectorXd Kp,
                          Eigen::VectorXd Kd) {
  int i;
  // size init
  ip_list_.resize(joint_num_);
  absolute_pos_.resize(joint_num_);
  joint_gear_ratio_.resize(joint_num_);
  joint_dir_.resize(joint_num_);
  c_t_scale_.resize(joint_num_);
  linear_count_.resize(joint_num_);
  control_config_.resize(joint_num_);
  enable_status_.resize(joint_num_);
  control_command_.resize(joint_num_);
  lose_error_count_.resize(joint_num_);

  // data
  q_a_.resize(joint_num_);
  qd_a_.resize(joint_num_);
  current_a_.resize(joint_num_);
  torque_a_.resize(joint_num_);

  // init value
  absolute_pos_ = absolute_pos;
  if(adam_type==ADAM_TYPE::AdamLite){
    joint_dir_ << -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 
                  -1, 1, 1,-1,
                  1, 1, 1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51,
                        51, 51, 51, 51;
    c_t_scale_ << 0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592, 
                  0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592,
                  0.074, 0.074, 0.074, 
                  0.0592, 0.0592, 0.063, 0.063, 
                  0.0592, 0.0592, 0.063, 0.063;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    joint_dir_ << -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 
                  -1, 1, 1,-1, 1, -1, 1, 1,
                  1, 1, 1, 1, 1, 1, -1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51, 51;
    c_t_scale_ << 0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592, 
                  0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592,
                  0.074, 0.074, 0.074, 
                  0.0592, 0.0592, 0.063, 0.063, 0.063, 0.063, 0.063, 0.063,
                  0.0592, 0.0592, 0.063, 0.063, 0.063, 0.063, 0.063, 0.063;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    joint_dir_ << -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 
                  -1, 1, 1,-1,
                  1, 1, 1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51,
                        51, 51, 51, 51;
    c_t_scale_ << 0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592, 
                  0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592,
                  0.074, 0.074, 0.074, 
                  0.0592, 0.0592, 0.063, 0.063, 
                  0.0592, 0.0592, 0.063, 0.063;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    joint_dir_ << -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 
                  -1, 1, 1,-1, 1, -1, 1,
                  1, 1, 1, 1, 1, 1, -1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51;
    c_t_scale_ << 0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592, 
                  0.227, 0.136, 0.074, 0.227, 0.0592, 0.0592,
                  0.074, 0.074, 0.074, 
                  0.0592, 0.0592, 0.063, 0.063, 0.063, 0.063, 0.063, 
                  0.0592, 0.0592, 0.063, 0.063, 0.063, 0.063, 0.063;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }
  
  // harmonic
  // joint_gear_ratio_ << 31, 51, 7, 7, 51, 51,
  //     31, 51, 7, 7, 51, 51;
  //     // 51, 51, 51,
  //     // 51, 51, 51, 51,
  //     // 51, 51, 51, 51;
  // c_t_scale_ << 0.1089, 0.054, 0.24, 0.24, 0.0493, 0.0493,
  //     0.1089, 0.054, 0.24, 0.24, 0.0493, 0.0493;
  //     // 0.054, 0.054, 0.054,
  //     // 0.0493, 0.0493, 0.0493, 0.0493,
  //     // 0.0493, 0.0493, 0.0493, 0.0493;

  for (int i = 0; i < joint_num_; i++) {
    linear_count_[i] = 0.0;
    control_config_[i] = new MotionControllerConfig();
    // enable motor
    enable_status_[i] = 1;
    // control command init
    control_command_[i].pos = 0.0;
    control_command_[i].vel_ff = 0.0;
    control_command_[i].torque_ff = 0.0;
  }

  // joint ip
  i = 0;
  ip_list_[i] = "10.10.10.70";i++;
  ip_list_[i] = "10.10.10.71";i++;
  ip_list_[i] = "10.10.10.72";i++;
  ip_list_[i] = "10.10.10.73";i++;
  ip_list_[i] = "10.10.10.74";i++;
  ip_list_[i] = "10.10.10.75";i++;

  ip_list_[i] = "10.10.10.50";i++;
  ip_list_[i] = "10.10.10.51";i++;
  ip_list_[i] = "10.10.10.52";i++;
  ip_list_[i] = "10.10.10.53";i++;
  ip_list_[i] = "10.10.10.54";i++;
  ip_list_[i] = "10.10.10.55";i++;

  ip_list_[i] = "10.10.10.90";i++;
  ip_list_[i] = "10.10.10.91";i++;
  ip_list_[i] = "10.10.10.92";i++;

  ip_list_[i] = "10.10.10.10";i++;
  ip_list_[i] = "10.10.10.11";i++;
  ip_list_[i] = "10.10.10.12";i++;
  ip_list_[i] = "10.10.10.13";i++;

  if(adam_type==ADAM_TYPE::AdamLite){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    ip_list_[i] = "10.10.10.14";i++;
    ip_list_[i] = "10.10.10.15";i++;
    ip_list_[i] = "10.10.10.16";i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    ip_list_[i] = "10.10.10.14";i++;
    ip_list_[i] = "10.10.10.15";i++;
    ip_list_[i] = "10.10.10.16";i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }

  ip_list_[i] = "10.10.10.30";i++;
  ip_list_[i] = "10.10.10.31";i++;
  ip_list_[i] = "10.10.10.32";i++;
  ip_list_[i] = "10.10.10.33";i++;

  if(adam_type==ADAM_TYPE::AdamLite){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    ip_list_[i] = "10.10.10.34";i++;
    ip_list_[i] = "10.10.10.35";i++;
    ip_list_[i] = "10.10.10.36";i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    ip_list_[i] = "10.10.10.34";i++;
    ip_list_[i] = "10.10.10.35";i++;
    ip_list_[i] = "10.10.10.36";i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }

  for (int i = 0; i < joint_num_; i++) {
    joint_ip_index_.insert(std::pair<std::string, int>(ip_list_[i], i));
  }

  for (int i = 0; i < joint_num_; i++) {
    control_config_[i]->pos_gain = Kp[i];
    control_config_[i]->vel_gain = Kd[i];
    control_config_[i]->vel_integrator_gain = 0.0;
    control_config_[i]->vel_limit = 100.0;
    control_config_[i]->vel_limit_tolerance = 100.0;
    if (i == 3 || i == 9) {
      control_config_[i]->vel_limit = 25.0;
    }
  }

  // communication error
  for (int i = 0; i < joint_num_; i++) {
    lose_error_count_[i] = 0;
  }

  // Try and get the requested group.
  std::string str("10.10.10.255");
  Pnd::Lookup lookup(&str);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  lookup.setLookupFrequencyHz(0);
  // group = lookup.getGroupFromFamily("Default");
  std::vector<std::string> ctrl_ips;
  if(adam_type==ADAM_TYPE::AdamLite){
    ctrl_ips= {"10.10.10.10", "10.10.10.11", "10.10.10.12", "10.10.10.13",
                "10.10.10.30", "10.10.10.31", "10.10.10.32", "10.10.10.33",
                "10.10.10.50", "10.10.10.51", "10.10.10.52", "10.10.10.53", "10.10.10.54", "10.10.10.55",
                "10.10.10.70", "10.10.10.71", "10.10.10.72", "10.10.10.73", "10.10.10.74", "10.10.10.75",
                "10.10.10.90", "10.10.10.91", "10.10.10.92"};
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    ctrl_ips= {"10.10.10.10", "10.10.10.11", "10.10.10.12", "10.10.10.13", "10.10.10.14", "10.10.10.15", "10.10.10.16",
                "10.10.10.30", "10.10.10.31", "10.10.10.32", "10.10.10.33", "10.10.10.34", "10.10.10.35", "10.10.10.36", 
                "10.10.10.50", "10.10.10.51", "10.10.10.52", "10.10.10.53", "10.10.10.54", "10.10.10.55",
                "10.10.10.70", "10.10.10.71", "10.10.10.72", "10.10.10.73", "10.10.10.74", "10.10.10.75",
                "10.10.10.90", "10.10.10.91", "10.10.10.92"};
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    ctrl_ips= {"10.10.10.10", "10.10.10.11", "10.10.10.12", "10.10.10.13",
                "10.10.10.30", "10.10.10.31", "10.10.10.32", "10.10.10.33",
                "10.10.10.50", "10.10.10.51", "10.10.10.52", "10.10.10.53", "10.10.10.54", "10.10.10.55",
                "10.10.10.70", "10.10.10.71", "10.10.10.72", "10.10.10.73", "10.10.10.74", "10.10.10.75",
                "10.10.10.90", "10.10.10.91", "10.10.10.92"};
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    ctrl_ips= {"10.10.10.10", "10.10.10.11", "10.10.10.12", "10.10.10.13", "10.10.10.14", "10.10.10.15", "10.10.10.16",
                "10.10.10.30", "10.10.10.31", "10.10.10.32", "10.10.10.33", "10.10.10.34", "10.10.10.35", "10.10.10.36", 
                "10.10.10.50", "10.10.10.51", "10.10.10.52", "10.10.10.53", "10.10.10.54", "10.10.10.55",
                "10.10.10.70", "10.10.10.71", "10.10.10.72", "10.10.10.73", "10.10.10.74", "10.10.10.75",
                "10.10.10.90", "10.10.10.91", "10.10.10.92"};
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }
  
  group = lookup.getGroupFromIps(ctrl_ips);

  if (!group) {
    std::cout << "No group found!" << std::endl;
    return;
  }
  assert(group->size() == joint_num_);
  std::cout << "group size: " << group->size() << std::endl;

  // set error level
  pndSetLogLevel("ERROR", "ERROR");
  feedback = new Pnd::GroupFeedback(group->size());
  group_command = new Pnd::GroupCommand(group->size());

  // set linear count
  std::vector<float> linear_count(joint_num_);
  for (int i = 0; i < joint_num_; i++) {
    linear_count_[i] =
        absolute_pos_(i) * joint_dir_[i] * joint_gear_ratio_[i] / (2.0 * M_PI);
  }

  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    linear_count[count] = linear_count_[it->second];
    count++;
  }
  group_command->resetLinearCount(linear_count);
  group->sendCommand(*group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "set pndrive linear_count succeed" << std::endl;

  // set motor config
  std::vector<MotionControllerConfig *> control_config(joint_num_);
  count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    control_config[count] = control_config_[it->second];
    count++;
  }
  group_command->setMotionCtrlConfig(control_config);
  group->sendCommand(*group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Set pndrive motionCtrlConfig succeed" << std::endl;

  // get motor config
  group->sendFeedbackRequest();
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  group->getNextFeedback(*feedback, 20);

  // start enabling Devices
  for (int i = 0; i < group->size(); ++i) {
    enable_status_[i] = 1;
  }
  group_command->enable(enable_status_);
  group->sendCommand(*group_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Enable pndrive succeed" << std::endl;
}

void JointInterface::SetCommand(Eigen::VectorXd pos_cmd,
                                Eigen::VectorXd vel_cmd,
                                Eigen::VectorXd tor_cmd) {
  //
  pos_cmd = pos_cmd - absolute_pos_;
  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    control_command_[count].pos = pos_cmd[it->second] * joint_dir_[it->second] *
                                  joint_gear_ratio_[it->second] / (2.0 * M_PI);
    control_command_[count].vel_ff =
        vel_cmd[it->second] * joint_dir_[it->second] *
        joint_gear_ratio_[it->second] / (2.0 * M_PI);
    control_command_[count].torque_ff =
        tor_cmd[it->second] * joint_dir_[it->second] /
        (joint_gear_ratio_[it->second] * c_t_scale_[it->second]);
    count++;
  }
  group_command->setInputPositionPt(control_command_);
  group->sendCommand(*group_command);
}

void JointInterface::GetState(Eigen::VectorXd &pos, Eigen::VectorXd &vel,
                              Eigen::VectorXd &tor, double &communication_time) {
  group->getNextFeedback(*feedback, 3);

  std::map<std::string, int>::iterator it;
  int count = 0;
  for (it = joint_ip_index_.begin(); it != joint_ip_index_.end(); it++) {
    // error code
    if ((*feedback)[count]->error_code != 0) {
      std::cout << "Joint [" << it->second
                << "] error code: " << (*feedback)[count]->error_code
                << std::endl;
      joint_error_ = true;
    }
    // lose communication
    if ((*feedback)[count]->position ==
        std::numeric_limits<float>::quiet_NaN()) {
      lose_error_count_[it->second]++;
    } else {
      lose_error_count_[it->second] = 0;
    }
    if (lose_error_count_[it->second] == tolerance_count_) {
      std::vector<bool> status(joint_num_, 0);
      group_command->RcuPower(status);
      group->sendCommand(*group_command);
      std::cout << "Joint [" << it->second
                << "] lose communication, ctrlBox is disabled !" << std::endl;
      joint_error_ = true;
    }
    q_a_(it->second) = (*feedback)[count]->position * joint_dir_[it->second] *
                       2.0 * M_PI / joint_gear_ratio_[it->second];
    qd_a_(it->second) = (*feedback)[count]->velocity * joint_dir_[it->second] *
                        2.0 * M_PI / joint_gear_ratio_[it->second];
    current_a_(it->second) =
        (*feedback)[count]->current * joint_dir_[it->second];
    torque_a_(it->second) = current_a_(it->second) * c_t_scale_[it->second] *
                            joint_gear_ratio_[it->second];
    count++;
  }
  q_a_ = q_a_ + absolute_pos_;
  pos = q_a_;
  vel = qd_a_;
  tor = torque_a_;
  communication_time = feedback->Duration();
}

void JointInterface::Disable() {
  for (int i = 0; i < group->size(); ++i) {
    enable_status_[i] = 0;
  }
  group_command->enable(enable_status_);
  group->sendCommand(*group_command);
  for (int i = 0; i < joint_num_; i++) {
    std::cout << "Joint [" << i << "] disabled!" << std::endl;
  }
}
