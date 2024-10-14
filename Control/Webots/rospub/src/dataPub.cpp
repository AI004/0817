#include "dataPub.h"

DataPub::DataPub() {
  joint_desired_.name = joint_names_;
  joint_measured_.name = joint_names_;
  joint_command_.name = joint_names_;

  joint_desired_.position.resize(DuckDuck_joints_num);
  joint_desired_.velocity.resize(DuckDuck_joints_num);
  joint_desired_.effort.resize(DuckDuck_joints_num);

  joint_measured_.position.resize(DuckDuck_joints_num);
  joint_measured_.velocity.resize(DuckDuck_joints_num);
  joint_measured_.effort.resize(DuckDuck_joints_num);

  joint_command_.position.resize(DuckDuck_joints_num);
  joint_command_.velocity.resize(DuckDuck_joints_num);
  joint_command_.effort.resize(DuckDuck_joints_num);

  floatingBase_c = Eigen::VectorXd::Zero(7);
  floatingBase_a = Eigen::VectorXd::Zero(7);
  BodyPose_d = Eigen::VectorXd::Zero(6);
  BodyPose_a = Eigen::VectorXd::Zero(6);
  ComPose_a = Eigen::VectorXd::Zero(6);
  ChestPose_a = Eigen::VectorXd::Zero(6);
  LfootPose_d = Eigen::VectorXd::Zero(6);
  LfootPose_a = Eigen::VectorXd::Zero(6);
  RfootPose_d = Eigen::VectorXd::Zero(6);
  RfootPose_a = Eigen::VectorXd::Zero(6);

  observe_data_.position.resize(20);
  observe_data_.velocity.resize(20);
  observe_data_.effort.resize(20);
};
DataPub::~DataPub() {};
void DataPub::init(ros::NodeHandle& nh) {
  // 初始化发布者
  joint_desired_pub_ = nh.advertise<sensor_msgs::JointState>("/DuckDuck/joint_desired", 10);
  joint_measured_pub_ = nh.advertise<sensor_msgs::JointState>("/DuckDuck/joint_measured", 10);

  floatingBase_desired_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/floatingBase_desired", 10);
  floatingBase_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/floatingBase_measured", 10);
  floatingBaseDot_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/floatingBaseDot_measured", 10);

  body_desired_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/body_desired", 10);
  body_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/body_measured", 10);
  com_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/com_measured", 10);
  chest_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/chest_measured", 10);
  lfoot_desired_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/lfoot_desired", 10);
  lfoot_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/lfoot_measured", 10);
  rfoot_desired_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/rfoot_desired", 10);
  rfoot_measured_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/DuckDuck/rfoot_measured", 10);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/DuckDuck/imu_data", 10);

  observe_data_pub_ = nh.advertise<sensor_msgs::JointState>("/DuckDuck/observeData", 10);
}

// lhj:msg发布
int DataPub::update(const double time, const Robot_Data& robot_data) {
  time_ = time;
  robot_data_ = robot_data;
  this->jointStatesPub();
  this->robotStatesPub();
  this->imuMsgPub();
  this->observeDataPub();

  return 0;
}
// 对于robot_data_中的q_a、q_d、q_c，只有_a即为measured，_c即为desired
void DataPub::jointStatesPub() {
  Eigen::Map<Eigen::VectorXd> jointPosCmd(joint_desired_.position.data(), DuckDuck_joints_num);
  Eigen::Map<Eigen::VectorXd> jointVelCmd(joint_desired_.velocity.data(), DuckDuck_joints_num);
  Eigen::Map<Eigen::VectorXd> jointTorCmd(joint_desired_.effort.data(), DuckDuck_joints_num);

  Eigen::Map<Eigen::VectorXd> jointPosMea(joint_measured_.position.data(), DuckDuck_joints_num);
  Eigen::Map<Eigen::VectorXd> jointVelMea(joint_measured_.velocity.data(), DuckDuck_joints_num);
  Eigen::Map<Eigen::VectorXd> jointTorMea(joint_measured_.effort.data(), DuckDuck_joints_num);

  Eigen::Map<Eigen::Vector3d>(&floatingBase_desired_.pose.position.x) = robot_data_.q_c.head(3);
  Eigen::Map<Eigen::Vector3d>(&floatingBase_measured_.pose.position.x) = robot_data_.q_a.head(3);
  Eigen::Map<Eigen::Vector3d>(&floatingBaseDot_measured_.pose.position.x) = robot_data_.q_dot_a.head(3);

  Eigen::Map<Eigen::Vector4d>(&floatingBase_desired_.pose.orientation.x) = robot_data_.q_c.segment(3, 4);
  Eigen::Map<Eigen::Vector4d>(&floatingBase_measured_.pose.orientation.x) = robot_data_.q_a.segment(3, 4);
  Eigen::Map<Eigen::Vector4d>(&floatingBaseDot_measured_.pose.orientation.x) = robot_data_.q_dot_a.segment(3, 4);

  jointPosMea = robot_data_.q_a.segment(6, DuckDuck_joints_num);
  jointVelMea = robot_data_.q_dot_a.segment(6, DuckDuck_joints_num);
  jointTorMea = robot_data_.tau_a.segment(6, DuckDuck_joints_num);

  jointPosCmd = robot_data_.q_c.segment(6, DuckDuck_joints_num);
  jointVelCmd = robot_data_.q_dot_c.segment(6, DuckDuck_joints_num);
  jointTorCmd = robot_data_.tau_c.segment(6, DuckDuck_joints_num);

  floatingBase_a = robot_data_.q_a.head(7);
  floatingBase_c = robot_data_.q_c.head(7);

  joint_desired_.header.stamp = ros::Time(time_);
  joint_measured_.header.stamp = ros::Time(time_);
  joint_desired_pub_.publish(joint_desired_);
  joint_measured_pub_.publish(joint_measured_);

  floatingBase_desired_.header.stamp = ros::Time(time_);
  floatingBase_measured_.header.stamp = ros::Time(time_);
  floatingBaseDot_measured_.header.stamp = ros::Time(time_);
  floatingBase_desired_pub_.publish(floatingBase_desired_);
  floatingBase_measured_pub_.publish(floatingBase_measured_);
  floatingBaseDot_measured_pub_.publish(floatingBaseDot_measured_);
}

void DataPub::robotStatesPub() {
  // body foot
  Eigen::Map<Eigen::Vector3d>(&bodyPose_desired_.pose.position.x) = BodyPose_d.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&rfootPose_desired_.pose.position.x) = RfootPose_d.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&lfootPose_desired_.pose.position.x) = LfootPose_d.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&bodyPose_measured_.pose.position.x) = BodyPose_a.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&comPose_measured_.pose.position.x) = ComPose_a.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&chestPose_measured_.pose.position.x) = ChestPose_a.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&rfootPose_measured_.pose.position.x) = RfootPose_a.segment(3, 3);
  Eigen::Map<Eigen::Vector3d>(&lfootPose_measured_.pose.position.x) = LfootPose_a.segment(3, 3);

  Eigen::Map<Eigen::Vector4d>(&bodyPose_desired_.pose.orientation.x) = BodyPose_d.head(4);
  Eigen::Map<Eigen::Vector4d>(&rfootPose_desired_.pose.orientation.x) = RfootPose_d.head(4);
  Eigen::Map<Eigen::Vector4d>(&lfootPose_desired_.pose.orientation.x) = LfootPose_d.head(4);
  Eigen::Map<Eigen::Vector4d>(&bodyPose_measured_.pose.orientation.x) = BodyPose_a.head(4);
  Eigen::Map<Eigen::Vector4d>(&comPose_measured_.pose.orientation.x) = ComPose_a.head(4);
  Eigen::Map<Eigen::Vector4d>(&chestPose_measured_.pose.orientation.x) = ChestPose_a.head(4);
  Eigen::Map<Eigen::Vector4d>(&rfootPose_measured_.pose.orientation.x) = RfootPose_a.head(4);
  Eigen::Map<Eigen::Vector4d>(&lfootPose_measured_.pose.orientation.x) = LfootPose_a.head(4);

  BodyPose_d.head(6) = robot_data_.task_card_set[robot_data_.body_task_id]->X_d.block(0, 0, 1, 6).transpose();
  RfootPose_d.head(6) = robot_data_.task_card_set[robot_data_.right_foot_id]->X_d.block(0, 0, 1, 6).transpose();
  LfootPose_d.head(6) = robot_data_.task_card_set[robot_data_.left_foot_id]->X_d.block(0, 0, 1, 6).transpose();

  BodyPose_a.head(6) = robot_data_.task_card_set[robot_data_.body_task_id]->X_a.block(0, 0, 1, 6).transpose();
  ComPose_a.head(6) = robot_data_.task_card_set[robot_data_.com_task_id]->X_a.block(0, 0, 1, 6).transpose();
  ChestPose_a.head(6) = robot_data_.task_card_set[robot_data_.chest_task_id]->X_a.block(0, 0, 1, 6).transpose();
  RfootPose_a.head(6) = robot_data_.task_card_set[robot_data_.right_foot_id]->X_a.block(0, 0, 1, 6).transpose();
  LfootPose_a.head(6) = robot_data_.task_card_set[robot_data_.left_foot_id]->X_a.block(0, 0, 1, 6).transpose();

  bodyPose_desired_.header.stamp = ros::Time(time_);
  // bodyPose_desired_.header.stamp = ros::Time::now();
  bodyPose_measured_.header.stamp = ros::Time(time_);
  comPose_measured_.header.stamp = ros::Time(time_);
  chestPose_measured_.header.stamp = ros::Time(time_);
  lfootPose_desired_.header.stamp = ros::Time(time_);
  lfootPose_measured_.header.stamp = ros::Time(time_);
  rfootPose_desired_.header.stamp = ros::Time(time_);
  rfootPose_measured_.header.stamp = ros::Time(time_);

  body_desired_pub_.publish(bodyPose_desired_);
  body_measured_pub_.publish(bodyPose_measured_);
  com_measured_pub_.publish(comPose_measured_);
  chest_measured_pub_.publish(chestPose_measured_);
  lfoot_desired_pub_.publish(lfootPose_desired_);
  lfoot_measured_pub_.publish(lfootPose_measured_);
  rfoot_desired_pub_.publish(rfootPose_desired_);
  rfoot_measured_pub_.publish(rfootPose_measured_);
}

void DataPub::imuMsgPub() {
  imu_msg_.header.stamp = ros::Time(time_);
  imu_msg_.orientation.x = robot_data_.imu9D(0);
  imu_msg_.orientation.y = robot_data_.imu9D(1);
  imu_msg_.orientation.z = robot_data_.imu9D(2);
  imu_msg_.orientation.w = 1.0;
  imu_msg_.angular_velocity.x = robot_data_.imu9D(3);
  imu_msg_.angular_velocity.y = robot_data_.imu9D(4);
  imu_msg_.angular_velocity.z = robot_data_.imu9D(5);
  imu_msg_.linear_acceleration.x = robot_data_.imu9D(6);
  imu_msg_.linear_acceleration.y = robot_data_.imu9D(7);
  imu_msg_.linear_acceleration.z = robot_data_.imu9D(8);
  imu_pub_.publish(imu_msg_);
}

void DataPub::observeDataPub() {
  Eigen::Map<Eigen::VectorXd> obsData(observe_data_.position.data(), 20);
  obsData.head(12) = robot_data_.observeData.head(12);
  observe_data_.header.stamp = ros::Time(time_);
  observe_data_pub_.publish(observe_data_);
}