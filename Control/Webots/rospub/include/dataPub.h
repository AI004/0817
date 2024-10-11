#ifndef DATA_PUB_H
#define DATA_PUB_H

#include <Eigen/Dense>
#include <vector>
#include "../../../../RobotControl/include/Robot_Controller.h"
#include "../../../../RobotControl/include/Robot_Data.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

class DataPub {
 public:
  DataPub();
  ~DataPub();
  void init(ros::NodeHandle& nh);
  int update(const double time, const Robot_Data& robot_data);

 private:
  void jointStatesPub();
  void robotStatesPub();
  void imuMsgPub();
  void observeDataPub();

  std::vector<std::string> joint_names_{
      "hipPitch_Left",      "hipRoll_Left",      "hipYaw_Left",      "kneePitch_Left", "anklePitch_Left",
      "ankleRoll_Left",     "hipPitch_Right",    "hipRoll_Right",    "hipYaw_Right",   "kneePitch_Right",
      "anklePitch_Right",   "ankleRoll_Right",   "waistRoll",        "waistPitch",     "waistYaw",
      "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",     "shouldPitch_Right",
      "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right"};
  // joint states
  ros::Publisher joint_desired_pub_, joint_measured_pub_, joint_command_pub_;
  sensor_msgs::JointState joint_desired_, joint_measured_, joint_command_;
  // robot states
  ros::Publisher body_desired_pub_, body_measured_pub_, lfoot_desired_pub_, lfoot_measured_pub_, rfoot_desired_pub_,
      rfoot_measured_pub_;
  geometry_msgs::PoseStamped bodyPose_desired_, bodyPose_measured_, lfootPose_desired_, lfootPose_measured_,
      rfootPose_desired_, rfootPose_measured_;

  ros::Publisher chest_measured_pub_, com_measured_pub_, com_desired_pub_;
  geometry_msgs::PoseStamped comPose_desired_, comPose_measured_, chestPose_measured_;
  // floating base
  ros::Publisher floatingBase_desired_pub_, floatingBase_measured_pub_, floatingBaseDot_measured_pub_;
  geometry_msgs::PoseStamped floatingBase_desired_, floatingBase_measured_, floatingBase_command_,
      floatingBaseDot_measured_;
  // imu
  ros::Publisher imu_pub_;
  sensor_msgs::Imu imu_msg_;

  // temp observe data
  ros ::Publisher observe_data_pub_;
  sensor_msgs::JointState observe_data_;

  double time_;
  Robot_Data robot_data_;
  DataPackage* data_;
  Eigen::VectorXd floatingBase_d, floatingBase_a, floatingBase_c;
  Eigen::VectorXd LfootPos_d, LfootOri_d, RfootPos_d, RfootOri_d, BodyPos_d, BodyOri_d;
  Eigen::VectorXd LfootPose_d, RfootPose_d, BodyPose_d, LfootPose_a, RfootPose_a, BodyPose_a;
  Eigen::VectorXd ComPose_a, ChestPose_a, ComPose_d;
  Eigen::VectorXd LfootPos_c, LfootOri_c, RfootPos_c, RfootOri_c, BodyPos_c, BodyOri_c;
  Eigen::VectorXd ArmJoints_d, ArmJoints_a, ArmJoints_c;
};

#endif  // DATA_PUB_H