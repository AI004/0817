/**
 * @file RobotInterfaceImpl.h
 * @brief
 * @version 1.0
 * @date 2023-06-30
 *
 * @copyright Copyright (c) 2023 PND robotics
 *
 */

#ifndef ROBOT_INTERFACE_IMPL_H_
#define ROBOT_INTERFACE_IMPL_H_

#include "../../RobotControl/include/public_parament.h"
#include "JointInterface/include/JointInterface.h"
#include "RobotInterface.h"
#include "abs_encoder.h"
#include "functionIKID_S2P.h"
#include "../../RobotControl/include/public_parament.h"
#include "series_parallel.h"
#include "abs_encoder.h"
#ifdef STATE_ESTIMATE
#  include "StateEstimator/include/StateEstimate.h"
#endif
#ifdef WEBOTS
#  include "WebotsInterface/include/webotsInterface.h"
#endif
typedef enum PndAlgorithmStatusCode {
  PndAlgorithmSuccess = 0,
  PndAlgorithmError = 1,
} PndAlgorithmStatusCode;

enum class AdamStatusCode { UNKNOW = -1, AdamStatusSuccess = 0, AdamStatusFailure = 1 };

class PConfig {
 public:
  static PConfig& getInst() {
    static PConfig instance;
    return instance;
  }
  std::vector<int> jointIdFromNames(const std::vector<std::string>& names);
};
class RobotInterfaceImpl : public RobotInterface {
 public:
  RobotInterfaceImpl();

  ~RobotInterfaceImpl() override;
  void Init() override;
  void GetState(double t, Eigen::VectorXd imuData, RobotData& robot_data) override;
  void SetCommand(RobotData& robot_data) override;
  void DisableAllJoints() override;

 private:
#ifdef WEBOTS
  WebotsRobot humanoid;
  webotState robotStateSim;
  double simTime = 0;
#else
  JointInterface* jointInterface_;
  functionIKID_S2P* funS2P;

  std::vector<std::string> joint_name_;
  static AdamStatusCode ReadAbsEncoder(Eigen::VectorXd& init_pos, Eigen::VectorXd& motor_rotor_abs_pos);
  PndAlgorithmStatusCode PndAbsCheck(double motor_rotor_enc_gear_ratio, double end_enc_gear_ratio,
                                     double min_end_detect_pos, double detect_accuracy, double motor_rotor_enc_noise,
                                     double end_enc_noise, double motor_rotor_enc_offset, double end_enc_offset,
                                     int motor_rotor_enc_dir, int end_enc_dir, double motor_rotor_enc_cur_pos,
                                     double end_enc_cur_pos, double& end_pos);
  double radToDeg(double radians);
  Eigen::Matrix3d RotX(double q);
  Eigen::Matrix3d RotY(double q);
  Eigen::Matrix3d RotZ(double q);
  void EulerZYXToMatrix(Eigen::Matrix3d& R, Eigen::Vector3d euler_a);
  void MatrixToEulerXYZ(Eigen::Matrix3d R, Eigen::Vector3d& euler);
  Eigen::VectorXd absolute_pos_init_;
  Eigen::VectorXd absolute_pos_zero_;
  Eigen::VectorXd absolute_pos_dir_;
  Eigen::VectorXd absolute_pos_gear_ratio_;
  Eigen::VectorXd motor_rotor_abs_pos_;
  Eigen::VectorXd motor_rotor_cur_pos_;
  Eigen::VectorXd p_q_a_;
  Eigen::VectorXd p_q_dot_a_;
  Eigen::VectorXd p_tau_a_;
  Eigen::VectorXd p_q_d_;
  Eigen::VectorXd p_q_dot_d_;
  Eigen::VectorXd p_tau_d_;

  Eigen::VectorXd ankleOrienEst;
  Eigen::VectorXd ankleOmegaEst;
  Eigen::VectorXd ankleTorEst;
  Eigen::VectorXd ankleOrienRef;
  Eigen::VectorXd ankleOmegaRef;
  Eigen::VectorXd ankleTorDes;

  Eigen::VectorXd wristOrien_parallet;
  Eigen::VectorXd wristOmega_parallet;
  Eigen::VectorXd wristTor_parallet;
  Eigen::VectorXd wristOrien_series;
  Eigen::VectorXd wristOmega_series;
  Eigen::VectorXd wristTor_series;

  Eigen::VectorXd joint_dir_;
  Eigen::VectorXd joint_gear_ratio_;
#endif

#ifdef STATE_ESTIMATE
  StateEstimate* stateEstimate_;
#endif

  Eigen::VectorXd joint_Kp_;  // = Eigen::VectorXd::Zero(15);
  Eigen::VectorXd joint_Kd_;  // = Eigen::VectorXd::Zero(15);

  int joint_num_ = actor_num;

#ifdef parallel_data
  std::ofstream fout_data;
  Eigen::VectorXd dataL = Eigen::VectorXd::Zero(200);
#endif
};

#endif  // ROBOT_INTERFACE_IMPL_H_
