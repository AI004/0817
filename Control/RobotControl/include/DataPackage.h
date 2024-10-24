#ifndef DATAPACKAGE_H
#define DATAPACKAGE_H
#include <Eigen/Dense>
#include <vector>
class DataPackage {
 public:
  // construct function
  DataPackage();
  // data
  // dimension of the robot
  int dim;
  // control cycle time
  double dt;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_d_hands;
  /**
   * @brief q_a
   *     joint position sense
   * qa = [q_float, q_joint] for floating base robot and mobile robot
   * qa = [q_joint] for fixed robot
   */
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_a;
  bool hands_motion_flag{false};
  // joint velocity sense
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot_a;
  // joint acceleration command
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_ddot_a;
  // joint torque sense
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_a;
  // FT_sensor rows: fx fy fz mx my mz; columns: FTsensor1, FT_sensor2,...;
  // units: N, Nm
  Eigen::Matrix<double, 6, Eigen::Dynamic> ft_sensor;
  // Imu sensor rows: x, y, z, theta_x, theta_y, theta_z; column: imu1,
  // imu2,...; units: m, rad
  Eigen::Matrix<double, 18, Eigen::Dynamic> imu_sensor;
  /**
   * @brief task_desired_value
   * For example: task_1 dim = 1, task_2 dim=2, task_3 dim=1,...
   * task_desired_value = [task_1_x1_d    task_2_x1_d     task_2_x2_d
   * task_3_x1_d; task_1_x1_dot   task_2_x1_dot   task_2_x2_dot   task_3_x1_dot;
   *                      task_1_x1_ddot  task_2_x1_ddot  task_2_x2_ddot
   * task_3_x1_ddot; task_1_x1_f     task_2_x1_f     task_2_x2_f task_3_x1_f];
   */
  //    Eigen::Matrix<double, 4, Eigen::Dynamic> task_desired_value;

  std::vector<Eigen::MatrixXd> task_desired_value;
  // contact state for every type task
  std::vector<bool> task_desired_contact_state;
  // joint position command
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_c;
  // joint velocity command
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_dot_c;
  // joint acceleration command
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_ddot_c;
  // joint torque command
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_c;
  // contact force
  Eigen::Matrix<double, Eigen::Dynamic, 1> contact_force;

  // temp value
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> var_temp;

  //
  Eigen::VectorXd q_factor;
  Eigen::VectorXd q_dot_factor;

  // imu relative vars
  Eigen::Matrix3d xyz_R_init;
  Eigen::Matrix3d NED_R_YPR0;
  Eigen::Matrix3d NED_R_YPRa;
  bool imu_init_flag = 0;

  // head
  Eigen::Vector3d q_head_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_head_c = Eigen::Vector3d::Zero();
  // fsmstate : 2023 0223
  std::string fsmstate;

  // log
  std::vector<std::string> log_buffer;

  /**
   * @brief add log string
   *
   */
  int addlog(std::string log_);
  /**
   * @brief clear all log
   * clear log_buffer
   */
  int clearlog();

  // limits : if ub = lb = 0 -> no bounds
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_lbound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_ubound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> qd_bound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_bound;

  Eigen::VectorXd dataL;
};

#endif  // DATAPACKAGE_H
