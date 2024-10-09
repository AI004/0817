#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H
#include "Sensor.h"
#include "Task.h"
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <vector>
#include "public_parament.h"
const double _INF = 1.0e20;
/**
 * @brief The robot_type enum
 */
enum robot_type {
  Fixed_Base_Open_Chain = 1,
  Float_Base_Open_Chain,
  Mobile_Wheel_Open_Chain
};
/**
 * @brief The Wbc_Solver_type enum
 */
enum Wbc_Solver_type { hqp_kinematics = 1, hqp_dynamics, WBIC, WQP };
/**
 * @brief The Robot_Data class
 */
class Robot_Data {
public:
  // construct function
  Robot_Data();
  ~Robot_Data();
  // robot model
  RigidBodyDynamics::Model *robot_model = nullptr;
  // body id
  std::vector<uint> id_body;
  std::vector<RigidBodyDynamics::Body *> robot_body;
  std::vector<RigidBodyDynamics::Joint *> robot_joint;
  // task card set
  int npriority;
  int ntask;
  std::vector<Task *> task_card_set;
  // sensor set
  std::vector<Sensor *> sensor_set;
  // tau external
  Eigen::VectorXd tau_ext;
  // total dof
  int ndof = 0;
  // wheel dof
  int ndof_wheel = 0;
  double radius_wheel;
  double length_wheel;
  // limits : if ub = lb = 0 -> no bounds
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_lbound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> q_ubound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> qd_bound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_bound;
  // if mobile wheel robot, q = [q_float, q_wheel, other(q_arm...) ]
  // robot current state :
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_a; // current floating base and joints pos
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_dot_a; // current floating base and joints vel
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_ddot_a; // current floating base and joints acc
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      tau_a; // current floating base and joints torque
  // robot desired quantity
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_d; // desired floating base and joints pos
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_dot_d; // desired floating base and joints vel
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_ddot_d; // desired floating base and joints acc
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      tau_d; // desired floating base and joints torque
  // robot command quantity
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_c; // floating base and joints pos command
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_dot_c; // floating base and joints vel command
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      q_ddot_c; // floating base and joints acc command
  Eigen::Matrix<double, Eigen::Dynamic, 1>
      tau_c; // floating base and joints torque command
  // robot system paras
  // control cycle time units: /s
  double dt; // 0.0025
  // robot type
  enum robot_type robottype;
  // solver type
  enum Wbc_Solver_type wbcsolver;
  // desired net contect wrench
  Eigen::Matrix<double, Eigen::Dynamic, 1> net_contact_force;
  // desired contact force : left and right ankle force from Landing_MPC
  Eigen::MatrixXd contactforce;
  // contact state
  Eigen::MatrixXd C;
  //
  int sign;
  // gravity
  Eigen::Matrix<double, 6, 1> G;
  // PD gains
  Eigen::VectorXd q_factor;
  Eigen::VectorXd q_dot_factor;

  int body_task_id;    // pelvis rpy pos task
  int com_task_id;     // COM rpy pos task
  int left_foot_id;    // left foot rpy pos task
  int right_foot_id;   // right foot rpy pos task
  int chest_task_id;   // chest rpy pos task
  int upper_joints_id; // waist and arm joints task
  // wbic qp weight
  // Eigen::MatrixXd Q1; // for GRF
  // Eigen::MatrixXd Q2; // for qb
  // data from jason for Q1 and Q2
  double wq1;
  double wq2;
  double wq3;
  double vq1;
  double vq2;
  double vq3;
  double mq1;
  double mq2;
  double mq3;
  double fq1;
  double fq2;
  double fq3;
  Eigen::MatrixXd mpcforceref;
  // imu init matrix
  Eigen::Matrix3d imu_init;
  // WF1 WF2
  Eigen::MatrixXd WF1; // weight for ||F||
  Eigen::MatrixXd WF2; // weight for ||F - Fref||
  // stance foot index
  int stance_index = 1; // 1: right leg stance; 0: left leg stance
  int touch_index = 1;  // 1: right foot touch; 2: right foot fully touch down;
                        // 3: left foot touch; 4: left foot fully touch down.
  int touch_index_pre = 1;
  int flag_ankle = 0;

  // ground reaction force
  Eigen::VectorXd grf = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd grf_pre = Eigen::VectorXd::Zero(12);

  double MG{560.0};     // whole body M*g
  double time{0.0};     // run time (sec) for current behavior
  double t{0.0};        // passed-time (sec), since last Phase switch
  double t_switch{0.0}; // last TD time
  double s{0.0};        // the time-variant parameter: s = clamp(t/Ts, 0, 1)
  double T{0.4};
  double Td{0.4};
  double Td2{0.3};
  double Td_ratio;
  double Tc{0.02};
  int step{0};
  double sL;
  double sR;
  double sc;
  double sfL;
  double sfR;
  double sfc;
  double t_ftd{0.5};

  double grf_lb{50.0};
  double grf_ub{0.9 * 500};

  double hd;              // pelvis height
  double foot_zMid;       // foot swing height
  double carrybox_poffset_x;// carrybox CoM offset x
  double poffset_x;       // CoM offset x
  double poffset_y;       // CoM offset y
  double poffset_z;       // CoM offset z
  double vy_offset;       // y vel oscillation
  double vy_change_ratio; // y vel oscillation change ratio with avg_vx
  double sigma_ratio_x;   // x vel feedforward ratio
  double sigma_ratio_y;   // y vel feedforward ratio
  double kp_ratio_x;      // x vel feedback ratio
  double kp_ratio_y;      // y vel feedback ratio
  double foot_limit_x;    // x foothold limit
  double foot_limit_yin;  // y foothold in limit
  double foot_limit_yout; // y foothold out limit
  double foot_limit_z;    // z foothold limit

  double roll_weight;  // wbc body roll weight
  double pitch_weight; // wbc body pitch weight
  double px_weight;    // wbc body x weight
  double py_weight;    // wbc body y weight

  // uniPlan
  double T_leg{0.62}; // gait period
  double Tc_leg{0.02};
  int phase =
      1; // 1: right foot stance, 2: right double stance, 3: right flight, 4:
         // left foot stance, 5: left double stance, 6: left flight
  int phase_pre =
      1; // 1: right foot stance, 2: right double stance, 3: right flight, 4:
         // left foot stance, 5: left double stance, 6: left flight
  Eigen::Vector2d t0_leg =
      Eigen::Vector2d::Zero(); // start time of every gait cycle, gait circle
                               // contains stance, swing sequentially, so it's
                               // the time starts swing.
  Eigen::Vector2d offset_leg =
      Eigen::Vector2d::Zero(); // offset of every gait cycle, initial t0_leg =
                               // 0
                               // - offset_leg
  Eigen::Vector2d t_leg =
      Eigen::Vector2d::Zero(); // time in current gait cycle, t_leg = time -
                               // t0_leg
  Eigen::Vector2d tst_leg = Eigen::Vector2d::Zero(); // stance time length
  Eigen::Vector2d tsw_leg = Eigen::Vector2d::Zero(); // swing time length
  Eigen::Vector2d st_leg = Eigen::Vector2d::Ones();  // flag for stance index
  Eigen::Vector2d st_leg_pre =
      Eigen::Vector2d::Ones(); // previous flag for stance index
  Eigen::Vector2d late_leg =
      Eigen::Vector2d::Zero(); // flag for late touch, if late_flag==1, this
                               // leg extend, other swing leg desired position
                               // = current position, if there are legs in
                               // stance phase, decrease hd
  Eigen::Vector2d td_leg = Eigen::Vector2d::Zero(); // flag for touch down
  Eigen::Vector2d s_leg = Eigen::Vector2d::Zero();  // smoothed st_leg
  Eigen::Vector2d lx_leg =
      Eigen::Vector2d::Zero(); // x foothold location relative to waist
  Eigen::Vector2d ly_leg =
      Eigen::Vector2d::Zero(); // y foothold location relative to waist
  Eigen::VectorXd l_leg =
      Eigen::VectorXd::Zero(6); // foothold location relative to waist
  Eigen::VectorXd v0_leg =
      Eigen::VectorXd::Zero(6); // inital velocity for each stance phase
  Eigen::Vector2d vtd_leg =
      Eigen::Vector2d::Zero(); // predicted waist speed at touchdown
  Eigen::Vector2d vtd_filt_leg =
      Eigen::Vector2d::Zero(); // filted predicted waist speed at touchdown
  Eigen::Vector2d ptd_leg =
      Eigen::Vector2d::Zero(); // predicted waist position at touchdown
  double lambda;
  int flag_T = 1;
  Eigen::Vector2d isAnkleTouch = Eigen::Vector2d::Zero();
  Eigen::Vector2d isFootTouch = Eigen::Vector2d::Zero();

  Eigen::VectorXd pFootb_ref = Eigen::VectorXd::Zero(6); // foot reference pos
  Eigen::VectorXd vFootb_ref = Eigen::VectorXd::Zero(6); // foot reference vel
  Eigen::VectorXd pFoot_ref = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vFoot_ref = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pFoot_a = Eigen::VectorXd::Zero(6); // foot act pos
  Eigen::VectorXd vFoot_a = Eigen::VectorXd::Zero(6); // foot act vel
  Eigen::VectorXd rFoot_a = Eigen::VectorXd::Zero(6); // foot act pose
  Eigen::VectorXd pFootb_begin = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vFootb_begin = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pFootb_final = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vFootb_final = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd pTorso_ref = Eigen::VectorXd::Zero(3); // torso reference pos
  Eigen::VectorXd vTorso_ref = Eigen::VectorXd::Zero(3); // torso reference vel
  Eigen::VectorXd pTorso_ref_kin = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_ref_kin = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_begin = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_begin = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_final = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_final = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd rFoot_ref =
      Eigen::MatrixXd::Zero(6, 3);                       // foot reference pose
  Eigen::MatrixXd rTorso_ref = Eigen::Matrix3d::Zero();  // torso reference pose
  Eigen::MatrixXd rFoot_d = Eigen::MatrixXd::Zero(6, 3); // ankle reference pose
  Eigen::MatrixXd rFoot_td = Eigen::MatrixXd::Zero(6, 3);
  Eigen::MatrixXd rFoot = Eigen::MatrixXd::Zero(6, 3);
  Eigen::MatrixXd rFoot_kin_ref = Eigen::MatrixXd::Zero(6, 3);
  Eigen::VectorXd eulerFoot_kin_ref = Eigen::VectorXd::Zero(6);

  double arm_style{0.0};   // walk run arm style
  double arm_style_d{0.0}; // 0 for walk, 1 for run

  int gaitMode = 0; // 0 for walk, 1 for run
  bool onWalk = true;
  bool onRun = false;
  // uniPlan

  Eigen::VectorXd imuAcc = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd pFoot_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pFootb_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pFootb_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pFootb_tgt2 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pFootb_ini = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pFootb_end = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vFoot_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vFootb_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vFootb_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vFootb_ini = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vFootb_end = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd pTorso_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_kin_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_ini = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pTorso_end = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_td = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_kin_tgt = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_ini = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vTorso_end = Eigen::VectorXd::Zero(3);

  Eigen::Matrix3d rTorso_d = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rTorso = Eigen::Matrix3d::Identity();
  Eigen::Vector3d eulerTorso_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d eulerTorso_ref = Eigen::Vector3d::Zero();
  double coriolis_force = 0.0;
  double coriolis_force_filt = 0.0;
  Eigen::Matrix3d rTorso_td = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rTorso_tgt = Eigen::Matrix3d::Identity();

  // Eigen::Matrix3d rFoot_td = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rFoot_tgt = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d rFoot_l = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rFoot_r = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rFoot_ltd = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rFoot_rtd = Eigen::Matrix3d::Identity();

  Eigen::VectorXd tau_lb =
      Eigen::VectorXd::Zero(actor_num); // wbc joint torque lower bound
  Eigen::VectorXd tau_ub =
      Eigen::VectorXd::Zero(actor_num); // wbc joint torque upper bound
  Eigen::VectorXd GRF_ub =
      Eigen::VectorXd::Zero(12); // wbc ground reaction force lower bound
  Eigen::VectorXd GRF_lb =
      Eigen::VectorXd::Zero(12); // wbc ground reaction force upper bound

  //
  Eigen::VectorXd temp = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd temp2 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qdotcmd_temp = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd temp_kalman = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd temp_worldacc = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd qCmd_td = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qDotCmd_td = Eigen::VectorXd::Zero(12);

  Eigen::Vector3d vCmd = Eigen::Vector3d::Zero(); // velocity command
  double vy = 0.0;
  double vyaw = 0.0;
  Eigen::Vector3d vCmd_joystick = Eigen::Vector3d::Zero();
  Eigen::Vector3d vCmd_offset_joystick = Eigen::Vector3d::Zero();
  Eigen::Vector3d pCmd_joystick = Eigen::Vector3d::Zero();
  Eigen::Vector3d pCmd_joystick_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d rCmd_joystick = Eigen::Vector3d::Zero();
  Eigen::Vector3d rCmd_joystick_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_com = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_com_filt = Eigen::Vector3d::Zero();
  double avg_vx = 0.0;

  Eigen::Vector3d foot_odometer = Eigen::Vector3d::Zero();
  Eigen::Vector3d odometer = Eigen::Vector3d::Zero();
  Eigen::Vector3d odometer_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d odometer_avg = Eigen::Vector3d::Zero();

  Eigen::VectorXd f_odometer = Eigen::VectorXd::Zero(4);
  Eigen::Vector2d p_odometer = Eigen::Vector2d::Zero();

  // walk to stand flag
  bool prestand = false;
  bool standcmd = false;
  //
  Eigen::VectorXd xStand_init = Eigen::VectorXd::Zero(6);

  // swing arm phase
  Eigen::Vector2d pArm_tgt = Eigen::Vector2d::Zero();
  Eigen::Vector2d vArm_tgt = Eigen::Vector2d::Zero();
  Eigen::Vector2d pArm_td = Eigen::Vector2d::Zero();
  Eigen::Vector2d vArm_td = Eigen::Vector2d::Zero();

  // state
  std::string fsmname = "Start";

  bool inSingleStand = false;
  bool momentumTurnOn = false;

  bool motionTurnOn = false;
  int motionNumber = 0;
  int carryBoxState = 0;
  bool carryBoxFirstStand = true;
  bool standCarry = false;

  double rotateAngle = 0.0; // M_PI/22.;
  double com_x_offset = 0.035;

  Eigen::Vector3d imu_acc_offset;

  // log buffer
  Eigen::VectorXd dataL = Eigen::VectorXd::Zero(300);
  std::vector<std::string> log_buffer;

  bool PushWalkRecoveryFlag = false;
  double PushWalkRecoveryDelayTime = 1.5;
  bool step_calibration_flag = false;
  bool step_calibration_flag2 = false;
  double step_calibration_x_offset = 0;
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
};

#endif // ROBOT_DATA_H
