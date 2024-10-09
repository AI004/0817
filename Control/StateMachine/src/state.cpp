#include "../include/state.h"

#include <sys/time.h>

#include "../include/StateGenerator.h"
#include "public_parament.h"
#define Datalog

// Start /////////////////////////////////////////////
void Start::init() {
  // std::cout << "Start::init() " << std::endl;
  timer = 0.0;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  gait->robot_controller_._robot_data->addlog("Start::init()");
  stateName = "Start";
  addEventTrans("gotoZero", "Zero");
  // addEventTrans("gotoStand", "Stand");
  // addEventTrans("gotoTestmpc", "Testmpc");
  // addEventTrans("gotoJump", "Jump");
  // addEventTrans("gotoWalk", "Walk");
  // addEventTrans("gotoStep", "Step");
}

void Start::onEnter() {
  std::cout << "Started " << std::endl;
  timer = 0.0;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  gait->robot_controller_._robot_data->q_c =
      gait->robot_controller_._robot_data->q_a;
  gait->robot_controller_._robot_data->q_dot_c =
      gait->robot_controller_._robot_data->q_dot_a;
  gait->robot_controller_._robot_data->q_ddot_c =
      gait->robot_controller_._robot_data->q_ddot_a;
  gait->robot_controller_._robot_data->tau_c =
      gait->robot_controller_._robot_data->tau_a;
  gait->robot_controller_._robot_data->fsmname = stateName;
  gait->robot_controller_._robot_data->addlog("Start::onEnter()");
  // gait->setevent("gotoZero");
}

void Start::run() {
  // std::cout  << "Start::run() " <<std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  // gait->robot_controller_._robot_data->addlog("Start::run()");
  // gait->setevent("gotoZero");
  auto robotdata = gait->robot_controller_._robot_data;
  // log data
  timer += robotdata->dt;
  // robotdata->dataL(19) = 0;
  // robotdata->dataL(20) = timer;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();
}

void Start::onExit() {
  // std::cout << "Start::onExit" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  gait->robot_controller_._robot_data->addlog("Start::onExit()");
}

// Zero //////////////////////////////////////////////
void Zero::init() {
  // std::cout << "Zero::init() " << std::endl;
  stateName = "Zero";
  // addEventTrans("startstand", "Stand");
  // addEventTrans("gotoSingleLegTest", "SingleLegTest");
  // addEventTrans("gotoSingleSwingTest", "SingleSwingTest");
  // addEventTrans("gotoSquat", "Squat");
  // addEventTrans("gotoExcitingTrajectory", "ExcitingTrajectory");
  // addEventTrans("gotoMotorMotion", "MotorMotion");
  // addEventTrans("gotoStand", "Stand");

  addEventTrans("gotoZ2S", "Z2S");
  addEventTrans("gotoSwing", "Swing");
  addEventTrans("gotoStop", "Stop");
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  gait->robot_controller_._robot_data->addlog("Zero::init()");
  timer = 0.0;
  //
  qa = Eigen::MatrixXd::Zero(generalized_coordinates, 1);
  qa_dot = Eigen::MatrixXd::Zero(generalized_coordinates, 1);
  qd = Eigen::MatrixXd::Zero(generalized_coordinates, 1);
  qd_dot = Eigen::MatrixXd::Zero(generalized_coordinates, 1);
  //
  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand = Eigen::VectorXd::Zero(6);

  return;
}

void Zero::onEnter() {
  std::cout << "To Zero" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  // gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;
  gait->robot_controller_._robot_data->addlog("Zero::onEnter()");
  // update
  qa = gait->robot_controller_._robot_data->q_a;
  qa_dot = gait->robot_controller_._robot_data->q_dot_a;
  // init position
  xStand(0) = -0.01;
  xStand(1) = 0.12;
  xStand(2) = -0.83;
  xStand(3) = -0.01;
  xStand(4) = -0.12;
  xStand(5) = -0.83;
  footflag = true;
  wkSpace2Joint(xStand, qCmd, qDotCmd, footflag);
  qd.setZero();
  qd.block(6, 0, 12, 1) = qCmd;

  // sit position
  // xStand(0) = 0.3;
  // xStand(1) = 0.13;
  // xStand(2) = -0.35;
  // xStand(3) = 0.3;
  // xStand(4) = -0.13;
  // xStand(5) = -0.35;
  // footflag = true;
  // wkSpace2Joint(xStand, qCmd, qDotCmd, footflag);
  // qCmd(3) = 2.44;
  // qCmd(4) = 0;
  // qCmd(9) = 2.44;
  // qCmd(10) = 0;
  // qd.setZero();
  // qd.block(6, 0, 12, 1) = qCmd;
  // qd(19) = 1.2;
  // qd(21) = 0.8;
  // qd(24) = -1.6;
  // qd(25) = 0.8;
  // qd(28) = -1.6;

  // std::cout << "qCmd: " << qCmd << std::endl;
  // qd = qa;
  // qd.block(12,0,6,1) << 0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6, 0.0;
  // qd << 0., 0., 0., 0., 0., 0.,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6.0, 0.0,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6.0, 0.0;
  // qd << 0., 0., 0., 0., 0., 0.,
  //       0.220000, 0.000000 ,-0.260000 ,1.000000, -0.170000, 0.0,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6, 0.0;

  qd_dot.setZero();
  //
  // Eigen::VectorXd qArmCmd = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);
  // Eigen::VectorXd qArmCmd_temp = Eigen::VectorXd::Zero(8);
  // armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd_temp);
  // qArmCmd.setZero();
  // qArmCmd.head(4) = qArmCmd_temp.head(4);
  // qArmCmd.segment(arm_l_actor_num+hand_l_actor_num,4) = qArmCmd_temp.tail(4);
  // qd.block(21, 0, adam_upper_except_waist_actor_num, 1) = qArmCmd;

  // init total time
  avr_v = 0.6;
  double deltq = max(abs((qd - qa).block(6, 0, 12, 1).maxCoeff()),
                     abs((qd - qa).block(6, 0, 12, 1).minCoeff()));
  totaltime = floor(deltq / avr_v / gait->robot_controller_._robot_data->dt) *
              gait->robot_controller_._robot_data->dt;
  double totaltime_min = 1.0;
  if(totaltime < totaltime_min) {totaltime = totaltime_min;}
  // std::cout << "qd: " << qd.transpose() << std::endl;
  // std::cout << "qa_dot: " << qa_dot.transpose() << std::endl;
  std::cout << "Zero Duarion: " << totaltime << std::endl;
  gait->setevent("");
}

void Zero::run() {
  // std::cout << "Zero::run() " <<std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  // std::cout<<"error!"<<std::endl;
  auto robotdata = gait->robot_controller_._robot_data;
  // // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // run
  Eigen::MatrixXd O = Eigen::MatrixXd::Zero(robotdata->ndof, 1);
  double delth = 0.07;
  double T_squad = 2.0;
  double omega = 2.0 * M_PI / T_squad;
  double n = 30;
  Eigen::VectorXd FR = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd FL = Eigen::VectorXd::Zero(6);
  // totaltime=100;
  if (timer < totaltime) {
    gait->FifthPoly(qa, qa_dot, O, qd, qd_dot, O, totaltime, timer,
                    robotdata->q_c, robotdata->q_dot_c, robotdata->q_ddot_c);
    robotdata->tau_c.setZero();
    // robotdata->q_c.segment(6+12+3+7,12) = (3.1415926/20*sin(2*3.1415926*timer)+3.1415926/20)*Eigen::VectorXd::Ones(12, 1);
    // robotdata->q_c.segment(6+12+3+7+7+12,12) = (3.1415926/20*sin(2*3.1415926*timer)+3.1415926/20)*Eigen::VectorXd::Ones(12, 1);
  } else {
    // when zero complete tell gui zero
    gait->robot_controller_._robot_data->fsmname = stateName;
    robotdata->q_c.block(6, 0, actor_num, 1) = qd.block(6, 0, actor_num, 1);
    robotdata->q_dot_c.block(6, 0, actor_num, 1).setZero();
    robotdata->tau_c.setZero();
  }
  if (timer > totaltime - 0.5 * robotdata->dt &&
      timer < totaltime + 0.5 * robotdata->dt) {
    std::cout << "Zero" << std::endl;
    std::cout << qd.block(6, 0, actor_num, 1) << std::endl;
  }

  // PD gains
  robotdata->q_factor = 1. * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
  robotdata->q_dot_factor = 1. * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);

  // std::cout<<"error!"<<std::endl;
  // robotdata->q_c.block(0,0,6,1).setZero();
  // robotdata->q_dot_c.block(0,0,6,1).setZero();
  // robotdata->q_a.block(0,0,6,1).setZero();
  // robotdata->q_dot_a.block(0,0,6,1).setZero();
  // RigidBodyDynamics::Math::VectorNd CG =
  // RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof, 1);
  // RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model),robotdata->q_a,
  // robotdata->q_dot_a,CG); robotdata->tau_c = CG;

  timer += robotdata->dt;

  if (step_count >= 40) {
    step_count = 0;
    std::cout << robotdata->q_a.segment(3, 3).transpose() << std::endl;
  } else {
    step_count++;
  }
  // updata event
  if (timer < totaltime) {
    gait->setevent("");
    // std::cout<<"totaltime: "<<totaltime<<std::endl;
  } else {
    // gait->setevent("gotoSingleSwingTest");
    // gait->setevent("gotoSquat");
    // // gait->setevent("gotoExcitingTrajectory");
    // if(timer < totaltime + 10.0){
    //     gait->setevent("");
    //     // std::cout<<"prepare to gotoStand"<<std::endl;
    //     std::cout<<"prepare to gotoWalk"<<std::endl;
    // }else{
    //     // gait->setevent("gotoStand");
    //     gait->setevent("gotoWalk");
    // }
  }
  // log data
  // robotdata->dataL(19) = 1;
  // robotdata->dataL(20) = timer;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();
  // std::cout<<"error?"<<std::endl;
  // std::cout << robotdata->grf(5)+robotdata->grf(11) << std::endl;

  // std::cout << robotdata->q_c.segment(6+12+3+4,3).transpose() << std::endl;
}

void Zero::onExit() {
  // std::cout << "Zero::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand;
  robotdata->addlog("Zero::onExit()");
}

// Z2S //////////////////////////////////////////////////////////////
void Z2S::init() {
  // std::cout << "Z2S::init() " << std::endl;
  stateName = "Z2S";
  // addEventTrans("startwalk", "Walk");
  // addEventTrans("stopstand", "Zero");
  addEventTrans("gotoStand", "Stand");
  addEventTrans("gotoStop", "Stop");
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("Z2S::init()");
  timer = 0.0;

  // init
  // com_x_a = Eigen::MatrixXd::Zero(3,6);
  // com_x_d = Eigen::MatrixXd::Zero(3,6);
  // com_x_c = Eigen::MatrixXd::Zero(3,6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  // std::cout << "adam_upper_actor_num: " << adam_upper_actor_num << std::endl;
  // std::cout << "adam_upper_except_waist_actor_num: " << adam_upper_except_waist_actor_num << std::endl;
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand = Eigen::VectorXd::Zero(6);
  xStand_tgt = Eigen::VectorXd::Zero(6);
  // qArmCmd = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);

  return;
}

void Z2S::onEnter() {
  std::cout << "To Stand" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("Z2S::onEnter()");
  gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;

  // update task actual state
  robotdata->stance_index = 1;
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  // int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  // com_x_a =
  // gait->robot_controller_._robot_data->task_card_set[com_task_id]->X_a.block(0,0,3,6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
        adam_upper_actor_num);

  avr_v = 0.1;
  totaltime = 1.0;

  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      70.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->upper_joints_id]->weight <<
      100.0 * Eigen::VectorXd::Ones(adam_upper_actor_num);

  // test
  // com_x_a.row(1).setZero();
  // com_x_d = com_x_a;
  // target
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;
  cali_imu_done = false;
  // init position
  // xStand.head(3) = left_foot_x_a.block(0,3,1,3) -
  // robotdata->q_a.block(0,0,3,1).transpose(); xStand.tail(3) =
  // right_foot_x_a.block(0,3,1,3) - robotdata->q_a.block(0,0,3,1).transpose();
  // xStand(0) = 0.08;
  // xStand(1) = 0.12;
  // xStand(2) = -0.50;
  // xStand(3) = 0.08;
  // xStand(4) = -0.12;
  // xStand(5) = -0.50;
  xStand = robotdata->xStand_init;
  wkSpace2Joint(xStand, qCmd, qDotCmd, first_flag);
  xStand_tgt = xStand;
  double delth = -0.0;
  xStand_tgt(2) += delth;
  xStand_tgt(5) += delth;

  // Eigen::VectorXd qArmCmd_temp = Eigen::VectorXd::Zero(8); 
  // armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd_temp);
  // qArmCmd.setZero();
  // qArmCmd.head(4) = qArmCmd_temp.head(4);
  // qArmCmd.segment(arm_l_actor_num+hand_l_actor_num,4) = qArmCmd_temp.tail(4);
  gait->setevent("");
}

void Z2S::run() {
  // std::cout << "Z2S::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // std::cout<<"hehe1"<<std::endl;
  // start: plan
  // double delth = -0.26;
  double delth = xStand_tgt(5) - xStand(5);
  totaltime = 3.0;
  double omega = M_PI / totaltime;
  if (timer < totaltime) {
    Eigen::VectorXd xStand_cmd = xStand;
    xStand_cmd(2) = xStand(2) + delth * (1 - cos(omega * timer)) / 2.0;
    xStand_cmd(5) = xStand(5) + delth * (1 - cos(omega * timer)) / 2.0;
    wkSpace2Joint(xStand_cmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    // robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
    // body_x_d = robotdata->task_card_set[robotdata->body_task_id]->X_a;
    body_x_d.setZero();
    body_x_d(0, 3) = -xStand_cmd(3);
    body_x_d(0, 4) = -xStand_cmd(4);
    body_x_d(0, 5) = -xStand_cmd(5);
    body_x_d(1, 5) = -delth * omega * sin(omega * (timer)) / 2.0;
    body_x_d(2, 5) = -delth * omega * omega * cos(omega * (timer)) / 2.0;
    // body_x_d.row(2).setZero();
    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();
  } else {
    body_x_d.setZero();
    body_x_d(0, 3) = -xStand(3);
    body_x_d(0, 4) = -(xStand(4));
    body_x_d(0, 5) = -(xStand(5) + delth);
    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
    robotdata->tau_c.setZero();
  }
  // std::cout << "======================" << std::endl;
  // std::cout << "qArmCmd: \t\t\t" << qArmCmd.transpose() << std::endl;
  // set waist and arm qc
  robotdata->q_c.segment(18, 3).setZero();
  // std::cout << "qArmCmd.size(): " << qArmCmd.size() << std::endl;
  robotdata->q_c.tail(adam_upper_except_waist_actor_num) = qArmCmd;
  robotdata->q_dot_c.tail(adam_upper_actor_num).setZero();
  upper_joints_x_d.row(0) = robotdata->q_c.tail(adam_upper_actor_num);
  // std::cout << "upper_joints_x_d.row(0): \t" << upper_joints_x_d.row(0).tail(8) << std::endl;

  // foot motion plan

  // end: plan
  // control
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  upper_joints_x_c = upper_joints_x_d;

  // gait data update

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->body_task_id]->X_d.block(0, 0, 3, 6) =
      body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d.block(0, 0, 3, 6) =
      left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d.block(0, 0, 3, 6) =
      right_foot_x_c;
  robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, adam_upper_actor_num) =
      upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);

  // std::cout << "robotdata->q_c: \t\t" << robotdata->q_c.tail(8).transpose() << std::endl;
  //    std::cout<<"hehe3"<<std::endl;
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  if(adam_type==ADAM_TYPE::AdamLite){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              82.5, 82.5, 82.5,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
  tau_lb = -tau_ub;
  GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
      200.0, 700.0;
  GRF_lb = -GRF_ub;
  GRF_lb(5) = 0.0;
  GRF_lb(11) = 0.0;
  Fref = robotdata->contactforce;

  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);

  // std::cout << "robotdata->q_c: \t\t" << robotdata->q_c.tail(8).transpose() << std::endl;
  // std::cout << "robotdata->q_a: \t\t" << robotdata->q_a.tail(8).transpose() << std::endl;
  // std::cout<<"timer1: "<<timer<<std::endl;
  // std::cout<<"hehe4"<<std::endl;
  double ratio = 0.0;
  double deltt = 1.5;
  if (timer < deltt) {
    ratio = 0.9 * timer / deltt;
#ifndef WEBOTS
    robotdata->tau_c = robotdata->tau_c * timer / deltt;
#endif
  } else {
    ratio = 0.9;
  }
  robotdata->q_factor =
      (1. - ratio) * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
  robotdata->q_dot_factor =
      (1. - ratio) * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);

  // update time
  timer += robotdata->dt;
  // std::cout<<"timer2: "<<timer<<std::endl;

  // log data
  // robotdata->dataL(19) = 2;
  // robotdata->dataL(20) = timer;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();
  // robotdata->tau_c = ratio*robotdata->tau_c;
  if (!cali_imu_done and timer > totaltime + 3) {
    cali_imu_done = true;
    // update imu acc offset
    // robotdata->imu_acc_offset =
    //     Eigen::Vector3d(0.0, 0.0, 9.8) - robotdata->imuAcc;
    // robotdata->imu_acc_offset.head(2).setZero();
    // robotdata->imu_acc_offset = Eigen::Vector3d(0.0, 0.0, -0.2);
    std::cout << "imu acc:" << robotdata->imuAcc.transpose() << std::endl;
    std::cout << "imu acc offset:" << robotdata->imu_acc_offset.transpose()
              << std::endl;
  }
  // updata event
  if (timer < totaltime + 0) {
    gait->setevent("");
  } else {
    gait->setevent("gotoStand");
  }

  if (timer > 0.25 &&
      robotdata->grf(5) + robotdata->grf(11) < 0.3 * robotdata->MG) {
    gait->setevent("gotoStop");
  }
}

void Z2S::onExit() {
  // std::cout << "Z2S::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt;

  robotdata->addlog("Z2S::onExit()");
}

// SingleSwingTest
// //////////////////////////////////////////////////////////////
void Swing::init() {
  stateName = "Swing";
  std::cout << "Swing" << std::endl;
  addEventTrans("gotoZero", "Zero");
  addEventTrans("gotoStop", "Stop");
  timer = 0.0;

  // init
  q_init = Eigen::VectorXd::Zero(18);
  //
  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = Eigen::VectorXd::Zero(Nwout);
  // wbc init
  body_x_a = Eigen::MatrixXd::Zero(3, 3);
  body_x_d = Eigen::MatrixXd::Zero(3, 3);
  body_x_c = Eigen::MatrixXd::Zero(3, 3);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  qArmCmd = Eigen::VectorXd::Zero(8);

  return;
}

void Swing::onEnter() {
  std::cout << "Swing" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  timer = 0.0;

  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = robotdata->xStand_init;
  xCmd = xStand;

  gait->setevent("");

  // init plan
  qCmd = robotdata->q_c.tail(12);
  qDotCmd = robotdata->q_dot_c.tail(12);

  // robotdata->pFootb_tgt = xStand.tail(3);
  // robotdata->vFootb_tgt.setZero();

  // robotdata->pTorso_tgt = -xStand.head(3);
  // robotdata->vTorso_tgt.setZero();

  robotdata->time = 0.0;
  // wbc
  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  // int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  // com_x_a =
  // gait->robot_controller_._robot_data->task_card_set[com_task_id]->X_a.block(0,0,3,6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  // test
  // com_x_a.row(1).setZero();
  // com_x_d = com_x_a;
  // target
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
}

void Swing::run() {
  // std::cout << "SingleSwingTest::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // run
  // update task actual state
  robotdata->time = timer;
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);

  // q_c = q_init;
  // q_dot_c.setZero();
  tau_c.setZero();
  if ((fabs(robotdata->vCmd_joystick(0)) > 0.1)) {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0015)) {
      robotdata->vCmd(0) +=
          0.0015 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
          fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  } else {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0025)) {
      robotdata->vCmd(0) +=
          0.0025 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
          fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  }
  // // //----------------joint space---------------------//
  airWalk(timer, robotdata->vCmd_joystick(0), vCmd, tStepPre, stIndex, xStand,
          xInit, xDotInit, xDDotInit, xEnd, xDotEnd, xCmd, xDotCmd, xDDotCmd,
          fCmd);
  wkSpace2Joint(xCmd, qCmd, qDotCmd, firstFlag);

  // set control command
  robotdata->q_c.segment(6, 12) = qCmd;
  robotdata->q_dot_c.segment(6, 12) = qDotCmd;

  armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd);
  robotdata->q_c.tail(8) = qArmCmd;

  timer += robotdata->dt;
  robotdata->tau_c.setZero();

  // log data
  robotdata->dataL(19) = 11;
  robotdata->dataL(20) = timer;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();
  // std::cout<<"error?"<<std::endl;
}

void Swing::onExit() {
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
}


// Stand //////////////////////////////////////////////////////////////
void Stand::init() {
  std::cout << "Stand::init() " << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("Stand::init()");
  stateName = "Stand";
  addEventTrans("gotoS2W", "S2W");
  addEventTrans("gotoZero", "Zero");
  addEventTrans("gotoWalk", "Walk");
  addEventTrans("gotoStop", "Stop");
  addEventTrans("gotoUniGait", "UniGait");
  addEventTrans("gotoDual2Single", "Dual2Single");

  timer = 0.0;

  // init
  com_x_a = Eigen::MatrixXd::Zero(3, 6);
  com_x_d = Eigen::MatrixXd::Zero(3, 6);
  com_x_c = Eigen::MatrixXd::Zero(3, 6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);

  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  com_tgt = Eigen::VectorXd::Zero(6);
  xStand = Eigen::VectorXd::Zero(6);
  xStand_tgt = Eigen::VectorXd::Zero(6);
  xStand_Zero = Eigen::VectorXd::Zero(6);
  q_factor_init = Eigen::VectorXd::Zero(actor_num);
  q_dot_factor_init = Eigen::VectorXd::Zero(actor_num);
  torso_d.setZero();
  rTorsoInit = Eigen::VectorXd::Zero(6);
  rTorsoCmd = Eigen::VectorXd::Zero(6);
  rTorso_tgt = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);

  // upperJointsRefPos = Eigen::VectorXd::Zero(adam_upper_actor_num);
  // upperJointsRefPosFilter = Eigen::VectorXd::Zero(adam_upper_actor_num);
  // upperJointsRefPosLast = Eigen::VectorXd::Zero(adam_upper_actor_num);

  // time1 = 0.0;
  // time2 = 0.0;
  // time3 = 0.0;
  // time4 = 0.0;
  // motion_start_flag = true;
  // box_last_state = 0;
  // box_start_timer = 0.0;

  return;
}

void Stand::onEnter() {
  std::cout << "Stand" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  std::cout << robotdata->stance_index << std::endl;
  gait->robot_controller_._robot_data->fsmname = stateName;
  robotdata->addlog("Stand::onEnter()");
  timer = 0.0;

  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  com_x_a = gait->robot_controller_._robot_data->task_card_set[com_task_id]
                ->X_a.block(0, 0, 3, 6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
                                                                      adam_upper_actor_num);

  avr_v = 0.1;
  totaltime = 3.0;

  // target
  com_x_d = com_x_a;
  com_x_d.row(2).setZero();
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  robotdata->arm_style_d = 0.0;

  // control
  com_x_c = com_x_a;
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  // upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;
  // init position
  // xStand.head(3) = left_foot_x_a.block(0,3,1,3) -
  // robotdata->q_a.block(0,0,3,1).transpose(); xStand.tail(3) =
  // right_foot_x_a.block(0,3,1,3) - robotdata->q_a.block(0,0,3,1).transpose();
  // xStand(0) = 0.08;
  // xStand(1) = 0.12;
  // xStand(2) = -0.76;
  // xStand(3) = 0.08;
  // xStand(4) = -0.12;
  // xStand(5) = -0.76;
  rTorsoInit.head(3) = robotdata->q_a.segment(3, 3);
  rTorsoInit.tail(3) = rTorsoInit.head(3);
  xStand = robotdata->xStand_init;
  torso_d = 0.5 * (left_foot_x_a.row(0).tail(3).transpose() +
                   right_foot_x_a.row(0).tail(3).transpose());
  torso_d(0) += 0.01;
  torso_d(2) = 0.83;
  xStand_tgt.head(3) = left_foot_x_a.row(0).tail(3).transpose() - torso_d;
  xStand_tgt.tail(3) = right_foot_x_a.row(0).tail(3).transpose() - torso_d;
  xStand_Zero = xStand_tgt;
  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand, rTorsoInit, rFootCmd, qCmd, qDotCmd, first_flag);
  // wkSpace2Joint(xStand,qCmd,qDotCmd,first_flag);

  com_tgt = com_x_a.row(0).transpose();

  // init
  //  WBC weights and constraints
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      100.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  // robotdata->task_card_set[robotdata->upper_joints_id]->weight =
  //     100.0 * Eigen::VectorXd::Ones(adam_upper_actor_num);

  // joystick zero
  robotdata->pCmd_joystick_last.setZero();
  robotdata->rCmd_joystick_last.setZero();
  // q_factor
  q_factor_init = robotdata->q_factor;
  q_dot_factor_init = robotdata->q_dot_factor;

  // upperJointsRefPos = upper_joints_x_a.row(0).transpose();
  upperJointsRefPosFilter = upper_joints_x_a.row(0).transpose();
  upperJointsRefPosLast = upper_joints_x_a.row(0).transpose();

  gait->setevent("");
}

void Stand::run() {
  // std::cout << "Stand::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // std::cout<<"hehe1"<<std::endl;
  // start: plan

  Eigen::VectorXd delt = xStand_Zero - xStand;
  Eigen::VectorXd delt_rpy = rTorso_tgt - rTorsoInit;
  totaltime = 0.0;
  double omega = M_PI / totaltime;
  // robotdata->q_factor << 0.1 * Eigen::VectorXd::Ones(12), 0.5 *
  // Eigen::VectorXd::Ones(3), 0.5 * Eigen::VectorXd::Ones(8);
  // robotdata->q_dot_factor = robotdata->q_factor;

  if (timer < totaltime) {
    robotdata->task_card_set[robotdata->body_task_id]->weight
        << (100., 100., 100., 10., 10., 10.) * timer / totaltime +
               (100., 100., 100., 10., 10., 100.) * (totaltime - timer) /
                   totaltime;
    robotdata->task_card_set[robotdata->com_task_id]->weight
        << (20., 20., 20., 100., 100., 0.) * timer / totaltime;
    // robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
    // 100., 100., 100., 100.;
    // robotdata->task_card_set[robotdata->com_task_id]->weight << 0., 0.,
    // 0., 10., 0., 0.;

    Eigen::VectorXd xStand_cmd = xStand;
    xStand_cmd = xStand + delt * (1 - cos(omega * timer)) / 2.0;
    // xStand_cmd = xStand_Zero;
    rTorsoCmd = rTorsoInit + delt_rpy * (1 - cos(omega * timer)) / 2.0;

    // wkSpace2Joint(xStand_cmd, qCmd, qDotCmd, first_flag);
    wkSpace2Joint(xStand_cmd, rTorsoCmd, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;

    com_tgt = robotdata->task_card_set[robotdata->com_task_id]
                  ->X_a.row(0)
                  .transpose();
    com_tgt.segment(3, 2) = 0.5 * (left_foot_x_d.block(0, 3, 1, 2) +
                                   right_foot_x_d.block(0, 3, 1, 2))
                                      .transpose();
    com_tgt(3) += robotdata->com_x_offset;
    com_x_d.setZero();
    com_x_d.row(0) = com_x_a.row(0) + (com_tgt.transpose() - com_x_a.row(0)) *
                                          (1 - cos(omega * timer)) / 2.0;

    // body_x_d = robotdata->task_card_set[robotdata->body_task_id]->X_a;
    body_x_d.setZero();
    if (robotdata->stance_index == 1) {
      body_x_d.row(0).tail(3) = -xStand_cmd.tail(3).transpose();
      body_x_d.row(1).tail(3) =
          -delt.tail(3).transpose() * omega * sin(omega * (timer)) / 2.0;
      body_x_d.row(2).tail(3) = -delt.tail(3).transpose() * omega * omega *
                                cos(omega * (timer)) / 2.0;
    } else {
      body_x_d.row(0).tail(3) = -xStand_cmd.head(3).transpose();
      body_x_d.row(1).tail(3) =
          -delt.head(3).transpose() * omega * sin(omega * (timer)) / 2.0;
      body_x_d.row(2).tail(3) = -delt.head(3).transpose() * omega * omega *
                                cos(omega * (timer)) / 2.0;
    }

    // body_x_d.row(2).setZero();
    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    // // q_factor
    Eigen::VectorXd factor_tgt = Eigen::VectorXd::Zero(robotdata->ndof - 6);
    factor_tgt << 0.1 * Eigen::VectorXd::Ones(12),
        0.5 * Eigen::VectorXd::Ones(3), 0.5 * Eigen::VectorXd::Ones(8);

    robotdata->q_factor = factor_tgt * timer / totaltime +
                          q_factor_init * (totaltime - timer) / totaltime;
    robotdata->q_dot_factor =
        factor_tgt * timer / totaltime +
        q_dot_factor_init * (totaltime - timer) / totaltime;
  } else {
    // double t_trans = 1.0;
    // if (timer < totaltime + t_trans){
    //   robotdata->task_card_set[robotdata->body_task_id]->weight
    //     << (100., 100., 100., 10., 10., 10.) * (timer-totaltime) / t_trans +
    //     (100., 100., 100., 100., 100., 100.) * (1.0 - (timer-totaltime) /
    //     t_trans);
    //   robotdata->task_card_set[robotdata->com_task_id]->weight <<
    //   (20., 20., 20., 100., 100., 100.) * (timer-totaltime) / t_trans + (0.,
    //   0., 0., 10., 0., 0.)* (1.0 - (timer-totaltime) / t_trans);
    // }
    // else{
    //   robotdata->task_card_set[robotdata->body_task_id]->weight << 100.,
    //   100., 100., 10., 10., 10.;
    //   robotdata->task_card_set[robotdata->com_task_id]->weight
    //   << 20., 20., 20., 100., 100., 100.;
    // }

    robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
        100., 10., 10., 10.;
    robotdata->task_card_set[robotdata->com_task_id]->weight << 20., 20., 20.,
        100., 100., 100.;
    robotdata->momentumTurnOn = false;

    com_x_d.setZero();
    com_x_d.row(0) = com_tgt.transpose();
    com_x_d.block(0, 3, 1, 2) = 0.5 * (left_foot_x_d.block(0, 3, 1, 2) +
                                       right_foot_x_d.block(0, 3, 1, 2));
    com_x_d(0, 3) += robotdata->com_x_offset;

    body_x_d.setZero();

    // joystick x position and pitch
    double delt_x = 0.0005;
    if (fabs(robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0)) >
        delt_x) {
      robotdata->pCmd_joystick_last(0) +=
          delt_x *
          (robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0)) /
          fabs(robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0));
    } else {
      robotdata->pCmd_joystick_last(0) = robotdata->pCmd_joystick(0);
    }
    xStand_tgt(0) = xStand_Zero(0) + 0.5 * robotdata->pCmd_joystick_last(0);
    xStand_tgt(3) = xStand_Zero(3) + 0.5 * robotdata->pCmd_joystick_last(0);
    rTorsoCmd(1) = 2.0 * robotdata->pCmd_joystick_last(0);
    rTorsoCmd(4) = 2.0 * robotdata->pCmd_joystick_last(0);
    // robotdata->q_c(19) = 3.0 * robotdata->pCmd_joystick_last(0);

    // joystick y position and roll
    double delt_y = 0.00025;
    if (fabs(robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1)) >
        delt_y) {
      robotdata->pCmd_joystick_last(1) +=
          delt_y *
          (robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1)) /
          fabs(robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1));
    } else {
      robotdata->pCmd_joystick_last(1) = robotdata->pCmd_joystick(1);
    }
    xStand_tgt(1) = xStand_Zero(1) + 0.4 * robotdata->pCmd_joystick_last(1);
    xStand_tgt(4) = xStand_Zero(4) + 0.4 * robotdata->pCmd_joystick_last(1);
    rTorsoCmd(0) = -2.0 * robotdata->pCmd_joystick_last(1);
    rTorsoCmd(3) = -2.0 * robotdata->pCmd_joystick_last(1);
    // robotdata->q_c(18) = -3.0 * robotdata->pCmd_joystick_last(1);

    // joystick z position
    double delt_z = 0.0005;
    if (fabs(robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2)) >
        delt_z) {
      robotdata->pCmd_joystick_last(2) +=
          delt_z *
          (robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2)) /
          fabs(robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2));
    } else {
      robotdata->pCmd_joystick_last(2) = robotdata->pCmd_joystick(2);
    }
    xStand_tgt(2) = xStand_Zero(2) - robotdata->pCmd_joystick_last(2);
    xStand_tgt(5) = xStand_Zero(5) - robotdata->pCmd_joystick_last(2);
    com_x_d(0, 5) = com_tgt(5) + robotdata->pCmd_joystick_last(2);

    // joystick Yaw
    double delt_yaw = 0.001;
    if (fabs(robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2)) >
        delt_yaw) {
      robotdata->rCmd_joystick_last(2) +=
          delt_yaw *
          (robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2)) /
          fabs(robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2));
    } else {
      robotdata->rCmd_joystick_last(2) = robotdata->rCmd_joystick(2);
    }
    rTorsoCmd(2) = robotdata->rCmd_joystick_last(2);
    rTorsoCmd(5) = robotdata->rCmd_joystick_last(2);
    // robotdata->q_c(20) = robotdata->rCmd_joystick_last(2);

    // wkSpace2Joint(xStand_tgt, qCmd, qDotCmd, first_flag);
    wkSpace2Joint(xStand_tgt, rTorsoCmd, rFootCmd, qCmd, qDotCmd, first_flag);

    // robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    // robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
    body_x_d.row(0).head(3) = rTorsoCmd.head(3).transpose();
    if (robotdata->stance_index == 1) {
      body_x_d.row(0).tail(3) = -xStand_tgt.tail(3).transpose();
    } else {
      body_x_d.row(0).tail(3) = -xStand_tgt.head(3).transpose();
    }

    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
    robotdata->tau_c.setZero();
  }
  
  com_x_c = com_x_d;
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  // upper_joints_x_c = upper_joints_x_d;

  // gait data update

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->com_task_id]->X_d.block(0, 0, 3, 6) =
      com_x_c;
  robotdata->task_card_set[robotdata->body_task_id]->X_d.block(0, 0, 3, 6) =
      body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d.block(0, 0, 3, 6) =
      left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d.block(0, 0, 3, 6) =
      right_foot_x_c;
  // robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, adam_upper_actor_num) =
  //     upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);
  //    std::cout<<"hehe3"<<std::endl;
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  if(adam_type==ADAM_TYPE::AdamLite){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              82.5, 82.5, 82.5,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
  tau_lb = -tau_ub;
  GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
      200.0, 700.0;
  GRF_lb = -GRF_ub;
  GRF_lb(5) = 0.0;
  GRF_lb(11) = 0.0;
  Fref = robotdata->contactforce;

  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);
  // std::cout<<"timer1: "<<timer<<std::endl;
  // std::cout<<"hehe4"<<std::endl;
  // std::cout << "tau: " << robotdata->tau_a.segment(6, 12).transpose() <<
  // std::endl; std::cout<<"q_factor: "<<robotdata->q_factor.transpose()<<
  // std::endl; std::cout << "q_c: " << robotdata->q_c.segment(6,
  // 12).transpose() << std::endl; std::cout << "q_a: " <<
  // robotdata->q_a.segment(6, 12).transpose() << std::endl;
  // robotdata->q_factor.setZero();
  // robotdata->q_dot_factor.setZero();

  // update time
  timer += robotdata->dt;
  // std::cout<<"timer2: "<<timer<<std::endl;
  // updata event
  // walk criteria
  // cp
  double lamada = sqrt(9.81 / (fabs(torso_d(2)) + robotdata->poffset_z));
  double center_x =
      0.5 * (robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 3) +
             robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 3));
  double center_y =
      0.5 * (robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 4) +
             robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 4));
  double p_x =
      robotdata->task_card_set[robotdata->com_task_id]->X_a(0, 3) - center_x;
  double p_y =
      robotdata->task_card_set[robotdata->com_task_id]->X_a(0, 4) - center_y;
  double m_total = robotdata->task_card_set[robotdata->com_task_id]->IG(5, 5);
  double cp_x = p_x + 0.0 * robotdata->poffset_x +
                robotdata->task_card_set[robotdata->com_task_id]->X_a(1, 3) /
                    (m_total * lamada);
  double cp_y = robotdata->task_card_set[robotdata->com_task_id]->X_a(1, 4) /
                (m_total * lamada);
  Eigen::Vector3d chestPoseAct =
      robotdata->task_card_set[robotdata->chest_task_id]
          ->X_a.block(0, 0, 1, 3)
          .transpose();
  double delt_l = 0.06;

  // if(timer > totaltime + 1.0 && robotdata->motionTurnOn == false &&
  // robotdata->carryBoxState == 0 && fabs(robotdata->pCmd_joystick_last(0)) <
  // 0.01 &&  fabs(robotdata->pCmd_joystick_last(1)) < 0.01
  //  && fabs(robotdata->rCmd_joystick_last(2)) < 0.01 &&
  //  fabs(robotdata->pCmd_joystick_last(2)) < 0.005)
  // {
  //     //
  //     if((fabs(cp_x)>0.04)||(cp_y>(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)-delt_l))
  //     //
  //     ||(cp_y<(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)+delt_l))){
  //     //     gait->setevent("gotoUniGait");
  //     //     std::cout << "enter walk" << std::endl;
  //     //
  //     if(cp_y>(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)-delt_l)){
  //     //         robotdata->stance_index = 0;
  //     //     }
  //     //
  //     if(cp_y<(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)+delt_l)){
  //     //         robotdata->stance_index = 1;
  //     //     }
  //     // }
  //     if((fabs(cp_x-0.035)>0.045)||cp_y>0.8*(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)
  //     - robotdata->task_card_set[robotdata->com_task_id]->X_a(0, 4))
  //         ||cp_y<0.8*(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)-robotdata->task_card_set[robotdata->com_task_id]->X_a(0,
  //         4))
  //         ||fabs(chestPoseAct(1))>0.4 || fabs(chestPoseAct(0))>0.3){
  //         gait->setevent("gotoUniGait");
  //         std::cout << "enter walk" << std::endl;
  //         std::cout << "cp_x:" << cp_x << std::endl;
  //         std::cout << "cp_y:" << cp_y << std::endl;
  //         if(cp_y>0){
  //             robotdata->stance_index = 0;
  //         }
  //         else{
  //             robotdata->stance_index = 1;
  //         }
  //         if(fabs(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)
  //         - robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)) <
  //         0.2){
  //             robotdata->stance_index = 1 - robotdata->stance_index;
  //         }
  //     }
  // }

  // log data
  // robotdata->dataL(19) = 3;
  // robotdata->dataL(20) = timer;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();

  // robotdata->dataL.segment(243, 3) = robotdata->pCmd_joystick_last;
  // robotdata->dataL.segment(246, 3) = robotdata->rCmd_joystick_last;
  // robotdata->dataL.segment(249, 6) =
  //     robotdata->task_card_set[robotdata->com_task_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(255, 6) =
  //     robotdata->task_card_set[robotdata->com_task_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(261, 6) =
  //     robotdata->task_card_set[robotdata->body_task_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(267, 6) =
  //     robotdata->task_card_set[robotdata->body_task_id]->X_a.row(1).transpose();

  // robotdata->dataL(273) = robotdata->motionTurnOn;
  // robotdata->dataL(274) = robotdata->motionNumber;
  // robotdata->dataL(275) = robotdata->momentumTurnOn;
  // robotdata->dataL(276) = robotdata->carryBoxState;
  // robotdata->dataL.segment(277, 6) = robotdata->temp;

  // robotdata->tau_c = ratio*robotdata->tau_c;
  // std::cout << robotdata->grf(5)+robotdata->grf(11) << std::endl;

  if (timer > totaltime + 1.0 &&
      robotdata->grf(5) + robotdata->grf(11) < 0.3 * robotdata->MG) {
    std::cout << robotdata->grf(5) << " " << robotdata->grf(11) << std::endl;
    std::cout << "go to stop auto" << std::endl;
    gait->setevent("gotoStop");
  }
  if (robotdata->PushWalkRecoveryFlag) {
    robotdata->PushWalkRecoveryFlag = false;
    gait->setevent("gotoS2W");
    std::cout << "push walk recovery" << std::endl;
  }
}

void Stand::onExit() {
  // std::cout << "Stand::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt;
  robotdata->rCmd_joystick_last.setZero();
  robotdata->rCmd_joystick.setZero();
  // robotdata->stance_index = 0;
  robotdata->momentumTurnOn = false;
  robotdata->motionTurnOn = false;
  motion_start_flag = true;
  robotdata->addlog("Stand::onExit()");
}

// S2W //////////////////////////////////////////////////////////////
void S2W::init() {
  // std::cout << "S2W::init()" << std::endl;
  stateName = "S2W";
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("S2W::init()");
  // addEventTrans("startwalk", "Walk");
  // addEventTrans("stopstand", "Zero");
  addEventTrans("gotoWalk", "Walk");
  addEventTrans("gotoStop", "Stop");
  addEventTrans("gotoUniGait", "UniGait");
  timer = 0.0;

  // init
  // com_x_a = Eigen::MatrixXd::Zero(3,6);
  // com_x_d = Eigen::MatrixXd::Zero(3,6);
  // com_x_c = Eigen::MatrixXd::Zero(3,6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  // upper_joints_x_a = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  // upper_joints_x_d = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  // upper_joints_x_c = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);

  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand = Eigen::VectorXd::Zero(6);
  xStand_tgt = Eigen::VectorXd::Zero(6);

  rTorsoCmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);

  q_factor_init = Eigen::VectorXd::Zero(actor_num);
  q_dot_factor_init = Eigen::VectorXd::Zero(actor_num);
  return;
}

void S2W::onEnter() {
  std::cout << "To Walk" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;
  robotdata->addlog("S2W::onEnter()");
  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      100.0 * Eigen::VectorXd::Ones(6);
  // robotdata->task_card_set[robotdata->upper_joints_id]->weight =
  //     100.0 * Eigen::VectorXd::Ones(adam_upper_actor_num);

  // record static start state
  // start position
  // int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  // com_x_a =
  // gait->robot_controller_._robot_data->task_card_set[com_task_id]->X_a.block(0,0,3,6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  // upper_joints_x_a =
      // robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
      //                                                                 adam_upper_actor_num);

  avr_v = 0.1;
  totaltime = 3.0;

  // test
  // com_x_a.row(1).setZero();
  // com_x_d = com_x_a;
  // target
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;
  // init position
  // xStand.head(3) = left_foot_x_a.block(0,3,1,3) -
  // robotdata->q_a.block(0,0,3,1).transpose(); xStand.tail(3) =
  // right_foot_x_a.block(0,3,1,3) - robotdata->q_a.block(0,0,3,1).transpose();
  // xStand(0) = 0.08;
  // xStand(1) = 0.12;
  // xStand(2) = -0.76;
  // xStand(3) = 0.08;
  // xStand(4) = -0.12;
  // xStand(5) = -0.76;
  xStand = robotdata->xStand_init;
  xStand(0) = left_foot_x_a(0, 3) - robotdata->q_a(0);
  xStand(3) = right_foot_x_a(0, 3) - robotdata->q_a(0);
  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand, rTorsoCmd, rFootCmd, qCmd, qDotCmd, first_flag);
  // wkSpace2Joint(xStand,qCmd,qDotCmd,first_flag);
  double delty = 0.04;
  xStand_tgt = xStand;
  // robotdata->stance_index = 1;
  // if (robotdata->stance_index == 0) {
  //   delty = -delty;
  // }
  if (robotdata->stance_index == 0) {
    delty = body_x_a(0, 4) + 0.07;
  } else {
    delty = body_x_a(0, 4) - 0.07;
  }
  xStand_tgt(1) += delty;
  xStand_tgt(4) += delty;

  q_factor_init = robotdata->q_factor;
  q_dot_factor_init = robotdata->q_dot_factor;

  gait->setevent("");
}

void S2W::run() {
//   std::cout << "S2W::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // std::cout<<"hehe1"<<std::endl;
  // start: plan

  double delty = 0.0;
  delty = xStand_tgt(1) - xStand(1);
  // std::cout << "delty: " << delty << std::endl;
  totaltime = 0.5;
  double omega = 0.5 * M_PI / totaltime;
  if (timer < totaltime) {
    Eigen::VectorXd xStand_cmd = xStand;
    xStand_cmd(1) = xStand(1) + delty * (1 - cos(omega * timer));
    xStand_cmd(4) = xStand(4) + delty * (1 - cos(omega * timer));
    // wkSpace2Joint(xStand_cmd, qCmd, qDotCmd, first_flag);
    wkSpace2Joint(xStand_cmd, rTorsoCmd, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
    // body_x_d = robotdata->task_card_set[robotdata->body_task_id]->X_a;
    body_x_d.setZero();
    if (robotdata->stance_index == 1) {
      body_x_d(0, 3) = -xStand_cmd(3);
      body_x_d(0, 4) = -xStand_cmd(4);
      body_x_d(0, 5) = -xStand_cmd(5);
    } else {
      body_x_d(0, 3) = -xStand_cmd(0);
      body_x_d(0, 4) = -xStand_cmd(1);
      body_x_d(0, 5) = -xStand_cmd(2);
    }

    body_x_d(1, 4) = -delty * omega * sin(omega * (timer));
    body_x_d(2, 4) = -delty * omega * omega * cos(omega * (timer));
    // body_x_d.row(2).setZero();
    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    // Eigen::VectorXd qArmCmd = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);
    // Eigen::VectorXd qArmCmd_temp = Eigen::VectorXd::Zero(8); 
    // armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd_temp);
    // qArmCmd.setZero();
    // qArmCmd.head(4) = qArmCmd_temp.head(4);
    // qArmCmd.segment(arm_l_actor_num+hand_l_actor_num,4) = qArmCmd_temp.tail(4);

    // robotdata->q_c.segment(18, 3) =
    //     (1.0 - timer / totaltime) *
    //         upper_joints_x_a.block(0, 0, 3, 1).transpose() +
    //     (timer / totaltime) * Eigen::VectorXd::Zero(3);
    // robotdata->q_c.tail(adam_upper_except_waist_actor_num) =
    //     (1.0 - timer / totaltime) *
    //         upper_joints_x_a.block(0, 3, adam_upper_except_waist_actor_num, 1).transpose() +
    //     (timer / totaltime) * qArmCmd;
    // upper_joints_x_d.row(0) = robotdata->q_c.tail(adam_upper_actor_num);
  } else {
    body_x_d.setZero();
    if (robotdata->stance_index == 1) {
      body_x_d(0, 3) = -xStand(3);
      body_x_d(0, 4) = -(xStand(4) + delty);
      body_x_d(0, 5) = -xStand(5);
    } else {
      body_x_d(0, 3) = -xStand(0);
      body_x_d(0, 4) = -(xStand(1) + delty);
      body_x_d(0, 5) = -xStand(2);
    }

    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
    robotdata->tau_c.setZero();

    // Eigen::VectorXd qArmCmd = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);
    // Eigen::VectorXd qArmCmd_temp = Eigen::VectorXd::Zero(8); 
    // armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd_temp);
    // qArmCmd.setZero();
    // qArmCmd.head(4) = qArmCmd_temp.head(4);
    // qArmCmd.segment(arm_l_actor_num+hand_l_actor_num,4) = qArmCmd_temp.tail(4);

    // robotdata->q_c.segment(18, 3).setZero();
    // robotdata->q_c.tail(14) = qArmCmd;
    // upper_joints_x_d.row(0) = robotdata->q_c.tail(adam_upper_actor_num);
  }

  if (robotdata->carryBoxState == 1) {
    Eigen::VectorXd Q_arm_d = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num, 1);
    if(adam_type==ADAM_TYPE::AdamLite){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0,
                -0.5, -0.03, 0.3, -1.0;
    }else if(adam_type==ADAM_TYPE::AdamStandard){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1, 0.0,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1, 0.0;
    }else if(adam_type==ADAM_TYPE::StandardPlus23){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0,
                -0.5, -0.03, 0.3, -1.0;
    }else if(adam_type==ADAM_TYPE::StandardPlus29){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1;
    }else if(adam_type==ADAM_TYPE::StandardPlus53){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    robotdata->q_c.tail(adam_upper_except_waist_actor_num) = Q_arm_d;
    upper_joints_x_d.row(0) = robotdata->q_c.tail(adam_upper_actor_num).transpose();
    robotdata->carryBoxFirstStand = false;
  } else if (robotdata->carryBoxState == 2) {
    robotdata->carryBoxState = 0;
    robotdata->carryBoxFirstStand = true;
  }

  // foot motion plan

  // end: plan
  // control
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  upper_joints_x_c = upper_joints_x_d;

  // gait data update

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->body_task_id]->X_d = body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d = left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d = right_foot_x_c;
  // robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, adam_upper_actor_num) =
  //     upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);
  //    std::cout<<"hehe3"<<std::endl;
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  if(adam_type==ADAM_TYPE::AdamLite){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0,
              18.0, 18.0, 18.0, 18.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 60.0,
              82.5, 82.5, 82.5, 
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              200.0, 130.0, 100.0, 200.0, 60.0, 20.0,
              82.5, 82.5, 82.5,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
              18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0,
              1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  }
  tau_lb = -tau_ub;
  GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 450.0, 16.6, 32.0, 10.0, 200.0,
      200.0, 450.0;
  GRF_lb = -GRF_ub;
  GRF_lb(5) = 0.0;
  GRF_lb(11) = 0.0;
  Fref = robotdata->contactforce;

  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);
  // std::cout<<"timer1: "<<timer<<std::endl;
  // std::cout<<"hehe4"<<std::endl;

  Eigen::VectorXd factor_tgt = Eigen::VectorXd::Ones(robotdata->ndof - 6);
  factor_tgt << 0.2 * Eigen::VectorXd::Ones(12);
  if (timer < totaltime) {
    robotdata->q_factor = factor_tgt * timer / totaltime +
                          q_factor_init * (totaltime - timer) / totaltime;
    robotdata->q_dot_factor =
        factor_tgt * timer / totaltime +
        q_dot_factor_init * (totaltime - timer) / totaltime;
  } else {
    robotdata->q_factor = factor_tgt;
    robotdata->q_dot_factor = factor_tgt;
  }

  // update time
  timer += robotdata->dt;
  // std::cout<<"timer2: "<<timer<<std::endl;
  // updata event
  // log data
  // robotdata->dataL(19) = 4;
  // robotdata->dataL(20) = timer;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();
  // robotdata->tau_c = ratio*robotdata->tau_c;

  // std::cout<<"error!"<<std::endl;
  // updata event
  if (timer < totaltime) {
    gait->setevent("");
  } else {
    // gait->setevent("gotoWalk");
    gait->setevent("gotoUniGait");
  }
}

void S2W::onExit() {
  // std::cout << "S2W::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt;
  robotdata->addlog("S2W::onExit()");
}

// Walk //////////////////////////////////////////////////////////////
void Walk::init() {
  // std::cout << "Walk::init() " << std::endl;

  stateName = "Walk";
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("Walk::init()");
  addEventTrans("gotoStand", "Stand");
  addEventTrans("gotoStop", "Stop");
  timer = 0.0;

  // init
  q_init = Eigen::VectorXd::Zero(18);
  //
  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = Eigen::VectorXd::Zero(Nwout);
  xStand(0) = 0.08;
  xStand(1) = 0.08 + 0.04;
  xStand(2) = -0.76;
  xStand(3) = 0.08;
  xStand(4) = -0.08 + 0.04;
  xStand(5) = -0.76;
  // wbc init
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, 11);

  rTorsoCmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);

  qArmCmd = Eigen::VectorXd::Zero(8);

  return;
}

void Walk::onEnter() {
  std::cout << "Walk" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;
  robotdata->addlog("Walk::onEnter()");
  // gait para init-----------------------------//
  // robotdata->stance_index = 1;
  if (robotdata->stance_index == 1) {
    robotdata->touch_index = 1;
    robotdata->touch_index_pre = 1;
  } else {
    robotdata->touch_index = 3;
    robotdata->touch_index_pre = 3;
  }

  robotdata->time = 0.0;     // run time (sec) for current behavior
  robotdata->t = 0.0;        // passed-time (sec), since last Phase switch
  robotdata->t_switch = 0.0; // last TD time
  robotdata->s = 0.0; // the time-variant parameter: s = clamp(t/Ts, 0, 1)
  // double T{0.4};
  // double Td{0.4};
  // double Td2{0.3};
  // double Tc{0.02};
  robotdata->step = 0;

  robotdata->foot_odometer.setZero();
  robotdata->odometer.setZero();
  robotdata->odometer_d.setZero();
  robotdata->odometer_avg.setZero();

  //-----------------------------------------//
  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = Eigen::VectorXd::Zero(Nwout);
  // xStand(0) = 0.08;
  // xStand(1) = 0.08 + 0.04;
  // xStand(2) = -0.76;
  // xStand(3) = 0.08;
  // xStand(4) = -0.08 + 0.04;
  // xStand(5) = -0.76;
  xStand = robotdata->xStand_init;
  xCmd = xStand;

  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand, rTorsoCmd, rFootCmd, qCmd, qDotCmd, firstFlag);
  // wkSpace2Joint(xStand, qCmd, qDotCmd, firstFlag);
  q_init.setZero();
  q_init.tail(12) = qCmd;

  gait->setevent("");
  // gait plan
  gait_plan = new gaitPlan();
  // init plan
  qCmd = robotdata->q_c.segment(6, 12);
  qDotCmd = robotdata->q_dot_c.segment(6, 12);

  if (robotdata->stance_index == 1) {
    robotdata->pFootb_tgt = xStand.tail(3);
    robotdata->vFootb_tgt.setZero();

    robotdata->pTorso_tgt = -xStand.head(3);
    robotdata->vTorso_tgt.setZero();
  } else {
    robotdata->pFootb_tgt = xStand.head(3);
    robotdata->vFootb_tgt.setZero();

    robotdata->pTorso_tgt = -xStand.tail(3);
    robotdata->vTorso_tgt.setZero();
  }

  robotdata->time = 0.0;

  gait_plan->init(qCmd, qDotCmd, xStand, robotdata);
  // wbc
  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  // int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  // com_x_a =
  // gait->robot_controller_._robot_data->task_card_set[com_task_id]->X_a.block(0,0,3,6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
                                                                      11);
  // test
  // com_x_a.row(1).setZero();
  // com_x_d = com_x_a;
  // target
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();
  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  gait->W2S = false;
  robotdata->prestand = false;
  prestand_step = 0;
  // joystick zero
  robotdata->vyaw = 0.0;
  robotdata->vCmd.setZero();

  // robotdata->pArm_tgt << 0.5, 0.5;
  // robotdata->vArm_tgt << 0.0, 0.0;
}

void Walk::run() {
  // std::cout << "Walk::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // run
  // update task actual state
  robotdata->time = timer;
  gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(
      robotdata);

  // q_c = q_init;
  // q_dot_c.setZero();
  tau_c.setZero();
  // //----------------joint space---------------------//
  // gait plan
  gait_plan->walkPlan(robotdata, qCmd, qDotCmd);
  // robot arm

  armPolyJoint(robotdata->pArm_tgt, qArmCmd);
  robotdata->q_c.tail(8) = qArmCmd;
  upper_joints_x_d.row(0) = robotdata->q_c.tail(11);
  if (robotdata->carryBoxState == 1) {
    Eigen::VectorXd Q_arm_d = Eigen::VectorXd::Zero(8, 1);
    Q_arm_d << -0.5, 0.03, -0.3, -1.0, -0.5, -0.03, 0.3, -1.0;
    robotdata->q_c.tail(8) = Q_arm_d;
    upper_joints_x_d.row(0) = robotdata->q_c.tail(11).transpose();
  }
  // std::cout << "robotdata->q_c.tail(8): " <<
  // robotdata->q_c.tail(8).transpose() << std::endl;

  // wbc solve
  // control
  body_x_c = robotdata->task_card_set[robotdata->body_task_id]->X_a;
  body_x_c.row(0).head(3).setZero();
  body_x_c(0, 0) = -0.2 * robotdata->avg_vx * robotdata->vyaw;
  body_x_c.row(1).head(3).setZero();
  // joystick yaw
  if ((fabs(robotdata->vCmd_joystick(2)) > 0.1)) {
    if (fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw) > 0.04) {
      robotdata->vyaw += 0.04 *
                         (robotdata->vCmd_joystick(2) - robotdata->vyaw) /
                         fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw);
    } else {
      robotdata->vyaw = robotdata->vCmd_joystick(2);
    }
  } else {
    if (fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw) > 0.1) {
      robotdata->vyaw += 0.1 * (robotdata->vCmd_joystick(2) - robotdata->vyaw) /
                         fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw);
    } else {
      robotdata->vyaw = robotdata->vCmd_joystick(2);
    }
  }

  body_x_c(1, 2) = robotdata->vyaw;
  body_x_c.row(2).setZero();
  body_x_c.block(0, 3, 1, 3) = robotdata->pTorso_tgt.transpose();
  body_x_c.block(1, 3, 1, 3) = robotdata->vTorso_tgt.transpose();
  if (robotdata->stance_index == 0) {
    left_foot_x_c = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_c.row(2).setZero();
    right_foot_x_c.setZero();
    right_foot_x_c.block(0, 3, 1, 3) =
        robotdata->pFootb_tgt.transpose() +
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1,
                                                                     3);
    right_foot_x_c.block(1, 3, 1, 3) =
        robotdata->vFootb_tgt.transpose() +
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1,
                                                                     3);
  } else {
    right_foot_x_c = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_c.row(2).setZero();
    left_foot_x_c.setZero();
    left_foot_x_c.block(0, 3, 1, 3) =
        robotdata->pFootb_tgt.transpose() +
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1,
                                                                     3);
    left_foot_x_c.block(1, 3, 1, 3) =
        robotdata->vFootb_tgt.transpose() +
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1,
                                                                     3);
  }
  // gait data update

  // plan_data
  robotdata->task_card_set[robotdata->body_task_id]->X_d = body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d = left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d = right_foot_x_c;
  robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, 11) =
      upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);

  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  // tau_lb << 82.5, 82.5, 100.0, 100.0, 32.0, 32.0, 82.5, 82.5, 100.0,
  // 100.0, 32.0, 32.0; tau_lb = -tau_lb; tau_ub << 82.5, 82.5, 100.0,
  // 100.0, 32.0, 32.0, 82.5, 82.5, 100.0, 100.0, 32.0, 32.0; GRF_ub
  // << 16.6,32.0,10.0,200.0,200.0,450.0,16.6,32.0,10.0,200.0,200.0,450.0;
  // GRF_lb = -GRF_ub;
  // GRF_lb(5) = 0.0;
  // GRF_lb(11) = 0.0;
  // Fref  = robotdata->contactforce;
  tau_lb = robotdata->tau_lb;
  tau_ub = robotdata->tau_ub;
  GRF_lb = robotdata->GRF_lb;
  GRF_ub = robotdata->GRF_ub;
  Fref = robotdata->contactforce;

  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);

  // set control command
  robotdata->q_c.segment(6, 12) = qCmd;
  robotdata->q_dot_c.segment(6, 12) = qDotCmd;

  timer += robotdata->dt;
  // updata event
  if (gait->current_fsmstate_command == "gotoZ2S") {
    gait->W2S = true;
  } else {
    robotdata->prestand = false;
    prestand_step = 0;
    gait->W2S = false;
  }
  if (gait->W2S == true) {
    if ((robotdata->t < 0.5 * robotdata->dt) &&
        (robotdata->prestand == false) &&
        (fabs(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 3) -
              robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 3)) <
         0.02) &&
        (fabs(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 4) -
              robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 4)) <
         0.32) &&
        (fabs(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 4) -
              robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 4)) >
         0.17) &&
        robotdata->step > 0) {
      robotdata->prestand = true;
      prestand_step = robotdata->step;
    }
    if ((robotdata->prestand == true) &&
        (robotdata->step == prestand_step + 1) &&
        (robotdata->t < 0.5 * robotdata->dt)) {
      gait->setevent("gotoStand");
      robotdata->prestand = false;
      prestand_step = 0;
      gait->W2S = false;
    }
  } else {
    gait->setevent("");
  }
  // std::cout<<"W2S: "<<gait->W2S<<std::endl;
  // std::cout<<"prestand: "<<robotdata->prestand<<std::endl;
  // log data
  robotdata->dataL(19) = 5;
  robotdata->dataL(20) = robotdata->time;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();

  robotdata->dataL.segment(243, 3) = robotdata->vCmd;
}

void Walk::onExit() {
  // std::cout << "Walk::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  delete gait_plan;
  auto robotdata = gait->robot_controller_._robot_data;
  if (robotdata->stance_index == 0) {
    robotdata->xStand_init.head(3) = -robotdata->pTorso_tgt;
    robotdata->xStand_init.tail(3) = robotdata->pFootb_tgt;
  } else {
    robotdata->xStand_init.head(3) = robotdata->pFootb_tgt;
    robotdata->xStand_init.tail(3) = -robotdata->pTorso_tgt;
  }
  robotdata->addlog("Walk::onExit()");
}

// UniGait ///////////////////////////////////////////////////////////
void UniGait::init() {
  // std::cout << "Walk::init() " << std::endl;

  stateName = "UniGait";
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->addlog("UniGait::init()");
  addEventTrans("gotoStand", "Stand");
  addEventTrans("gotoStop", "Stop");
  timer = 0.0;

  // init
  q_init = Eigen::VectorXd::Zero(18);
  //
  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = Eigen::VectorXd::Zero(Nwout);
  xStand(0) = 0.08;
  xStand(1) = 0.08 + 0.04;
  xStand(2) = -0.76;
  xStand(3) = 0.08;
  xStand(4) = -0.08 + 0.04;
  xStand(5) = -0.76;
  // wbc init
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  // upper_joints_x_a = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  // upper_joints_x_d = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);
  // upper_joints_x_c = Eigen::MatrixXd::Zero(3, adam_upper_actor_num);

  rTorsoCmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);

  // qArmCmd = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);

  return;
}

void UniGait::onEnter() {
  std::cout << "UniGait" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;
  robotdata->addlog("UniGait::onEnter()");
  // gait para init-----------------------------//
  // robotdata->stance_index = 1;
  if (robotdata->stance_index == 1) {
    robotdata->touch_index = 1;
    robotdata->touch_index_pre = 1;
  } else {
    robotdata->touch_index = 3;
    robotdata->touch_index_pre = 3;
  }

  robotdata->time = 0.0;     // run time (sec) for current behavior
  robotdata->t = 0.0;        // passed-time (sec), since last Phase switch
  robotdata->t_switch = 0.0; // last TD time
  robotdata->s = 0.0; // the time-variant parameter: s = clamp(t/Ts, 0, 1)
  // double T{0.4};
  // double Td{0.4};
  // double Td2{0.3};
  // double Tc{0.02};
  robotdata->step = 0;

  robotdata->foot_odometer.setZero();
  robotdata->odometer.setZero();
  robotdata->odometer_d.setZero();
  robotdata->odometer_avg.setZero();

  robotdata->vy = 0.;

  //-----------------------------------------//
  q_c = Eigen::VectorXd::Zero(18);
  q_dot_c = Eigen::VectorXd::Zero(18);
  q_ddot_c = Eigen::VectorXd::Zero(18);
  tau_c = Eigen::VectorXd::Zero(18);
  q_ini = robotdata->q_a;

  // intialize bezier parameter
  M = 6;
  Nout = 12;
  stIndex = 0;
  tStepPre = 0.0;
  init_pos = Eigen::VectorXd::Zero(Nout);

  // WorkSpace Trajectory
  firstFlag = true;
  Nwout = 6;
  xInit = Eigen::VectorXd::Zero(Nwout);
  xDotInit = Eigen::VectorXd::Zero(Nwout);
  xDDotInit = Eigen::VectorXd::Zero(Nwout);
  xEnd = Eigen::VectorXd::Zero(Nwout);
  xDotEnd = Eigen::VectorXd::Zero(Nwout);
  xCmd = Eigen::VectorXd::Zero(Nwout);
  xDotCmd = Eigen::VectorXd::Zero(Nwout);
  xDDotCmd = Eigen::VectorXd::Zero(Nwout);
  fCmd = Eigen::VectorXd::Zero(Nwout);

  qCmd = Eigen::VectorXd::Zero(Nout);
  qDotCmd = Eigen::VectorXd::Zero(Nout);

  vCmd = 0.0;
  xStand = Eigen::VectorXd::Zero(Nwout);
  // xStand(0) = 0.08;
  // xStand(1) = 0.08 + 0.04;
  // xStand(2) = -0.76;
  // xStand(3) = 0.08;
  // xStand(4) = -0.08 + 0.04;
  // xStand(5) = -0.76;
  xStand = robotdata->xStand_init;
  xCmd = xStand;

  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand, rTorsoCmd, rFootCmd, qCmd, qDotCmd, firstFlag);
  // wkSpace2Joint(xStand, qCmd, qDotCmd, firstFlag);
  q_init.setZero();
  q_init.tail(12) = qCmd;

  gait->setevent("");
  // gait plan
  gait_plan = new uniPlan();
  // init plan
  qCmd = robotdata->q_c.segment(6, 12);
  qDotCmd = robotdata->q_dot_c.segment(6, 12);

  // if (robotdata->stance_index == 1) {
  //   robotdata->pFootb_tgt = xStand.tail(3);
  //   robotdata->vFootb_tgt.setZero();

  //   robotdata->pTorso_tgt = -xStand.head(3);
  //   robotdata->vTorso_tgt.setZero();
  // } else {
  //   robotdata->pFootb_tgt = xStand.head(3);
  //   robotdata->vFootb_tgt.setZero();

  //   robotdata->pTorso_tgt = -xStand.tail(3);
  //   robotdata->vTorso_tgt.setZero();
  // }

  if (robotdata->stance_index == 1) {
    robotdata->pFootb_ref = xStand;
    robotdata->vFootb_ref.setZero();

    robotdata->pTorso_ref = -xStand.tail(3);
    robotdata->vTorso_tgt.setZero();
  } else {
    robotdata->pFootb_ref = xStand;
    robotdata->vFootb_ref.setZero();

    robotdata->pTorso_ref = -xStand.head(3);
    robotdata->vTorso_tgt.setZero();
  }

  robotdata->time = 0.0;

  gait_plan->init(qCmd, qDotCmd, xStand, robotdata);
  // wbc
  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  // int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  // com_x_a =
  // gait->robot_controller_._robot_data->task_card_set[com_task_id]->X_a.block(0,0,3,6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  // upper_joints_x_a =
  //     robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
  //                                                                     adam_upper_actor_num);
  // test
  // com_x_a.row(1).setZero();
  // com_x_d = com_x_a;
  // target
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  // upper_joints_x_d = upper_joints_x_a;
  // upper_joints_x_d.row(2).setZero();
  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  // upper_joints_x_c = upper_joints_x_a;

  gait->W2S = false;
  robotdata->prestand = false;
  prestand_step = 0;
  // joystick zero
  robotdata->vyaw = 0.0;
  robotdata->vCmd.setZero();

  // robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  // robotdata->task_card_set[robotdata->upper_joints_id]->weight =
  //     100.0 * Eigen::VectorXd::Ones(adam_upper_actor_num);
  robotdata->q_factor << .2 * Eigen::VectorXd::Ones(12);
  robotdata->q_dot_factor << .2 * Eigen::VectorXd::Ones(12);
  // robotdata->pArm_tgt << 0.5, 0.5;
  // robotdata->vArm_tgt << 0.0, 0.0;

  if (robotdata->carryBoxState == 2) {
    robotdata->carryBoxState = 0;
    robotdata->carryBoxFirstStand = true;
  }
}

void UniGait::run() {
  // std::cout << "Walk::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  if (robotdata->q_a(10) < -0.96)
    std::cout << "left ankle pitch:" << robotdata->q_a(10) << std::endl;
  if (robotdata->q_a(16) < -0.96)
    std::cout << "right ankle pitch:" << robotdata->q_a(16) << std::endl;
  // run
  // update task actual state
  robotdata->time = timer;
  gait->robot_controller_._estimation_operator->task_state_update_x_a_walk(
      robotdata);

  // q_c = q_init;
  // q_dot_c.setZero();
  tau_c.setZero();
  // //----------------joint space---------------------//
  // gait plan
  gait_plan->locoPlan(robotdata, qCmd, qDotCmd);
  // // robot arm
  // robotdata->arm_style =
  //     0.005 * robotdata->arm_style_d + 0.995 * robotdata->arm_style;
  // Eigen::VectorXd arm_run = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num);
  // if(adam_type==ADAM_TYPE::AdamLite){
  //   arm_run << M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0,
  //             M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0;
  // }else if(adam_type==ADAM_TYPE::AdamStandard){
  //   arm_run << M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0, 0.0,
  //             M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0, 0.0;
  // }else if(adam_type==ADAM_TYPE::StandardPlus23){
  //   arm_run << M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0,
  //             M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0;
  // }else if(adam_type==ADAM_TYPE::StandardPlus29){
  //   arm_run << M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0,
  //             M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0;
  // }else if(adam_type==ADAM_TYPE::StandardPlus53){
  //   arm_run << M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0,
  //             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //             M_PI / 20.0, 0.0, 0.0, -M_PI / 3.0, 0.0, 0.0, 0.0,
  //             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // }
  // Eigen::VectorXd qArmCmd_temp = Eigen::VectorXd::Zero(8); 
  // armPolyJoint(robotdata->pArm_tgt, qArmCmd_temp);
  // qArmCmd.setZero();
  // qArmCmd.head(4) = qArmCmd_temp.head(4);vyaw
  // }
  // upper_joints_x_c.row(0) = robotdata->q_c.tail(adam_upper_actor_num);
  if (robotdata->carryBoxState == 1) {
    Eigen::VectorXd Q_arm_d = Eigen::VectorXd::Zero(adam_upper_except_waist_actor_num, 1);
    if(adam_type==ADAM_TYPE::AdamLite){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0,
                -0.5, -0.03, 0.3, -1.0;
    }else if(adam_type==ADAM_TYPE::AdamStandard){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1, 0.0,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1, 0.0;
    }else if(adam_type==ADAM_TYPE::StandardPlus23){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0,
                -0.5, -0.03, 0.3, -1.0;
    }else if(adam_type==ADAM_TYPE::StandardPlus29){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1;
    }else if(adam_type==ADAM_TYPE::StandardPlus53){
      Q_arm_d << -0.5, 0.03, -0.3, -1.0, 0.4, -0.1, -0.1,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -0.5, -0.03, 0.3, -1.0, -0.4, -0.1, -0.1,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    robotdata->q_c.tail(adam_upper_except_waist_actor_num) = Q_arm_d;
    upper_joints_x_c.row(0) = robotdata->q_c.tail(adam_upper_actor_num).transpose();
  }
  // std::cout << "robotdata->q_c.tail(8): " <<
  // robotdata->q_c.tail(8).transpose() << std::endl;

  // wbc solve
  // control
  body_x_c = robotdata->task_card_set[robotdata->body_task_id]->X_a;
  body_x_c.row(0).head(3).setZero();
  // body_x_c(0, 0) = -0.15 * robotdata->avg_vx * robotdata->vyaw;
  body_x_c.row(0).head(3) = robotdata->eulerTorso_ref.transpose();
  body_x_c.row(1).head(3).setZero();
  // joystick yaw
  if ((fabs(robotdata->vCmd_joystick(2)) > 0.1)) {
    if (fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw) > 0.04) {
      robotdata->vyaw += 0.04 *
                         (robotdata->vCmd_joystick(2) - robotdata->vyaw) /
                         fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw);
    } else {
      robotdata->vyaw = robotdata->vCmd_joystick(2);
    }
  } else {
    if (fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw) > 0.1) {
      robotdata->vyaw += 0.1 * (robotdata->vCmd_joystick(2) - robotdata->vyaw) /
                         fabs(robotdata->vCmd_joystick(2) - robotdata->vyaw);
    } else {
      robotdata->vyaw = robotdata->vCmd_joystick(2);
    }
  }

  body_x_c(1, 2) = robotdata->vyaw;
  body_x_c.row(2).setZero();
  body_x_c.block(0, 3, 1, 3) = robotdata->pTorso_ref.transpose();
  body_x_c.block(1, 3, 1, 3) = robotdata->vTorso_ref.transpose();

  left_foot_x_c = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
  left_foot_x_c.block(0, 3, 1, 3) =
      robotdata->pFootb_ref.head(3).transpose() +
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3);
  left_foot_x_c.block(1, 3, 1, 3) =
      robotdata->vFootb_ref.head(3).transpose() +
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3);
  left_foot_x_c.row(2).setZero();

  right_foot_x_c = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
  right_foot_x_c.block(0, 3, 1, 3) =
      robotdata->pFootb_ref.tail(3).transpose() +
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3);
  right_foot_x_c.block(1, 3, 1, 3) =
      robotdata->vFootb_ref.tail(3).transpose() +
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3);
  right_foot_x_c.row(2).setZero();
  // if (robotdata->stance_index == 0) {
  //   left_foot_x_c = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
  //   left_foot_x_c.row(2).setZero();
  //   right_foot_x_c.setZero();
  //   right_foot_x_c.block(0, 3, 1, 3) =
  //       robotdata->pFootb_tgt.transpose() +
  //       robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1,
  //       3);
  //   right_foot_x_c.block(1, 3, 1, 3) =
  //       robotdata->vFootb_tgt.transpose() +
  //       robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1,
  //       3);
  // } else {
  //   right_foot_x_c = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
  //   right_foot_x_c.row(2).setZero();
  //   left_foot_x_c.setZero();
  //   left_foot_x_c.block(0, 3, 1, 3) =
  //       robotdata->pFootb_tgt.transpose() +
  //       robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1,
  //       3);
  //   left_foot_x_c.block(1, 3, 1, 3) =
  //       robotdata->vFootb_tgt.transpose() +
  //       robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1,
  //       3);
  // }
  // gait data update

  // plan_data
  robotdata->task_card_set[robotdata->body_task_id]->X_d = body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d = left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d = right_foot_x_c;
  // robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, adam_upper_actor_num) =
  //     upper_joints_x_c;
  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);

  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  // tau_lb << 82.5, 82.5, 100.0, 100.0, 32.0, 32.0, 82.5, 82.5, 100.0,
  // 100.0, 32.0, 32.0; tau_lb = -tau_lb; tau_ub << 82.5, 82.5, 100.0,
  // 100.0, 32.0, 32.0, 82.5, 82.5, 100.0, 100.0, 32.0, 32.0; GRF_ub
  // << 16.6,32.0,10.0,200.0,200.0,450.0,16.6,32.0,10.0,200.0,200.0,450.0;
  // GRF_lb = -GRF_ub;
  // GRF_lb(5) = 0.0;
  // GRF_lb(11) = 0.0;
  // Fref  = robotdata->contactforce;
  tau_lb = robotdata->tau_lb;
  tau_ub = robotdata->tau_ub;
  GRF_lb = robotdata->GRF_lb;
  GRF_ub = robotdata->GRF_ub;

  Fref = robotdata->contactforce;
  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);

  // set control command
  robotdata->q_c.segment(6, 12) = qCmd;
  robotdata->q_dot_c.segment(6, 12) = qDotCmd;

  timer += robotdata->dt;
  // updata event
  if (gait->current_fsmstate_command == "gotoZ2S") {
    gait->W2S = true;
  } else {
    robotdata->prestand = false;
    prestand_step = 0;
    gait->W2S = false;
  }

  robotdata->standcmd = gait->W2S;
  double e1 = std::exp(robotdata->lambda * 0.5 * robotdata->T_leg);
  double e2 = std::exp(-robotdata->lambda * 0.5 * robotdata->T_leg);
  double sigma2 = robotdata->lambda *
                  std::tanh(0.5 * 0.5 * robotdata->T_leg * robotdata->lambda);
  double py0 = 0.13; // 0.7*robotdata->vy_offset/sigma2;

  if (gait->W2S == true) {
    if ((robotdata->t_leg(0) < 0.5 * robotdata->dt) &&
        (fabs(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0, 3) -
              robotdata->task_card_set[robotdata->right_foot_id]->X_a(0, 3)) <
         0.025) &&
        ((fabs(-robotdata->q_a(1) - py0) < 0.06 &&
          robotdata->stance_index == 0)) &&
        robotdata->step > 0) {
      gait->setevent("gotoStand");
      gait->W2S = false;
    }
  } else {
    gait->setevent("");
  }
  // std::cout<<"W2S: "<<gait->W2S<<std::endl;
  // std::cout<<"prestand: "<<robotdata->prestand<<std::endl;
  if (robotdata->step_calibration_flag) {
    // std::cout << "robotdata->step_calibration_flag: " << robotdata->step_calibration_flag << std::endl;
    if (stance_flag != robotdata->stance_index) {
      // std::cout << "once" << std::endl;
      stance_flag = robotdata->stance_index;
      Eigen::VectorXd x_err = robotdata->task_card_set[robotdata->left_foot_id]
                                  ->X_a.row(0)
                                  .transpose() -
                              robotdata->task_card_set[robotdata->right_foot_id]
                                  ->X_a.row(0)
                                  .transpose();
      // std::cout << "x_err: " << x_err << std::endl;
      if (stance_flag == 1) {
        robotdata->step_calibration_x_offset = x_err(3) * -1;
      } else if (stance_flag == 0) {
        robotdata->step_calibration_x_offset = x_err(3);
      }
      robotdata->step_calibration_flag2 = true;
    }
  }

  // robotdata->dataL(19) = 6;
  // robotdata->dataL(20) = robotdata->time;
  // robotdata->dataL.segment(21, 29) = robotdata->q_a;
  // robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  // robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  // robotdata->dataL.segment(108, 12) = robotdata->grf;
  // robotdata->dataL.segment(120, 29) = robotdata->q_c;
  // robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  // robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  // robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  // robotdata->dataL.segment(219, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  // robotdata->dataL.segment(225, 6) =
  //     robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  // robotdata->dataL.segment(231, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(0)
  //         .transpose();
  // robotdata->dataL.segment(237, 6) =
  //     robotdata->task_card_set[robotdata->right_foot_id]
  //         ->X_a.row(1)
  //         .transpose();

  // robotdata->dataL.segment(243, 3) = robotdata->vCmd;

  // robotdata->dataL(246) = robotdata->step;
  // robotdata->dataL(247) = robotdata->stance_index;
  // robotdata->dataL(248) = robotdata->phase;
  // robotdata->dataL(249) = robotdata->st_leg(0);
  // robotdata->dataL(250) = robotdata->st_leg(1);
  // robotdata->dataL(251) = robotdata->t_leg(0);
  // robotdata->dataL(252) = robotdata->t_leg(1);
}

void UniGait::onExit() {
  // std::cout << "Walk::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  delete gait_plan;
  auto robotdata = gait->robot_controller_._robot_data;
  if (robotdata->stance_index == 0) {
    robotdata->xStand_init.head(3) = -robotdata->pTorso_ref;
    robotdata->xStand_init.tail(3) = robotdata->pFootb_ref.tail(3);
  } else {
    robotdata->xStand_init.head(3) = robotdata->pFootb_ref.head(3);
    robotdata->xStand_init.tail(3) = -robotdata->pTorso_ref;
  }
  robotdata->addlog("UniGait::onExit()");
  
  // save com_x to file
  int num;
  for(num=0;num<2;num++){ 
    QString path;
    if(num==0){path = "./Sources/config/pnc_config_sim.json";}
    if(num==1){path = "./Sources/config/pnc_config.json";}
    QFile loadFile(path);
    if (!loadFile.open(QIODevice::ReadOnly)) {
      qDebug() << "could't open projects json";
      return;
    }
    QByteArray allData = loadFile.readAll();
    loadFile.close();
    QString jsonString(allData);
    // Modify the com_x parameter
    double new_com_x_value = robotdata->poffset_x; // Replace with the desired new value
    QString new_com_x_value_str = QString::number(new_com_x_value);
    // Use regular expression to find and replace the com_x value
    QRegularExpression re1("\"com_x\"\\s*:\\s*\\d+(\\.\\d+)?");
    jsonString.replace(re1, "\"com_x\": " + new_com_x_value_str);
    // Modify the carrybox_com_x parameter
    double new_carrybox_com_x_value = robotdata->carrybox_poffset_x; // Replace with the desired new value
    QString new_carrybox_com_x_value_str = QString::number(new_carrybox_com_x_value);
    // Use regular expression to find and replace the carrybox_com_x value
    QRegularExpression re2("\"carrybox_com_x\"\\s*:\\s*\\d+(\\.\\d+)?");
    jsonString.replace(re2, "\"carrybox_com_x\": " + new_carrybox_com_x_value_str);
    // Write the modified JSON string back to the file
    QFile saveFile(path);
    if (!saveFile.open(QIODevice::WriteOnly)) {
      qDebug() << "could't open projects json for writing";
      return;
    }
    saveFile.write(jsonString.toUtf8());
    saveFile.close();
  }
}

// Zero //////////////////////////////////////////////
void Stop::init() {
  // std::cout << "Stop::init() " << std::endl;
  stateName = "Stop";
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  gait->robot_controller_._robot_data->addlog("Stop::init()");
  // addEventTrans("startstand", "Stand");
  // addEventTrans("gotoSingleLegTest", "SingleLegTest");
  // addEventTrans("gotoSingleSwingTest", "SingleSwingTest");
  // addEventTrans("gotoSquat", "Squat");
  // addEventTrans("gotoExcitingTrajectory", "ExcitingTrajectory");
  // addEventTrans("gotoMotorMotion", "MotorMotion");
  // addEventTrans("gotoStand", "Stand");

  addEventTrans("gotoZero", "Zero");
  timer = 0.0;
  //
  qa = Eigen::MatrixXd::Zero(18, 1);
  qa_dot = Eigen::MatrixXd::Zero(18, 1);
  qd = Eigen::MatrixXd::Zero(18, 1);
  qd_dot = Eigen::MatrixXd::Zero(18, 1);
  //
  avr_v = 0.1;
  totaltime = 0;

  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand = Eigen::VectorXd::Zero(6);
  return;
}

void Stop::onEnter() {
  std::cout << "Stop" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  gait->robot_controller_._robot_data->fsmname = stateName;
  timer = 0.0;
  gait->robot_controller_._robot_data->addlog("Stop::onEnter()");
  // update
  qa = gait->robot_controller_._robot_data->q_a;
  qa_dot = gait->robot_controller_._robot_data->q_dot_a;

  // init position
  xStand(0) = -0.01;
  xStand(1) = 0.12;
  xStand(2) = -0.7;
  xStand(3) = -0.01;
  xStand(4) = -0.12;
  xStand(5) = -0.7;
  footflag = true;
  wkSpace2Joint(xStand, Eigen::VectorXd::Zero(6), qCmd, qDotCmd, footflag);
  qd.setZero();
  qd = qa;

  // qd = qa;
  // qd.block(12,0,6,1) << 0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6, 0.0;
  // qd << 0., 0., 0., 0., 0., 0.,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6.0, 0.0,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6.0, 0.0;
  // qd << 0., 0., 0., 0., 0., 0.,
  //       0.220000, 0.000000 ,-0.260000 ,1.000000, -0.170000, 0.0,
  //       0.0, 0.0, -M_PI/6.0, M_PI/3.0, -M_PI/6, 0.0;

  qd_dot.setZero();
  // init total time
  avr_v = 0.2;
  double deltq = max(abs((qd - qa).block(6, 0, 12, 1).maxCoeff()),
                     abs((qd - qa).block(6, 0, 12, 1).minCoeff()));
  totaltime = deltq / avr_v;
  gait->setevent("");
  robotdata->q_c = robotdata->q_a;
}

void Stop::run() {
  // std::cout << "Stop::run() " <<std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  // std::cout<<"error!"<<std::endl;
  auto robotdata = gait->robot_controller_._robot_data;
  // run
  // Eigen::MatrixXd O = Eigen::MatrixXd::Zero(18, 1);
  // double delth = 0.07;
  // double T_squad = 2.0;
  // double omega = 2.0 * M_PI / T_squad;
  // double n = 30;
  // Eigen::VectorXd FR = Eigen::VectorXd::Zero(6);
  // Eigen::VectorXd FL = Eigen::VectorXd::Zero(6);
  // if (timer < totaltime) {
  //   gait->FifthPoly(qa, qa_dot, O, qd, qd_dot, O, totaltime, timer,
  //   robotdata->q_c, robotdata->q_dot_c,
  //                   robotdata->q_ddot_c);
  //   robotdata->tau_c.setZero();
  // } else {
  //   robotdata->q_c.block(6, 0, 12, 1) = qd.block(6, 0, 12, 1);
  //   robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
  //   robotdata->tau_c.setZero();
  // }
  // 阻尼模式
  robotdata->tau_c.setZero();

  robotdata->q_dot_c = -0.0 * robotdata->q_dot_a;
  // PD gains
  robotdata->q_factor = 0.01 * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
  robotdata->q_dot_factor = 1.5 * Eigen::VectorXd::Ones(robotdata->ndof - 6, 1);
  // robotdata->q_dot_factor.block(6, 0, 12, 1) =
  //     5.0 * Eigen::VectorXd::Ones(12, 1);

  // arm
  // robotdata->q_factor.block(21, 0, 8, 1) = Eigen::VectorXd::Ones(8, 1);
  robotdata->q_dot_factor.block(15, 0, 8, 1) =
      0.5 * Eigen::VectorXd::Ones(8, 1);

  // waist is normal
  robotdata->q_factor.block(12, 0, 3, 1) = 0.5 * Eigen::VectorXd::Ones(3, 1);
  robotdata->q_dot_factor.block(12, 0, 3, 1) =
      0.8 * Eigen::VectorXd::Ones(3, 1);

  // std::cout<<"error!"<<std::endl;
  // robotdata->q_c.block(0,0,6,1).setZero();
  // robotdata->q_dot_c.block(0,0,6,1).setZero();
  // robotdata->q_a.block(0,0,6,1).setZero();
  // robotdata->q_dot_a.block(0,0,6,1).setZero();
  // RigidBodyDynamics::Math::VectorNd CG =
  // RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof, 1);
  // RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model),robotdata->q_a,
  // robotdata->q_dot_a,CG); robotdata->tau_c = CG;

  timer += robotdata->dt;
  // std::cout<<"error!"<<std::endl;
  // updata event
  // if (timer < totaltime) {
  //   gait->setevent("");
  //   // std::cout<<"totaltime: "<<totaltime<<std::endl;
  // } else {
  //   // gait->setevent("gotoSingleSwingTest");
  //   // gait->setevent("gotoSquat");
  //   // // gait->setevent("gotoExcitingTrajectory");
  //   // if(timer < totaltime + 10.0){
  //   //     gait->setevent("");
  //   //     // std::cout<<"prepare to gotoStand"<<std::endl;
  //   //     std::cout<<"prepare to gotoWalk"<<std::endl;
  //   // }else{
  //   //     // gait->setevent("gotoStand");
  //   //     gait->setevent("gotoWalk");
  //   // }
  // }
  // log data
  robotdata->dataL(19) = 10;
  robotdata->dataL(20) = timer;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();
  // std::cout<<"error?"<<std::endl;
}

void Stop::onExit() {
  // std::cout << "Stop::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand;
  robotdata->addlog("Stop::onExit()");
}

// Dual2Single //////////////////////////////////////////////////////////////
void Dual2Single::init() {
  // std::cout << "Dual2Single::init() " << std::endl;
  stateName = "Dual2Single";
  addEventTrans("gotoSingleStand", "SingleStand");
  addEventTrans("gotoStop", "Stop");

  timer = 0.0;

  // init
  com_x_a = Eigen::MatrixXd::Zero(3, 6);
  com_x_d = Eigen::MatrixXd::Zero(3, 6);
  com_x_c = Eigen::MatrixXd::Zero(3, 6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, 11);

  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand_init = Eigen::VectorXd::Zero(6);
  xStand_tgt1 = Eigen::VectorXd::Zero(6);
  xStand_tgt2 = Eigen::VectorXd::Zero(6);
  xStand_cmd = Eigen::VectorXd::Zero(6);
  xCoM_init = Eigen::VectorXd::Zero(6);
  rTorso_init = Eigen::VectorXd::Zero(6);
  rTorso_tgt = Eigen::VectorXd::Zero(6);
  rTorso_cmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);
  leftFoot_init = Eigen::VectorXd::Zero(6);
  q_factor_init = Eigen::VectorXd::Zero(23);
  q_dot_factor_init = Eigen::VectorXd::Zero(23);

  CoM_delta = Eigen::VectorXd::Zero(3);
  leftFootAirDelta = Eigen::VectorXd::Zero(3);

  qArmCmd = Eigen::VectorXd::Zero(8);
  leftFootYawInit = 0.0;
  return;
}

void Dual2Single::onEnter() {
  std::cout << "To SingleStand" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  timer = 0.0;

  // update task actual state
  robotdata->stance_index = 1;
  robotdata->inSingleStand = true;
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  com_x_a = gait->robot_controller_._robot_data->task_card_set[com_task_id]
                ->X_a.block(0, 0, 3, 6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
                                                                      11);

  avr_v = 0.1;
  totaltime = 3.0;

  // target
  com_x_d = com_x_a;
  com_x_d.row(1).setZero();
  com_x_d.row(2).setZero();
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  // control
  com_x_c = com_x_a;
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;

  // init position
  rTorso_init.head(3) = robotdata->q_a.segment(3, 3);
  rTorso_init.tail(3) = rTorso_init.head(3);
  rTorso_cmd.setZero();
  rTorso_tgt << 0.02, 0.0, 0.0, 0.02, 0.0, 0.0;
  leftFoot_init = left_foot_x_a.row(0).transpose();
  leftFootAirDelta = Vector3d(0.0, 0.24, 0.1) - leftFoot_init.tail(3);
  xStand_init = robotdata->xStand_init;
  xStand_cmd = xStand_init;
  xCoM_init = com_x_a.row(0).transpose();
  CoM_delta = xCoM_init.tail(3);
  CoM_delta(0) -= robotdata->com_x_offset;
  // CoM_delta(1) -= 0.01;
  CoM_delta(2) = 0.0;
  leftFootYawInit = robotdata->q_c(8);

  xStand_tgt1 << 0.0, 0.24 + 0.005, -0.9, 0.0, 0.005, -0.9;
  xStand_tgt2 << 0.0, 0.24 + 0.005, -0.8, 0.0, 0.005, -0.9;
  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand_init, rTorso_init, rFootCmd, qCmd, qDotCmd, first_flag);
  armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd);
  //    wkSpace2Joint(xStand,qCmd,qDotCmd,first_flag);

  // init
  //  WBC weights and constraints
  // robotdata->task_card_set[robotdata->body_task_id]->weight
  // << 50., 50., 50., 1., 1., 1.;
  // robotdata->task_card_set[robotdata->com_task_id]->weight << 10., 10., 10.,
  // 100., 100., 100.;
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      100.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->upper_joints_id]->weight =
      100.0 * Eigen::VectorXd::Ones(11);

  // q_factor
  q_factor_init = robotdata->q_factor;
  q_dot_factor_init = robotdata->q_dot_factor;

  gait->setevent("");
}

void Dual2Single::run() {
  // std::cout << "Dual2Single::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // start: plan
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);

  totaltime = 4.0;
  double omega = M_PI / totaltime * 2.0;
  if (timer <= totaltime / 2.0) {
    // kinematics
    Eigen::VectorXd body_delta1 = xStand_tgt1 - xStand_init;
    Eigen::VectorXd rpy_delt = rTorso_tgt - rTorso_init;
    xStand_cmd = xStand_init + body_delta1 * (1 - cos(omega * timer)) / 2.0;
    rTorso_cmd = rTorso_init + rpy_delt * (1 - cos(omega * timer)) / 2.0;
    wkSpace2Joint(xStand_cmd, rTorso_cmd, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;

    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_cmd.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_cmd.tail(3).transpose();
    body_x_d.block(1, 3, 1, 3) =
        -body_delta1.transpose() * omega * sin(omega * (timer)) / 2.0;
    body_x_d.block(2, 3, 1, 3) =
        -body_delta1.transpose() * omega * omega * cos(omega * (timer)) / 2.0;

    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d.block(0, 3, 1, 3) =
        xCoM_init.tail(3).transpose() -
        CoM_delta.transpose() * (1 - cos(omega * timer)) / 2.0;
    com_x_d.block(1, 3, 1, 3) =
        -CoM_delta.transpose() * omega * sin(omega * (timer)) / 2.0;
    com_x_d.block(2, 3, 1, 3) =
        -CoM_delta.transpose() * omega * omega * cos(omega * (timer)) / 2.0;

    // left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(0) = leftFoot_init.transpose();
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    double GRF_ratio = 0.1;
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_ub.head(6) =
        (1 - GRF_ratio) * (1 - (timer / totaltime * 2.0)) * GRF_ub.head(6) +
        GRF_ratio * GRF_ub.head(6);
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 0.0;
    GRF_lb(11) = 20.0;

    robotdata->q_factor = q_factor_init;
    robotdata->q_dot_factor = q_dot_factor_init;
  } else if (timer < totaltime) {
    // kinematics
    double timer2 = timer - totaltime / 2.0;
    Eigen::VectorXd body_delta2 = xStand_tgt2 - xStand_tgt1;
    xStand_cmd = xStand_tgt1 + body_delta2 * (1 - cos(omega * timer2)) / 2.0;
    // rFootCmd(2) = leftFootYawInit * (1 + cos(omega * timer2)) / 2.0;
    wkSpace2Joint(xStand_cmd, rTorso_tgt, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;

    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_tgt.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_cmd.tail(3).transpose();
    body_x_d.block(1, 3, 1, 3) =
        -body_delta2.transpose() * omega * sin(omega * (timer2)) / 2.0;
    body_x_d.block(2, 3, 1, 3) =
        -body_delta2.transpose() * omega * omega * cos(omega * (timer2)) / 2.0;

    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d.block(0, 3, 1, 3) =
        xCoM_init.tail(3).transpose() - CoM_delta.transpose();
    com_x_d.row(1).setZero();
    com_x_d.row(2).setZero();

    left_foot_x_d.block(0, 0, 1, 3) << 0.0, 0.0, rFootCmd(2);
    left_foot_x_d.block(0, 3, 1, 3) =
        leftFoot_init.tail(3).transpose() +
        leftFootAirDelta.transpose() * (1 - cos(omega * timer2)) / 2.0;
    left_foot_x_d.block(1, 3, 1, 3) =
        leftFootAirDelta.transpose() * omega * sin(omega * (timer2)) / 2.0;
    left_foot_x_d.block(2, 3, 1, 3) = leftFootAirDelta.transpose() * omega *
                                      omega * cos(omega * (timer2)) / 2.0;
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    robotdata->task_card_set[robotdata->left_foot_id]->weight =
        (1000.0 - 950.0 * timer2 / totaltime * 2.0) * Eigen::VectorXd::Ones(6);
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 0.0;
    GRF_lb(11) = 20.0;
    GRF_lb.head(6).setZero();
    GRF_ub.head(6).setZero();

    Eigen::VectorXd q_factor_single_stand = q_factor_init;
    Eigen::VectorXd q_dot_factor_single_stand = q_dot_factor_init;
    q_factor_single_stand.segment(0, 6) = 0.3 * Eigen::VectorXd::Ones(6);
    q_dot_factor_single_stand.segment(0, 6) = 0.3 * Eigen::VectorXd::Ones(6);
    q_factor_single_stand.segment(6, 6) = 0.1 * Eigen::VectorXd::Ones(6);
    q_dot_factor_single_stand.segment(6, 6) = 0.1 * Eigen::VectorXd::Ones(6);

    robotdata->q_factor =
        q_factor_single_stand * timer2 / (totaltime / 2.0) +
        q_factor_init * (totaltime / 2.0 - timer2) / (totaltime / 2.0);
    robotdata->q_dot_factor =
        q_dot_factor_single_stand * timer2 / (totaltime / 2.0) +
        q_dot_factor_init * (totaltime / 2.0 - timer2) / (totaltime / 2.0);
  } else {
    // kinematics
    wkSpace2Joint(xStand_tgt2, rTorso_tgt, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_tgt.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_tgt2.tail(3).transpose();

    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d.block(0, 3, 1, 3) =
        xCoM_init.tail(3).transpose() - CoM_delta.transpose();
    com_x_d.row(1).setZero();
    com_x_d.row(2).setZero();

    left_foot_x_d.row(0) = leftFoot_init.transpose();
    left_foot_x_d.block(0, 3, 1, 3) =
        leftFoot_init.tail(3).transpose() + leftFootAirDelta.transpose();
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 0.0;
    GRF_lb(11) = 20.0;
    GRF_lb.head(6).setZero();
    GRF_ub.head(6).setZero();
  }

  // set waist and arm qc
  robotdata->q_c.segment(18, 3) =
      (1.0 - timer / totaltime) *
      upper_joints_x_a.block(0, 0, 3, 1).transpose();
  robotdata->q_c.tail(8) = (1.0 - timer / totaltime) *
                               upper_joints_x_a.block(0, 3, 8, 1).transpose() +
                           (timer / totaltime) * qArmCmd;
  robotdata->q_dot_c.tail(11).setZero();
  upper_joints_x_d.setZero();
  upper_joints_x_d.row(0) = robotdata->q_c.tail(11);

  // end: plan
  // control
  com_x_c = com_x_d;
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  upper_joints_x_c = upper_joints_x_d;

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->com_task_id]->X_d.block(0, 0, 3, 6) =
      com_x_c;
  robotdata->task_card_set[robotdata->body_task_id]->X_d.block(0, 0, 3, 6) =
      body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d.block(0, 0, 3, 6) =
      left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d.block(0, 0, 3, 6) =
      right_foot_x_c;
  robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, 11) =
      upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);
  //    std::cout<<"hehe3"<<std::endl;

  Fref = robotdata->contactforce;
  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);

  // robotdata->q_factor.setZero();
  // robotdata->q_dot_factor.setZero();
  // update time
  timer += robotdata->dt;

  // log data
  robotdata->dataL(19) = 7;
  robotdata->dataL(20) = robotdata->time;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();
  // robotdata->tau_c = ratio*robotdata->tau_c;
  // updata event
  if (timer < totaltime) {
    gait->setevent("");
  } else {
    gait->setevent("gotoSingleStand");
  }
}

void Dual2Single::onExit() {
  // std::cout << "Dual2Single::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt2;
  robotdata->rCmd_joystick_last.setZero();
  robotdata->rCmd_joystick.setZero();
  // robotdata->stance_index = 0;
}

// SingleStand //////////////////////////////////////////////////////////////
void SingleStand::init() {
  // std::cout << "SingleStand::init() " << std::endl;
  stateName = "SingleStand";
  addEventTrans("gotoSingle2Dual", "Single2Dual");
  addEventTrans("gotoStop", "Stop");

  timer = 0.0;

  // init
  com_x_a = Eigen::MatrixXd::Zero(3, 6);
  com_x_d = Eigen::MatrixXd::Zero(3, 6);
  com_x_c = Eigen::MatrixXd::Zero(3, 6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, 11);

  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand = Eigen::VectorXd::Zero(6);
  xStand_tgt = Eigen::VectorXd::Zero(6);
  xStand_Zero = Eigen::VectorXd::Zero(6);
  xCoM_init = Eigen::VectorXd::Zero(6);
  q_factor_init = Eigen::VectorXd::Zero(23);
  q_dot_factor_init = Eigen::VectorXd::Zero(23);
  torso_d.setZero();
  rTorso_init = Eigen::VectorXd::Zero(6);
  rTorso_cmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);
  leftFoot_tgt = Eigen::VectorXd::Zero(6);
  qArmCmd = Eigen::VectorXd::Zero(8);
  return;
}

void SingleStand::onEnter() {
  std::cout << "SingleStand" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  timer = 0.0;

  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  com_x_a = gait->robot_controller_._robot_data->task_card_set[com_task_id]
                ->X_a.block(0, 0, 3, 6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
                                                                      11);

  totaltime = 1.0;

  // target
  com_x_d = com_x_a;
  com_x_d.row(1).setZero();
  com_x_d.row(2).setZero();
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  // control
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;
  // init position
  // rTorso_init.head(3) = robotdata->q_a.segment(3, 3);
  // rTorso_init.tail(3) = rTorso_init.head(3);
  rTorso_init << 0.02, 0.0, 0.0, 0.02, 0.0, 0.0;
  rTorso_cmd = rTorso_init;
  xCoM_init = com_x_a.row(0).transpose();
  xStand = robotdata->xStand_init;
  xStand_tgt = xStand;
  xStand_Zero = xStand_tgt;
  leftFoot_tgt << 0.0, 0.0, robotdata->q_c(7), 0.0, 0.24, 0.1;
  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand, rTorso_cmd, rFootCmd, qCmd, qDotCmd, first_flag);
  armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd);
  // wkSpace2Joint(xStand,qCmd,qDotCmd,first_flag);

  // init
  //  WBC weights and constraints
  // robotdata->task_card_set[robotdata->body_task_id]->weight
  // << 50., 50., 50., 1., 1., 1.;
  // robotdata->task_card_set[robotdata->com_task_id]->weight << 10., 10., 10.,
  // 100., 100., 100.;
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      100.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      50.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->upper_joints_id]->weight =
      100.0 * Eigen::VectorXd::Ones(11);

  // joystick zero
  robotdata->pCmd_joystick_last.setZero();
  robotdata->rCmd_joystick_last.setZero();
  // q_factor
  q_factor_init = robotdata->q_factor;
  q_dot_factor_init = robotdata->q_dot_factor;
  gait->setevent("");
}

void SingleStand::run() {
  // std::cout << "SingleStand::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // std::cout<<"hehe1"<<std::endl;
  // start: plan

  com_x_d.row(0) = xCoM_init.transpose();
  com_x_d.block(0, 3, 1, 2).setZero();
  com_x_d(0, 3) = robotdata->com_x_offset;
  // com_x_d(0, 4) = 0.005;
  body_x_d.setZero();

  // joystick x position and pitch
  double delt_ = 0.00025;
  if (fabs(robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0)) >
      delt_) {
    robotdata->pCmd_joystick_last(0) +=
        delt_ *
        (robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0)) /
        fabs(robotdata->pCmd_joystick(0) - robotdata->pCmd_joystick_last(0));
  } else {
    robotdata->pCmd_joystick_last(0) = robotdata->pCmd_joystick(0);
  }

  if (timer < totaltime) {
    xStand_tgt(0) =
        (totaltime - timer) / totaltime *
            (xStand_Zero(0) + 0.4 * robotdata->pCmd_joystick_last(0)) +
        timer / totaltime *
            (leftFoot_tgt(3) -
             robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 3));
  } else {
    xStand_tgt(0) =
        leftFoot_tgt(3) -
        robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 3);
  }
  xStand_tgt(3) = xStand_Zero(3) + 0.4 * robotdata->pCmd_joystick_last(0);

  if (timer < totaltime) {
    rTorso_cmd(1) =
        (totaltime - timer) / totaltime *
            (rTorso_init(1) + 2.0 * robotdata->pCmd_joystick_last(0)) +
        timer / totaltime *
            (-leftFoot_tgt(1) +
             robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 1));
  } else {
    rTorso_cmd(1) =
        -leftFoot_tgt(1) +
        robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 1);
  }
  rTorso_cmd(4) = rTorso_init(4) + 2.0 * robotdata->pCmd_joystick_last(0);
  robotdata->q_c(19) = 3.0 * robotdata->pCmd_joystick_last(0);

  // joystick y position and roll
  // double delt_y = 0.00025;
  double delt_y = 0.0;
  if (fabs(robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1)) >
      delt_y) {
    robotdata->pCmd_joystick_last(1) +=
        delt_y *
        (robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1)) /
        fabs(robotdata->pCmd_joystick(1) - robotdata->pCmd_joystick_last(1));
  } else {
    robotdata->pCmd_joystick_last(1) = robotdata->pCmd_joystick(1);
  }
  if (timer < totaltime) {
    xStand_tgt(1) =
        (totaltime - timer) / totaltime *
            (xStand_Zero(1) + 0.4 * robotdata->pCmd_joystick_last(1)) +
        timer / totaltime *
            (leftFoot_tgt(4) -
             robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 4));
  } else {
    xStand_tgt(1) =
        leftFoot_tgt(4) -
        robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 4);
  }
  xStand_tgt(4) = xStand_Zero(4) + 0.4 * robotdata->pCmd_joystick_last(1);

  if (timer < totaltime) {
    rTorso_cmd(0) =
        (totaltime - timer) / totaltime *
            (rTorso_cmd(0) - 2.0 * robotdata->pCmd_joystick_last(1)) +
        timer / totaltime *
            (-leftFoot_tgt(0) +
             robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 0));
  } else {
    rTorso_cmd(0) =
        -leftFoot_tgt(0) +
        robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 0);
  }
  rTorso_cmd(3) = rTorso_init(3) - 2.0 * robotdata->pCmd_joystick_last(1);
  robotdata->q_c(18) = -3.0 * robotdata->pCmd_joystick_last(1);

  // joystick z position
  if (fabs(robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2)) >
      delt_) {
    robotdata->pCmd_joystick_last(2) +=
        delt_ *
        (robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2)) /
        fabs(robotdata->pCmd_joystick(2) - robotdata->pCmd_joystick_last(2));
  } else {
    robotdata->pCmd_joystick_last(2) = robotdata->pCmd_joystick(2);
  }

  if (timer < totaltime) {
    xStand_tgt(2) =
        (totaltime - timer) / totaltime *
            (xStand_Zero(2) - 2.0 * robotdata->pCmd_joystick_last(2)) +
        timer / totaltime *
            (leftFoot_tgt(5) -
             robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 5));
  } else {
    xStand_tgt(2) =
        leftFoot_tgt(5) -
        robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 5);
  }
  xStand_tgt(5) = xStand_Zero(5) - robotdata->pCmd_joystick_last(2);
  com_x_d(0, 5) = xCoM_init(5) + robotdata->pCmd_joystick_last(2);

  // joystick Yaw
  double delt_yaw = 0.00025;
  if (fabs(robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2)) >
      delt_yaw) {
    robotdata->rCmd_joystick_last(2) +=
        delt_yaw *
        (robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2)) /
        fabs(robotdata->rCmd_joystick(2) - robotdata->rCmd_joystick_last(2));
  } else {
    robotdata->rCmd_joystick_last(2) = robotdata->rCmd_joystick(2);
  }
  // if (timer < totaltime) {
  //   rTorso_cmd(2) =  (totaltime - timer) / totaltime * (rTorso_init(2) +
  //   robotdata->rCmd_joystick_last(2)) +
  //       timer / totaltime * (-leftFoot_tgt(2) - robotdata->q_a(13));
  // } else {
  //   rTorso_cmd(2) = -leftFoot_tgt(2) - robotdata->q_a(13);
  // }
  rTorso_cmd(2) = rTorso_init(2) + robotdata->rCmd_joystick_last(2);
  rTorso_cmd(5) = rTorso_init(5) + robotdata->rCmd_joystick_last(2);
  robotdata->q_c(20) = robotdata->rCmd_joystick_last(2);

  // wkSpace2Joint(xStand_tgt, qCmd, qDotCmd, first_flag);
  wkSpace2Joint(xStand_tgt, rTorso_cmd, rFootCmd, qCmd, qDotCmd, first_flag);

  robotdata->q_c.block(6, 0, 12, 1) = qCmd;
  robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
  body_x_d.block(0, 0, 1, 3) = rTorso_cmd.tail(3).transpose();
  if (robotdata->stance_index == 1) {
    body_x_d.block(0, 3, 1, 3) = -xStand_tgt.tail(3).transpose();
  } else {
    body_x_d.block(0, 3, 1, 3) = -xStand_tgt.head(3).transpose();
  }

  left_foot_x_d.row(0) = leftFoot_tgt.transpose();
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
  right_foot_x_d.row(2).setZero();
  // robotdata->q_c.block(6,0,12,1) = qCmd;
  // robotdata->q_dot_c.block(6,0,12,1).setZero();
  robotdata->tau_c.setZero();

  // arm plan
  if (timer < 2.0) {
    robotdata->pArm_tgt(0) =
        (1.0 - timer / 2.0) * robotdata->pArm_tgt(0) + (timer / 2.0) * 0.42;
    robotdata->pArm_tgt(1) =
        (1.0 - timer / 2.0) * robotdata->pArm_tgt(1) + (timer / 2.0) * 0.42;
  } else {
    robotdata->pArm_tgt(0) = 0.42;
    robotdata->pArm_tgt(1) = 0.42;
  }

  // set waist and arm qc
  robotdata->q_c.tail(8) = qArmCmd;
  robotdata->q_dot_c.tail(11).setZero();
  upper_joints_x_d.setZero();
  upper_joints_x_d.row(0) = robotdata->q_c.tail(11).transpose();

  // foot motion plan

  // end: plan
  // control
  com_x_c = com_x_d;
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  upper_joints_x_c = upper_joints_x_d;

  // gait data update

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->com_task_id]->X_d.block(0, 0, 3, 6) =
      com_x_c;
  robotdata->task_card_set[robotdata->body_task_id]->X_d.block(0, 0, 3, 6) =
      body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d.block(0, 0, 3, 6) =
      left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d.block(0, 0, 3, 6) =
      right_foot_x_c;
  robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, 11) =
      upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);
  //    std::cout<<"hehe3"<<std::endl;
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  // to be add values
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);
  tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0, 200.0,
      60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
      18.0;
  tau_lb = -tau_ub;
  GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
      200.0, 700.0;
  GRF_lb = -GRF_ub;
  GRF_lb(5) = 0.0;
  GRF_lb(11) = 20.0;
  GRF_lb.head(6).setZero();
  GRF_ub.head(6).setZero();

  Fref = robotdata->contactforce;

  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);
  // std::cout<<"timer1: "<<timer<<std::endl;
  // std::cout<<"hehe4"<<std::endl;

  Eigen::VectorXd q_factor_single_stand = q_factor_init;
  Eigen::VectorXd q_dot_factor_single_stand = q_dot_factor_init;
  q_factor_single_stand.segment(0, 6) = 0.3 * Eigen::VectorXd::Ones(6);
  q_dot_factor_single_stand.segment(0, 6) = 0.3 * Eigen::VectorXd::Ones(6);
  q_factor_single_stand.segment(6, 6) = 0.05 * Eigen::VectorXd::Ones(6);
  q_dot_factor_single_stand.segment(6, 6) = 0.05 * Eigen::VectorXd::Ones(6);
  double deltt = 1.0;
  if (timer < deltt) {
    robotdata->q_factor = q_factor_single_stand * timer / deltt +
                          q_factor_init * (deltt - timer) / deltt;
    robotdata->q_dot_factor = q_dot_factor_single_stand * timer / deltt +
                              q_dot_factor_init * (deltt - timer) / deltt;
  } else {
    robotdata->q_factor = q_factor_single_stand;
    robotdata->q_dot_factor = q_dot_factor_single_stand;
  }

  // update time
  timer += robotdata->dt;
  // std::cout<<"timer2: "<<timer<<std::endl;
  // updata event
  // walk criteria
  // cp
  Eigen::Vector3d pOffset;
  pOffset[0] = 0.015;
  pOffset[1] = -0.000;
  pOffset[2] = -0.033; // stand has these paras too.
  double lamada = sqrt(9.81 / (fabs(torso_d(2)) + pOffset(2)));
  double cp_x =
      robotdata->task_card_set[robotdata->com_task_id]->X_a(0, 3) + pOffset(0) +
      robotdata->task_card_set[robotdata->body_task_id]->X_a(1, 3) / lamada;
  double cp_y =
      robotdata->task_card_set[robotdata->com_task_id]->X_a(0, 4) + pOffset(1) +
      robotdata->task_card_set[robotdata->body_task_id]->X_a(1, 4) / lamada;

  double delt_l = 0.05;
  // if(timer > robotdata->T)
  // {
  //     if((fabs(cp_x)>0.08)||(cp_y>(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)-delt_l))
  //         ||(cp_y<(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)+delt_l))){
  //         gait->setevent("gotoWalk");
  //         if(cp_y>(robotdata->task_card_set[robotdata->left_foot_id]->X_a(0,4)-delt_l)){
  //             robotdata->stance_index = 0;
  //         }
  //         if(cp_y<(robotdata->task_card_set[robotdata->right_foot_id]->X_a(0,4)+delt_l)){
  //             robotdata->stance_index = 1;
  //         }
  //     }
  // }
  // log data
  robotdata->dataL(19) = 8;
  robotdata->dataL(20) = robotdata->time;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();
  // robotdata->tau_c = ratio*robotdata->tau_c;
}

void SingleStand::onExit() {
  // std::cout << "SingleStand::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt;
  robotdata->rCmd_joystick_last.setZero();
  robotdata->rCmd_joystick.setZero();
  // robotdata->stance_index = 0;
}

// Single2Dual //////////////////////////////////////////////////////////////
void Single2Dual::init() {
  // std::cout << "Single2Dual::init() " << std::endl;
  stateName = "Single2Dual";
  addEventTrans("gotoStand", "Stand");
  addEventTrans("gotoStop", "Stop");

  timer = 0.0;

  // init
  com_x_a = Eigen::MatrixXd::Zero(3, 6);
  com_x_d = Eigen::MatrixXd::Zero(3, 6);
  com_x_c = Eigen::MatrixXd::Zero(3, 6);
  body_x_a = Eigen::MatrixXd::Zero(3, 6);
  body_x_d = Eigen::MatrixXd::Zero(3, 6);
  body_x_c = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  left_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_a = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_d = Eigen::MatrixXd::Zero(3, 6);
  right_foot_x_c = Eigen::MatrixXd::Zero(3, 6);
  upper_joints_x_a = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_d = Eigen::MatrixXd::Zero(3, 11);
  upper_joints_x_c = Eigen::MatrixXd::Zero(3, 11);

  avr_v = 0.1;
  totaltime = 0;
  qCmd = Eigen::VectorXd::Zero(12);
  qDotCmd = Eigen::VectorXd::Zero(12);
  xStand_init = Eigen::VectorXd::Zero(6);
  xStand_tgt = Eigen::VectorXd::Zero(6);
  xStand_cmd = Eigen::VectorXd::Zero(6);
  xCoM_init = Eigen::VectorXd::Zero(6);
  rTorso_init = Eigen::VectorXd::Zero(6);
  rTorso_tgt = Eigen::VectorXd::Zero(6);
  rTorso_cmd = Eigen::VectorXd::Zero(6);
  rFootCmd = Eigen::VectorXd::Zero(6);
  leftFoot_init = Eigen::VectorXd::Zero(6);
  q_factor_init = Eigen::VectorXd::Zero(23);
  q_dot_factor_init = Eigen::VectorXd::Zero(23);
  qArmCmd = Eigen::VectorXd::Zero(8);

  return;
}

void Single2Dual::onEnter() {
  std::cout << "To Stand" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  timer = 0.0;

  // update task actual state
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      gait->robot_controller_._robot_data);

  // record static start state
  // start position
  int com_task_id = gait->robot_controller_._robot_data->com_task_id;
  com_x_a = gait->robot_controller_._robot_data->task_card_set[com_task_id]
                ->X_a.block(0, 0, 3, 6);
  int body_task_id = gait->robot_controller_._robot_data->body_task_id;
  body_x_a = gait->robot_controller_._robot_data->task_card_set[body_task_id]
                 ->X_a.block(0, 0, 3, 6);
  int left_foot_task_id = gait->robot_controller_._robot_data->left_foot_id;
  left_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[left_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  int right_foot_task_id = gait->robot_controller_._robot_data->right_foot_id;
  right_foot_x_a =
      gait->robot_controller_._robot_data->task_card_set[right_foot_task_id]
          ->X_a.block(0, 0, 3, 6);
  upper_joints_x_a =
      robotdata->task_card_set[robotdata->upper_joints_id]->X_a.block(0, 0, 3,
                                                                      11);

  avr_v = 0.1;
  totaltime = 3.0;

  // target
  com_x_d = com_x_a;
  com_x_d.row(1).setZero();
  com_x_d.row(2).setZero();
  body_x_d = body_x_a;
  body_x_d.row(2).setZero();
  left_foot_x_d = left_foot_x_a;
  left_foot_x_d.row(2).setZero();
  right_foot_x_d = right_foot_x_a;
  right_foot_x_d.row(2).setZero();
  upper_joints_x_d = upper_joints_x_a;
  upper_joints_x_d.row(2).setZero();

  // control
  com_x_c = com_x_a;
  body_x_c = body_x_a;
  left_foot_x_c = left_foot_x_a;
  right_foot_x_c = right_foot_x_a;
  upper_joints_x_c = upper_joints_x_a;

  // kinematics
  first_flag = true;

  // init position
  // rTorso_init.head(3) = robotdata->q_a.segment(3, 3);
  // rTorso_init.tail(3) = rTorso_init.head(3);
  // rTorso_init << 0.02, 0.0, 0.0, 0.02, 0.0, 0.0;
  rTorso_init << body_x_a(0, 0), body_x_a(0, 1), 0.0, 0.02, 0.0, 0.0;
  rTorso_cmd = rTorso_init;
  xStand_init = robotdata->xStand_init;
  xStand_cmd = xStand_init;
  xCoM_init = com_x_a.row(0).transpose();
  leftFoot_init = left_foot_x_a.row(0).transpose();
  CoM_deltaY = 0.5 * leftFoot_init(4) - xCoM_init(4);
  bodyDeltaY = 0.5 * leftFoot_init(4) - body_x_a(0, 4);
  xStand_tgt = xStand_init;
  xStand_tgt(1) -= bodyDeltaY;
  xStand_tgt(2) -= leftFoot_init(5);
  xStand_tgt(4) -= bodyDeltaY;
  // rFootCmd(2) = robotdata->q_c(8);
  // rFootCmd(5) = robotdata->q_c(14);
  wkSpace2Joint(xStand_init, rTorso_init, rFootCmd, qCmd, qDotCmd, first_flag);
  armPolyJoint(Eigen::Vector2d(0.42, 0.42), qArmCmd);
  //    wkSpace2Joint(xStand,qCmd,qDotCmd,first_flag);

  // init
  //  WBC weights and constraints
  // robotdata->task_card_set[robotdata->body_task_id]->weight
  // << 50., 50., 50., 1., 1., 1.;
  // robotdata->task_card_set[robotdata->com_task_id]->weight << 10., 10., 10.,
  // 100., 100., 100.;
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      100.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->com_task_id]->weight.setZero();
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      50.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      1000.0 * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->upper_joints_id]->weight =
      100.0 * Eigen::VectorXd::Ones(11);

  // q_factor
  q_factor_init = robotdata->q_factor;
  q_dot_factor_init = robotdata->q_dot_factor;

  gait->setevent("");
}

void Single2Dual::run() {
  // std::cout << "Single2Dual::run()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  // auto plandata = gait->plandata_;
  // run
  // update task actual state
  //
  gait->robot_controller_._estimation_operator->task_state_update_x_a(
      robotdata);
  // start: plan
  // set wbc constraints
  // tau_lb , tau_ub, GRF_ub, Fref: 12*1
  Eigen::VectorXd tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  Eigen::VectorXd GRF_lb = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd GRF_ub = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd Fref = Eigen::VectorXd::Zero(12);

  totaltime = 4.0;
  double omega = M_PI / totaltime * 2.0;
  if (timer <= totaltime / 2.0) {
    // kinematics
    xStand_cmd(2) =
        xStand_init(2) - leftFoot_init(5) * (1 - cos(omega * timer)) / 2.0;
    rTorso_cmd(0) = rTorso_init(0) * (1 + cos(omega * timer)) / 2.0;
    rTorso_cmd(3) = rTorso_init(3) * (1 + cos(omega * timer)) / 2.0;
    wkSpace2Joint(xStand_cmd, rTorso_cmd, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_cmd.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_cmd.tail(3).transpose();
    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d.row(1).setZero();
    com_x_d.row(2).setZero();
    left_foot_x_d.row(0) = leftFoot_init.transpose();
    left_foot_x_d(0, 5) =
        leftFoot_init(5) - leftFoot_init(5) * (1 - cos(omega * timer)) / 2.0;
    left_foot_x_d(1, 5) =
        -leftFoot_init(5) * omega * sin(omega * (timer)) / 2.0;
    left_foot_x_d(2, 5) =
        -leftFoot_init(5) * omega * omega * cos(omega * (timer)) / 2.0;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 0.0;
    GRF_lb(11) = 20.0;
    GRF_lb.head(6).setZero();
    GRF_ub.head(6).setZero();

    robotdata->task_card_set[robotdata->left_foot_id]->weight =
        (50.0 + 950.0 * timer / totaltime * 2.0) * Eigen::VectorXd::Ones(6);

    Eigen::VectorXd q_factor_dual_stand = q_factor_init;
    Eigen::VectorXd q_dot_factor_dual_stand = q_dot_factor_init;
    q_factor_dual_stand.segment(0, 6) = 0.1 * Eigen::VectorXd::Ones(6);
    q_dot_factor_dual_stand.segment(0, 6) = 0.1 * Eigen::VectorXd::Ones(6);
    q_factor_dual_stand.segment(6, 6) = 0.1 * Eigen::VectorXd::Ones(6);
    q_dot_factor_dual_stand.segment(6, 6) = 0.1 * Eigen::VectorXd::Ones(6);

    robotdata->q_factor =
        q_factor_dual_stand * timer / (totaltime / 2.0) +
        q_factor_init * (totaltime / 2.0 - timer) / (totaltime / 2.0);
    robotdata->q_dot_factor =
        q_dot_factor_dual_stand * timer / (totaltime / 2.0) +
        q_dot_factor_init * (totaltime / 2.0 - timer) / (totaltime / 2.0);
  } else if (timer < totaltime) {
    double timer2 = timer - totaltime / 2.0;
    // kinematics
    xStand_cmd(1) =
        xStand_init(1) - bodyDeltaY * (1 - cos(omega * timer2)) / 2.0;
    xStand_cmd(4) =
        xStand_init(4) - bodyDeltaY * (1 - cos(omega * timer2)) / 2.0;
    wkSpace2Joint(xStand_cmd, rTorso_tgt, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1) = qDotCmd;
    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_tgt.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_cmd.tail(3).transpose();
    body_x_d(1, 4) = bodyDeltaY * omega * sin(omega * (timer2)) / 2.0;
    body_x_d(2, 4) = bodyDeltaY * omega * omega * cos(omega * (timer2)) / 2.0;
    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d(0, 4) = xCoM_init(4) + CoM_deltaY * (1 - cos(omega * timer2)) / 2.0;
    com_x_d(1, 4) = CoM_deltaY * omega * sin(omega * (timer2)) / 2.0;
    com_x_d(2, 4) = CoM_deltaY * omega * omega * cos(omega * (timer2)) / 2.0;
    // left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(0) = leftFoot_init.transpose();
    left_foot_x_d(0, 5) = 0.0;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    robotdata->task_card_set[robotdata->left_foot_id]->weight =
        1000.0 * Eigen::VectorXd::Ones(6);
    double GRF_ratio = 0.1;
    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_ub.head(6) =
        (1 - GRF_ratio) * (timer2 / totaltime * 2.0) * GRF_ub.head(6) +
        GRF_ratio * GRF_ub.head(6);
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 20.0 * timer2 / totaltime * 2.0;
    GRF_lb(11) = 20.0;
  } else {
    // kinematics
    wkSpace2Joint(xStand_tgt, rTorso_tgt, rFootCmd, qCmd, qDotCmd, first_flag);
    robotdata->q_c.block(6, 0, 12, 1) = qCmd;
    robotdata->q_dot_c.block(6, 0, 12, 1).setZero();
    // WBC
    body_x_d.setZero();
    body_x_d.block(0, 0, 1, 3) = rTorso_tgt.head(3).transpose();
    body_x_d.block(0, 3, 1, 3) = -xStand_tgt.tail(3).transpose();
    com_x_d.row(0) = xCoM_init.transpose();
    com_x_d(0, 4) = xCoM_init(4) + CoM_deltaY;
    com_x_d.row(1).setZero();
    com_x_d.row(2).setZero();
    left_foot_x_d = robotdata->task_card_set[robotdata->left_foot_id]->X_a;
    left_foot_x_d.row(2).setZero();
    right_foot_x_d = robotdata->task_card_set[robotdata->right_foot_id]->X_a;
    right_foot_x_d.row(2).setZero();

    tau_ub << 200.0, 130.0, 100.0, 200.0, 60.0, 60.0, 200.0, 130.0, 100.0,
        200.0, 60.0, 60.0, 82.5, 82.5, 82.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        18.0, 18.0;
    tau_lb = -tau_ub;
    GRF_ub << 16.6, 32.0, 10.0, 200.0, 200.0, 700.0, 16.6, 32.0, 10.0, 200.0,
        200.0, 700.0;
    GRF_lb = -GRF_ub;
    GRF_lb(5) = 20.0;
    GRF_lb(11) = 20.0;
  }
  // set waist and arm qc
  robotdata->q_c.segment(18, 3).setZero();
  robotdata->q_c.tail(8) = qArmCmd;
  robotdata->q_dot_c.tail(11).setZero();
  upper_joints_x_d.setZero();
  upper_joints_x_d.row(0) = robotdata->q_c.tail(11);

  // foot motion plan

  // end: plan
  // control
  com_x_c = com_x_d;
  body_x_c = body_x_d;
  left_foot_x_c = left_foot_x_d;
  right_foot_x_c = right_foot_x_d;
  upper_joints_x_c = upper_joints_x_d;

  // gait data update

  // plan_data
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->X_d<<std::endl;
  // std::cout<<"dim:
  // "<<robotdata->task_card_set[robotdata->body_task_id]->dim<<std::endl;
  robotdata->task_card_set[robotdata->com_task_id]->X_d.block(0, 0, 3, 6) =
      com_x_c;
  robotdata->task_card_set[robotdata->body_task_id]->X_d.block(0, 0, 3, 6) =
      body_x_c;
  robotdata->task_card_set[robotdata->left_foot_id]->X_d.block(0, 0, 3, 6) =
      left_foot_x_c;
  robotdata->task_card_set[robotdata->right_foot_id]->X_d.block(0, 0, 3, 6) =
      right_foot_x_c;
  robotdata->task_card_set[robotdata->upper_joints_id]->X_d.block(0, 0, 3, 11) =
      upper_joints_x_c;

  // update task desired state
  gait->robot_controller_._estimation_operator->task_state_update_x_d(
      robotdata);
  //    std::cout<<"hehe3"<<std::endl;

  Fref = robotdata->contactforce;
  gait->robot_controller_._wbc_solver->SetConstraints(tau_lb, tau_ub, GRF_lb,
                                                      GRF_ub, Fref);
  gait->robot_controller_._wbc_solver->SolveWbc(robotdata);

  // update time
  timer += robotdata->dt;

  // log data
  robotdata->dataL(19) = 9;
  robotdata->dataL(20) = robotdata->time;
  robotdata->dataL.segment(21, 29) = robotdata->q_a;
  robotdata->dataL.segment(50, 29) = robotdata->q_dot_a;
  robotdata->dataL.segment(79, 29) = robotdata->tau_a;
  robotdata->dataL.segment(108, 12) = robotdata->grf;
  robotdata->dataL.segment(120, 29) = robotdata->q_c;
  robotdata->dataL.segment(149, 29) = robotdata->q_dot_c;
  robotdata->dataL.segment(178, 29) = robotdata->tau_c;
  robotdata->dataL.segment(207, 12) = robotdata->contactforce;
  robotdata->dataL.segment(219, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(0).transpose();
  robotdata->dataL.segment(225, 6) =
      robotdata->task_card_set[robotdata->left_foot_id]->X_a.row(1).transpose();
  robotdata->dataL.segment(231, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(0)
          .transpose();
  robotdata->dataL.segment(237, 6) =
      robotdata->task_card_set[robotdata->right_foot_id]
          ->X_a.row(1)
          .transpose();
  // robotdata->tau_c = ratio*robotdata->tau_c;
  // updata event
  if (timer < totaltime) {
    gait->setevent("");
  } else {
    gait->setevent("gotoStand");
  }
}

void Single2Dual::onExit() {
  // std::cout << "Single2Dual::onExit()" << std::endl;
  GaitGenerator *gait = static_cast<GaitGenerator *>(app);
  auto robotdata = gait->robot_controller_._robot_data;
  robotdata->xStand_init = xStand_tgt;
  robotdata->rCmd_joystick_last.setZero();
  robotdata->rCmd_joystick.setZero();
  robotdata->inSingleStand = false;
  // robotdata->stance_index = 0;
}
