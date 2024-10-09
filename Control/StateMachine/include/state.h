#ifndef STATE_H
#define STATE_H

#include "../../MotionPlan/include/gaitPlan.h"
#include "../../MotionPlan/include/planTools.h"
#include "../../MotionPlan/include/uniPlan.h"
#include "../../RobotControl/include/LowPassFilter.h"
#include "XFsmState.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <QRegularExpression>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QRegularExpression>
class Start : public XFsmState {
public:
  Start(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  double timer = 0.0;
};

class Zero : public XFsmState {
public:
  Zero(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // current joint states
  Eigen::MatrixXd qa;
  Eigen::MatrixXd qa_dot;
  // target joint states
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qd_dot;
  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  bool footflag;
  int step_count = 0;
};

class Swing : public XFsmState {
public:
  Swing(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  //
  Eigen::VectorXd q_init;
  //
  Eigen::VectorXd q_c;
  Eigen::VectorXd q_dot_c;
  Eigen::VectorXd q_ddot_c;
  Eigen::VectorXd tau_c;
  //
  // bool dataLog(Eigen::VectorXd &v, std::ofstream &f);

  // intialize bezier parameter
  int M;
  int Nout;
  int stIndex;
  double tStepPre;
  Eigen::VectorXd init_pos;

  // WorkSpace Trajectory
  bool firstFlag;
  int Nwout;
  Eigen::VectorXd xInit, xDotInit, xDDotInit;
  Eigen::VectorXd xEnd, xDotEnd;
  Eigen::VectorXd xCmd, xDotCmd, xDDotCmd, fCmd;
  Eigen::VectorXd qCmd, qDotCmd;
  Eigen::VectorXd qArmCmd;
  double vCmd;
  Eigen::VectorXd xStand;
  // wbc
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
};

class Z2S : public XFsmState {
public:
  Z2S(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  // Eigen::MatrixXd com_x_a;
  // Eigen::MatrixXd com_x_d;
  // Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;

  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  Eigen::VectorXd xStand_tgt;
  Eigen::VectorXd qArmCmd;
  bool first_flag;
  bool cali_imu_done{false};
};

class Stand : public XFsmState {
public:
  Stand(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  Eigen::MatrixXd com_x_a;
  Eigen::MatrixXd com_x_d;
  Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;

  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  Eigen::VectorXd xStand_tgt;
  Eigen::VectorXd xStand_Zero;

  Eigen::VectorXd com_tgt;
  Eigen::VectorXd q_factor_init;
  Eigen::VectorXd q_dot_factor_init;
  bool first_flag;
  Eigen::Vector3d torso_d;
  Eigen::VectorXd rTorsoInit;
  Eigen::VectorXd rTorsoCmd;
  Eigen::VectorXd rTorso_tgt;
  Eigen::VectorXd rFootCmd;

  Eigen::VectorXd upperJointsRefPos;
  Eigen::VectorXd upperJointsRefPosFilter;
  Eigen::VectorXd upperJointsRefPosLast;

  bool motion_start_flag;
  double time1, time2, time3, time4;
  double box_start_timer;
  int box_last_state;

  int motion_gentle_flag;
};

class S2W : public XFsmState {
public:
  S2W(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  // Eigen::MatrixXd com_x_a;
  // Eigen::MatrixXd com_x_d;
  // Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  Eigen::VectorXd xStand_tgt;
  bool first_flag;
  Eigen::VectorXd rTorsoCmd;
  Eigen::VectorXd rFootCmd;

  Eigen::VectorXd q_factor_init;
  Eigen::VectorXd q_dot_factor_init;
};

class Walk : public XFsmState {
public:
  Walk(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  //
  Eigen::VectorXd q_init;
  //
  Eigen::VectorXd q_c;
  Eigen::VectorXd q_dot_c;
  Eigen::VectorXd q_ddot_c;
  Eigen::VectorXd tau_c;

  // intialize bezier parameter
  int M;
  int Nout;
  int stIndex;
  double tStepPre;
  Eigen::VectorXd init_pos;

  // WorkSpace Trajectory
  bool firstFlag;
  int Nwout;
  Eigen::VectorXd xInit, xDotInit, xDDotInit;
  Eigen::VectorXd xEnd, xDotEnd;
  Eigen::VectorXd xCmd, xDotCmd, xDDotCmd, fCmd;
  Eigen::VectorXd qCmd, qDotCmd;
  double vCmd;
  Eigen::VectorXd xStand;
  gaitPlan *gait_plan;
  // wbc
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // prepare to stand
  // bool prestand = false;
  int prestand_step = 0;
  Eigen::VectorXd rTorsoCmd;
  Eigen::VectorXd rFootCmd;
  Eigen::VectorXd qArmCmd;
};

class UniGait : public XFsmState {
public:
  UniGait(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  //
  Eigen::VectorXd q_init;
  //
  Eigen::VectorXd q_c;
  Eigen::VectorXd q_dot_c;
  Eigen::VectorXd q_ddot_c;
  Eigen::VectorXd tau_c;

  // intialize bezier parameter
  int M;
  int Nout;
  int stIndex;
  double tStepPre;
  Eigen::VectorXd init_pos;

  // WorkSpace Trajectory
  bool firstFlag;
  int Nwout;
  Eigen::VectorXd xInit, xDotInit, xDDotInit;
  Eigen::VectorXd xEnd, xDotEnd;
  Eigen::VectorXd xCmd, xDotCmd, xDDotCmd, fCmd;
  Eigen::VectorXd qCmd, qDotCmd;
  double vCmd;
  Eigen::VectorXd xStand;
  Eigen::VectorXd q_ini;
  uniPlan *gait_plan;
  // wbc
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // prepare to stand
  // bool prestand = false;
  int prestand_step = 0;
  Eigen::VectorXd rTorsoCmd;
  Eigen::VectorXd rFootCmd;
  Eigen::VectorXd qArmCmd;

  int stance_flag = 0;
};

class Stop : public XFsmState {
public:
  Stop(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // current joint states
  Eigen::MatrixXd qa;
  Eigen::MatrixXd qa_dot;
  // target joint states
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qd_dot;
  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  bool footflag;
};

class Dual2Single : public XFsmState {
public:
  Dual2Single(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  Eigen::MatrixXd com_x_a;
  Eigen::MatrixXd com_x_d;
  Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand_init, xStand_tgt1, xStand_tgt2, xStand_cmd;
  Eigen::VectorXd rTorso_init, rTorso_tgt, rTorso_cmd;
  Eigen::VectorXd xCoM_init;
  Eigen::VectorXd rFootCmd;
  Eigen::VectorXd leftFoot_init;
  Eigen::VectorXd qCmd, qDotCmd;
  Eigen::VectorXd qArmCmd;
  double leftFootYawInit;
  bool first_flag;

  Eigen::VectorXd CoM_delta;
  Eigen::VectorXd leftFootAirDelta;

  Eigen::VectorXd q_factor_init;
  Eigen::VectorXd q_dot_factor_init;
};

class SingleStand : public XFsmState {
public:
  SingleStand(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  Eigen::MatrixXd com_x_a;
  Eigen::MatrixXd com_x_d;
  Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // total time
  double totaltime;

  Eigen::VectorXd xStand, qCmd, qDotCmd;
  Eigen::VectorXd xStand_tgt;
  Eigen::VectorXd xStand_Zero;
  Eigen::VectorXd xCoM_init;
  Eigen::VectorXd q_factor_init;
  Eigen::VectorXd q_dot_factor_init;
  bool first_flag;
  Eigen::Vector3d torso_d;
  Eigen::VectorXd rTorso_init;
  Eigen::VectorXd rTorso_cmd;
  Eigen::VectorXd rFootCmd;
  Eigen::VectorXd leftFoot_tgt;
  Eigen::VectorXd qArmCmd;
};

class Single2Dual : public XFsmState {
public:
  Single2Dual(void *App) : XFsmState(App){};
  void onEnter();
  void run();
  void onExit();
  void init();
  // clock
  double timer = 0.0;
  // com task:
  Eigen::MatrixXd com_x_a;
  Eigen::MatrixXd com_x_d;
  Eigen::MatrixXd com_x_c;
  // body orientation task:
  Eigen::MatrixXd body_x_a;
  Eigen::MatrixXd body_x_d;
  Eigen::MatrixXd body_x_c;
  // left foot task:
  Eigen::MatrixXd left_foot_x_a;
  Eigen::MatrixXd left_foot_x_d;
  Eigen::MatrixXd left_foot_x_c;
  // right foot task:
  Eigen::MatrixXd right_foot_x_a;
  Eigen::MatrixXd right_foot_x_d;
  Eigen::MatrixXd right_foot_x_c;
  // upper body joints task:
  Eigen::MatrixXd upper_joints_x_a;
  Eigen::MatrixXd upper_joints_x_d;
  Eigen::MatrixXd upper_joints_x_c;
  // total time
  double totaltime;
  // average speed
  double avr_v;

  Eigen::VectorXd xStand_init, xStand_tgt, xStand_cmd;
  Eigen::VectorXd rTorso_init, rTorso_tgt, rTorso_cmd;
  Eigen::VectorXd xCoM_init;
  Eigen::VectorXd leftFoot_init;
  Eigen::VectorXd rFootCmd;
  Eigen::VectorXd qCmd, qDotCmd;
  Eigen::VectorXd qArmCmd;
  bool first_flag;

  double CoM_deltaY;
  double bodyDeltaY;

  Eigen::VectorXd q_factor_init;
  Eigen::VectorXd q_dot_factor_init;
};

#endif
