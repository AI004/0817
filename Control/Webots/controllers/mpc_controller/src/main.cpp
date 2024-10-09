#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>
#define JOYSTICK
// #define DATALOG_MAIN

// #define WEBOTS

#include "../../RobotControl/include/KalmanFilter.h"
#include "../../RobotControl/include/LowPassFilter.h"
#include "broccoli/core/Time.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

// include open source
#include "../../MotionPlan/include/planTools.h"
#include "../../StateMachine/include/StateGenerator.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>
#include <qpOASES.hpp>
#include <rbdl/rbdl.h>
#include <sys/time.h>

#ifdef JOYSTICK
#include "../../StateMachine/include/joystick.h"
#endif

#include "../include/webotsInterface.h"

//
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

#include "version.h"
#include "public_parament.h"
using namespace broccoli::core;

// imu data
// Eigen::VectorXd vnIMU::imuData=Eigen::VectorXd::Zero(9);
int main(int argc, char **argv) {
  // double dt = timeStep;
  double dt = 0.0025;
  // define data
  DataPackage *data = new DataPackage();
  // construct robot controller
  GaitGenerator gait;
  QString path = "./Sources/config/pnc_config_sim.json";
  // std::cout<<"controller init is start!"<<std::endl;
  // controller.init(path,dt);
  gait.init(path, dt, data);
  // std::cout<<"data init!"<<std::endl;
  data->addlog("data init complete, program is starting!");

#ifdef JOYSTICK
  Joystick_humanoid joy;
  joy.init();
#endif

  int motorNum = actor_num;
  // Initialize Webots
#ifdef WEBOTS
  WebotsRobot humanoid;
  humanoid.initWebots();
  webotState robotStateSim;
  Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd jointP = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd jointD = Eigen::VectorXd::Zero(motorNum);
  double simTime = 0;
  double myInf = 100000;
#endif
  // Initialize Webots

  //
  Time start_time;
  Time period(0, 2500000);
  Time sleep2Time;
  Time timer;
  timespec sleep2Time_spec;
  // double dt = 0.0025;

  Eigen::VectorXd absolute_pos = Eigen::VectorXd::Zero(motorNum);
  //
  //
  Eigen::VectorXd qEstDeg = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotEstDeg = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qEst = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotEst = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotEst_lowpass = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotEst_kalman = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd currentEst = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qTorEst = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDDotRef = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd currentCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qTorCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd frictioncurrentCmd = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd frictioncurrentCmd2 = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd frictionTor = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qCmd2 = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotCmd2 = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qTorCmd2 = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qEst_p = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd qDotEst_p = Eigen::VectorXd::Zero(motorNum);
  Eigen::VectorXd q_factor = Eigen::VectorXd::Ones(motorNum);
  Eigen::VectorXd qdot_factor = Eigen::VectorXd::Ones(motorNum);
  Eigen::VectorXd qDotfriciton = Eigen::VectorXd::Zero(motorNum);
  // init pos
  bool flag_start = true;
  Eigen::VectorXd qEstInit = Eigen::VectorXd::Zero(motorNum);

  //
  int simCnt = 0;
  double timeSim = 0.0;
  double timeStep = dt;
  double timeTotal = 500.0;
//
#ifdef DATALOG_MAIN
  std::ostringstream oss;
  spdlog::init_thread_pool(8190, 1);
  time_t currentTime = time(nullptr);
  char chCurrentTime[256];
  strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S",
           localtime(&currentTime));
  std::string stCurrentTime = chCurrentTime;
  std::string filename = stCurrentTime + "log.txt";
  auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      filename, 1024 * 1024 * 100, 3);
  rotating_sink->set_pattern("%v");
  std::vector<spdlog::sink_ptr> sinks{rotating_sink};
  auto logger = std::make_shared<spdlog::async_logger>(
      "loggername", sinks.begin(), sinks.end(), spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);
#endif
  // -----------------------------------------------robot controller software
  // -----------------------------------------------------------------

  Time time1;
  Time time2;
  Time time3;
  Time time4;
  Time time5;
  Time time6;

  // print version
  std::cout << "version: " << PROJECT_VERSION_MAJOR << "."
            << PROJECT_VERSION_MINOR << "." << PROJECT_VERSION_PATCH
            << std::endl;

  while (humanoid.robot->step(TIME_STEP) != -1) {
    start_time = timer.currentTime();
    //--------------------state estimation--------------------
    simTime = humanoid.robot->getTime();
    humanoid.readData(simTime, robotStateSim);
    qEst = robotStateSim.jointPosAct;
    qDotEst = robotStateSim.jointVelAct;
    qTorEst = jointTorCmd;

    time1 = timer.currentTime() - start_time;

    //---------------------high-level control-----------------------
    //

    time2 = timer.currentTime() - start_time - time1;

    // set actual task variable
    data->dim = generalized_coordinates;
    data->dt = dt;
    // // std::cout<<"data->dt: "<<data->dt<<std::endl;
    // // right leg
    data->q_a.block(6, 0, motorNum, 1) = qEst;
    data->q_dot_a.block(6, 0, motorNum, 1) = qDotEst;
    data->tau_a.block(6, 0, motorNum, 1) = qTorEst;

    data->imu_sensor.block(0, 0, 18, 1).setZero();
    data->imu_sensor.block(0, 0, 9, 1) = robotStateSim.imu9DAct;
    // gait.setevent("gotoZero");
    // // gait.setevent("gotoSingleLegTest");
    // // gait.setevent("gotoMotorMotion");
    // // //
#ifdef JOYSTICK
    if (gait.fsmstatename == "S2W" || gait.fsmstatename == "Z2S" ||
        gait.fsmstatename == "Dual2Single" ||
        gait.fsmstatename == "Single2Dual") {
      if (joy.get_state_change() == "gotoStop") {
        gait.setevent(joy.get_state_change());
        gait.set_current_fsm_command(joy.get_current_state_command());
      }
    } else {
      gait.setevent(joy.get_state_change());
      gait.set_current_fsm_command(joy.get_current_state_command());
    }
    if (gait.fsmstatename == "Walk" || gait.fsmstatename == "UniGait" ||
        gait.fsmstatename == "Swing") {
      gait.setvelocity(joy.get_walk_x_direction_speed(),
                       joy.get_walk_y_direction_speed(),
                       joy.get_walk_yaw_direction_speed());
      gait.setvelocity_offset(joy.get_walk_x_direction_speed_offset(),
                              joy.get_walk_y_direction_speed_offset());
      // gait.setFootRotateState(joy.get_foot_rotate());
      gait.setGaitMode(joy.get_gait_mode());
      gait.step_calibration(joy.get_calibration_flag());
    } else if (gait.fsmstatename == "Stand" ||
               gait.fsmstatename == "SingleStand") {
      gait.setxyz(joy.get_stand_x_direction_position(),
                  joy.get_stand_y_direction_posiiton(),
                  joy.get_stand_z_direction_posiiton());
      gait.setrollpitch(joy.get_stand_roll_direction_position(),
                        joy.get_stand_pitch_direction_posiiton(),
                        joy.get_stand_yaw_direction_posiiton());
      gait.setCarryBoxState(joy.get_carry_box_state(),
                            joy.get_if_stand_carry());
      if (gait.fsmstatename == "Stand")
        gait.setMotionState(joy.get_motion_state());
    } else {
    }

#endif
    // std::cout<<"event: "<<gait.event<<std::endl;
    gait.gait_run(data);

    // //
    qCmd = data->q_c.block(6, 0, motorNum, 1);
    qDotCmd = data->q_dot_c.block(6, 0, motorNum, 1);
    qTorCmd = data->tau_c.block(6, 0, motorNum, 1);
    q_factor = data->q_factor;
    qdot_factor = data->q_dot_factor;
    // std::cout << "q_factor.size(): " << q_factor.size() << std::endl;
    //----------------------------------//
    time4 = timer.currentTime() - start_time - time3 - time2 - time1;

    // --------------------friction compensation-------------------
    if(adam_type==ADAM_TYPE::AdamLite){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                60.0, 60.0, 60.0, 
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                1.0, 1.0, 1.0, 
                0.4, 0.4, 0.4, 0.4,
                0.4, 0.4, 0.4, 0.4;
    }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25;
    }else if(adam_type==ADAM_TYPE::AdamStandard){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                60.0, 60.0, 60.0,
                12.0, 12.0, 12.0, 12.0, 1.0, 1.0, 1.0, 1.0,
                12.0, 12.0, 12.0, 12.0, 1.0, 1.0, 1.0, 1.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                1.0, 1.0, 1.0, 
                0.4, 0.4, 0.4, 0.4, 0.1, 0.1, 0.1, 0.1,
                0.4, 0.4, 0.4, 0.4, 0.1, 0.1, 0.1, 0.1;
    }else if(adam_type==ADAM_TYPE::StandardPlus23){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                60.0, 60.0, 60.0, 
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                1.0, 1.0, 1.0, 
                0.4, 0.4, 0.4, 0.4,
                0.4, 0.4, 0.4, 0.4;
    }else if(adam_type==ADAM_TYPE::StandardPlus29){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                60.0, 60.0, 60.0, 
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                1.0, 1.0, 1.0, 
                0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
    }else if(adam_type==ADAM_TYPE::StandardPlus53){
      jointP << 450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                450.0, 120.0, 60.0, 400.0, 50.0, 2.0,
                60.0, 60.0, 60.0, 
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      jointD << 10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                10.0, 6.0, 1.0, 10.0, 2.0, 0.25,
                1.0, 1.0, 1.0, 
                0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
                0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
    }
    
    for (int i = 0; i < motorNum; i++) {
      // KP * q_factor * (qCmd - qEst) + KD * q_dot_factor * (qDotCmd - qDotEst) + qTorCmd
      jointTorCmd(i) = jointP(i) * q_factor(i) * (qCmd(i) - qEst(i)) +
                       jointD(i) * qdot_factor(i) * (qDotCmd(i) - qDotEst(i)) + qTorCmd(i);
      standPosCmd(i) = (1.0 - q_factor(i)) * qEst(i) + q_factor(i) * qCmd(i);
      // if(i==19){
      //   std::cout << jointTorCmd(i) << "\t" 
      //   << jointP(i) << "\t" 
      //   << q_factor(i) << "\t" 
      //   << qCmd(i) << "\t" 
      //   << qEst(i) << "\t" 
      //   << jointD(i) << "\t" 
      //   << qdot_factor(i) << "\t" 
      //   << qDotCmd(i) << "\t" 
      //   << qDotEst(i) << "\t"
      //   << qTorCmd(i) << "\t" << std::endl;
      // }
    }
    // for(int i=22;i<=33;i++){
    //   jointTorCmd(i) = 0;
    //   standPosCmd(i) = 0;
    // }
    // for(int i=41;i<=52;i++){
    //   jointTorCmd(i) = 0;
    //   standPosCmd(i) = 0;
    // }
    // jointTorCmd(19) = 0;jointTorCmd(20) = 0;jointTorCmd(21) = 0;standPosCmd(19) = 0;standPosCmd(20) = 0;standPosCmd(21) = 0;
    // jointTorCmd(26) = 0;jointTorCmd(27) = 0;jointTorCmd(28) = 0;standPosCmd(26) = 0;standPosCmd(27) = 0;standPosCmd(28) = 0;

    if (timeSim < 1.0 || gait.fsmstatename == "Start" ||
        gait.fsmstatename == "Zero" || gait.fsmstatename == "Swing") {
      humanoid.setMotorPos(standPosCmd);
    } else {
      // std::cout << "jointTorCmd: " << jointTorCmd.tail(8).transpose() << std::endl;
      // std::cout << "jointTorCmd: " << jointTorCmd.transpose() << std::endl;
      humanoid.setMotorTau(jointTorCmd);
      // int i;
      // for(i=0;i<humanoid.nJoint;i++) {
      //   if(i==19||i==20||i==21||i==26||i==27||i==28) {
      //     humanoid.legMotor[i]->setPosition(standPosCmd(i, 0));
      //   } else {
      //     humanoid.legMotor[i]->setTorque(jointTorCmd(i, 0));
      //   }
      // }
    }
    // std::cout<<"hehe: 4"<< std::endl;
    simCnt += 1;
    timeSim = simCnt * timeStep;
    time5 = timer.currentTime() - start_time - time4 - time3 - time2 - time1;
//
#ifdef DATALOG_MAIN
    data->dataL(0) = timeSim;
    data->dataL(1) = start_time.m_seconds;
    data->dataL(2) = start_time.m_nanoSeconds;
    data->dataL(3) = time1.m_nanoSeconds;
    data->dataL(4) = time2.m_nanoSeconds;
    data->dataL(5) = time3.m_nanoSeconds;
    data->dataL(6) = time4.m_nanoSeconds;
    data->dataL(7) = time5.m_nanoSeconds;
    data->dataL(8) = time6.m_nanoSeconds;
    data->dataL(9) = (timer.currentTime() - start_time).m_nanoSeconds;
    data->dataL.segment(10, 9) = data->imu_sensor.block(0, 0, 9, 1);
    for (const auto &i : data->dataL) {
      oss << i << " ";
    }
    logger->info(oss.str());
    oss.str("");
#endif
    //
    time6 = timer.currentTime() - start_time - time5 - time4 - time3 - time2 -
            time1;

    if (joy.disableJoints()) {
      break;
    }
  }
  humanoid.deleteRobot();
#ifdef DATALOG_MAIN
  logger->flush();
#endif
  return 0;
}
