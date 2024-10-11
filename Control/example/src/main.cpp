#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#define JOYSTICK
#define DATALOG_MAIN

#include "../../RobotControl/include/KalmanFilter.h"
#include "../../RobotControl/include/LowPassFilter.h"
#include "broccoli/core/Time.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

// include open source
#include <math.h>
#include <rbdl/rbdl.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <qpOASES.hpp>
#include "../../MotionPlan/include/planTools.h"
#include "../../RobotInterface/include/RobotInterface.h"
#include "../../StateMachine/include/StateGenerator.h"
#include "vnIMU/vnIMU.h"

#ifdef JOYSTICK
#  include "../../StateMachine/include/joystick.h"
#endif

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

#include "NokovClinet.h"
#include "public_parament.h"
#include "udp_hand_speed.h"
#include "version.h"
using namespace broccoli::core;
#define NOKOV_ENABLE

Eigen::VectorXd vnIMU::imuData = Eigen::VectorXd::Zero(9);
bool vnIMU::use_IMU_correction = true;
int main() {
  // double dt = timeStep;
  double dt = 0.0025;
  // define data
  DataPackage* data = new DataPackage();
  // construct robot controller
  GaitGenerator gait;
  QString path = "./Sources/config/pnc_config.json";
  gait.init(path, dt, data);
  // std::cout<<"data init!"<<std::endl;
  data->addlog("data init complete, program is starting!");

  // initial hand position
  std::vector<int> hands_position(12, 0);
  hands_position[4] = 30;
  hands_position[5] = 1000;
  hands_position[10] = 30;
  hands_position[11] = 1000;
  HandController controller(hands_position);

  vnIMU imu;
  const Robot_Data& robot_data111 = gait.getRobotData();
  imu.initIMU();
  imu.use_IMU_correction = robot_data111.use_IMU_correction;

#ifdef NOKOV_ENABLE
  // nokov data process
  NokovClient();
#endif

#ifdef JOYSTICK
  Joystick_humanoid joy;
  joy.init();
#endif

  // set cpu-affinity
  int cpus = 0;
  cpu_set_t mask;

  cpus = sysconf(_SC_NPROCESSORS_CONF);
  // printf("cpus: %d\n", cpus);

  CPU_ZERO(&mask);           // init mask
  CPU_SET(cpus - 1, &mask);  // add last cup core to cpu set

  if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
    printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
    return -1;
  }
  usleep(1000);
  // printf("set CPU affinity success\n");
  // set cpu-affinity

  // set sched-stratergy
  struct sched_param sched;

  int max_priority;

  max_priority = sched_get_priority_max(SCHED_RR);
  sched.sched_priority = max_priority;

  if (sched_setscheduler(getpid(), SCHED_RR, &sched) == -1) {
    printf("Set Scheduler Param, ERROR:%s\n", strerror(errno));
    return -1;
  }
  usleep(1000);
  // printf("set scheduler success\n");
  // set sched-stratergy

  // new sdk
  RobotData robot_data;
  RobotInterface* robot_interface = get_robot_interface();
  robot_interface->Init();

  //
  Time start_time;
  Time period(0, 2500000);
  Time sleep2Time;
  Time timer;
  timespec sleep2Time_spec;

  //
  int motorNum = actor_num;
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
//
#ifdef DATALOG_MAIN
  std::ostringstream oss;
  // spdlog::init_thread_pool(8190, 1);
  time_t currentTime = time(nullptr);
  char chCurrentTime[256];
  strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S", localtime(&currentTime));
  std::string stCurrentTime = chCurrentTime;
  std::string filename = stCurrentTime + "log.txt";
  auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, 1024 * 1024 * 100, 3);
  rotating_sink->set_pattern("%v");
  std::vector<spdlog::sink_ptr> sinks{rotating_sink};
  auto logger = std::make_shared<spdlog::async_logger>("loggername", sinks.begin(), sinks.end(), spdlog::thread_pool(),
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
  std::cout << "version: " << PROJECT_VERSION_MAJOR << "." << PROJECT_VERSION_MINOR << "." << PROJECT_VERSION_PATCH
            << std::endl;

  while (true) {
    start_time = timer.currentTime();
    //--------------------state estimation--------------------
    robot_interface->GetState(timeSim, vnIMU::imuData, robot_data);
    time1 = timer.currentTime() - start_time;
    qEst = robot_data.q_a_.tail(motorNum);
    qDotEst = robot_data.q_dot_a_.tail(motorNum);
    qTorEst = robot_data.tau_a_.tail(motorNum);
    //---------------------high-level control-----------------------
    //
    time2 = timer.currentTime() - start_time - time1;
    if (timeSim < 1.0) {
      for (int i = 0; i < motorNum; i++) {
        qCmd[i] = qEst(i);  // absolute_pos(i);
      }
      qDotCmd.setZero();
      q_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
      qdot_factor = 1.0 * Eigen::VectorXd::Ones(motorNum);
    } else {
      data->dim = 29;
      data->dt = dt;

      data->q_a.block(6, 0, motorNum, 1) = qEst;
      data->q_dot_a.block(6, 0, motorNum, 1) = qDotEst;
      data->tau_a.block(6, 0, motorNum, 1) = qTorEst;

      data->imu_sensor.block(0, 0, 18, 1).setZero();
      data->imu_sensor.block(0, 0, 9, 1) = robot_interface->imu_data_;

#ifdef JOYSTICK
      if (gait.fsmstatename == "S2W" || gait.fsmstatename == "Z2S" || gait.fsmstatename == "Dual2Single" ||
          gait.fsmstatename == "Single2Dual") {
        if (joy.get_state_change() == "gotoStop") {
          gait.setevent(joy.get_state_change());
          gait.set_current_fsm_command(joy.get_current_state_command());
        }
      } else if (gait.fsmstatename == "Zero" &&
                 gait.robot_controller_._robot_data->grf(5) + gait.robot_controller_._robot_data->grf(11) <
                     0.5 * gait.robot_controller_._robot_data->MG &&
                 joy.get_state_change() == "gotoZ2S") {
        std::cout << "Not fully standing! Lower the robot." << std::endl;
      } else {
        gait.setevent(joy.get_state_change());
        gait.set_current_fsm_command(joy.get_current_state_command());
      }
      if (gait.fsmstatename == "Walk" || gait.fsmstatename == "UniGait" || gait.fsmstatename == "Swing") {
        gait.setvelocity(joy.get_walk_x_direction_speed(), joy.get_walk_y_direction_speed(),
                         joy.get_walk_yaw_direction_speed());
        gait.setvelocity_offset(joy.get_walk_x_direction_speed_offset(), joy.get_walk_y_direction_speed_offset());
        gait.setGaitMode(joy.get_gait_mode());
        // gait.setFootRotateState(joy.get_foot_rotate());
        gait.step_calibration(joy.get_calibration_flag());
      } else if (gait.fsmstatename == "Stand" || gait.fsmstatename == "SingleStand") {
        gait.setxyz(joy.get_stand_x_direction_position(), joy.get_stand_y_direction_posiiton(),
                    joy.get_stand_z_direction_posiiton());
        gait.setMomtumController(joy.get_momentumController_on());
        gait.setrollpitch(joy.get_stand_roll_direction_position(), joy.get_stand_pitch_direction_posiiton(),
                          joy.get_stand_yaw_direction_posiiton());
        gait.setCarryBoxState(joy.get_carry_box_state(), joy.get_if_stand_carry());
        if (gait.fsmstatename == "Stand") gait.setMotionState(joy.get_motion_state());
      } else {
      }
#endif
      // leg and waist control
      gait.gait_run(data);
      //
      qCmd = data->q_c.block(6, 0, motorNum, 1);
      qDotCmd = data->q_dot_c.block(6, 0, motorNum, 1);
      qTorCmd = data->tau_c.block(6, 0, motorNum, 1);
      q_factor = data->q_factor;
      qdot_factor = data->q_dot_factor;
    }

    // --------------------friction compensation-------------------
    for (int i = 0; i < motorNum; i++) {
      qCmd2(i) = (1.0 - q_factor(i)) * qEst(i) + q_factor(i) * qCmd(i);
      qDotCmd2(i) = (1.0 - qdot_factor(i)) * qDotEst(i) + qdot_factor(i) * qDotCmd(i);
    }
    qTorCmd2 = qTorCmd;
    robot_data.q_d_.tail(motorNum) = qCmd2;
    robot_data.q_dot_d_.tail(motorNum) = qDotCmd2;
    robot_data.tau_d_.tail(motorNum) = qTorCmd2;

    robot_interface->SetCommand(robot_data);
    // motorlist.setcommand(qCmd2,qDotCmd2,qTorCmd2,0,0,qDotfriciton);

    if (adam_type == ADAM_TYPE::AdamLite) {
    } else if (adam_type == ADAM_TYPE::AdamStandard) {
    } else if (adam_type == ADAM_TYPE::StandardPlus23) {
    } else if (adam_type == ADAM_TYPE::StandardPlus29) {
      static int delay_ = 0;
      if (data->hands_motion_flag) {
        if (delay_ % 4 == 0) {
          for (int i = 0; i < data->q_d_hands.size(); i++) {
            hands_position[i] = static_cast<int>(data->q_d_hands(i));
          }
          controller.updateHandControlValues(hands_position);
          controller.getHandsState();
          for (int i = 0; i < 12; i++) {
            data->dataL(282 + i) = controller.val_act_buff[i];
          }
        }
        delay_++;
      }
    } else if (adam_type == ADAM_TYPE::StandardPlus53) {
    } else if (adam_type == ADAM_TYPE::AdamLiteSimple) {
    } else if (adam_type == ADAM_TYPE::DuckDuck) {
    }

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
    // data->dataL(299) = robot_data.communication_time;
    for (const auto& i : data->dataL) {
      oss << i << " ";
    }
    logger->info(oss.str());
    oss.str("");
#endif

    if (joy.disableJoints()) {
      break;
    }

    if (robot_interface->error_state_) {
      gait.setevent("gotoStop");
    }

    time6 = timer.currentTime() - start_time - time5 - time4 - time3 - time2 - time1;
    // std::cout<<"?"<<std::endl;
    sleep2Time = start_time + period;
    sleep2Time_spec = sleep2Time.toTimeSpec();
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(sleep2Time_spec), NULL);
  }

  robot_interface->DisableAllJoints();
  // motorlist.disable();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  delete robot_interface;
#ifdef DATALOG_MAIN
  logger->flush();
#endif
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  return 0;
}
