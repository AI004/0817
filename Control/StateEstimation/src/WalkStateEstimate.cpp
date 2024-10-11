#include "WalkStateEstimate.h"
#include "Robot_Data.h"
#define MOMENTUM_FILT

WalkStateEstimate::WalkStateEstimate() {
  lowpass = new LowPassFilter(50.0, 0.707, 0.0025, 3);
  lowpass2 = new LowPassFilter(50.0, 0.707, 0.0025, 3);
  accKalman = new AccKalmanFilter(0.001, 0.01, 0.0025, 3);
  // Eigen::VectorXd d_SysNoise = Eigen::VectorXd::Zero(4);
  // Eigen::VectorXd d_MeaNoise = Eigen::VectorXd::Zero(8);
  // d_SysNoise << 0.001*9.8/55.0, 0.001*9.8/55.0, 0.001*9.8/55.0, 0.001/55.0;
  // d_MeaNoise << 0.01, 0.01, 0.01, 0.001, 0.01, 0.01, 0.01, 0.001;
  // leggedKalman = new LeggedKalmanFilter(d_SysNoise, d_MeaNoise, 0.0025, 4, 8,
  // 3);
  Eigen::VectorXd d_SysNoise = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd d_MeaNoise = Eigen::VectorXd::Zero(12);
  d_SysNoise << 0.01, 0.01, 0.001, 0.001 * 9.8, 0.001 * 9.8, 0.001 * 9.8, 0.002, 0.002, 0.002, 0.002;
  d_SysNoise.segment(2, 4) = d_SysNoise.segment(2, 4) / 55.0;
  d_MeaNoise << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01;
  // d_MeaNoise.segment(3, 3) = d_MeaNoise.segment(3, 3) / 55.;
  // d_MeaNoise.segment(9, 3) = d_MeaNoise.segment(9, 3) / 55.;
  leggedKalman = new LeggedKalmanFilter(d_SysNoise, d_MeaNoise, 0.0025, 10, 12, 3);

  angularMomentum = Vector3d::Zero();

  windowSize = 4;
  threshold = 1.0;
}

// lhj notice: 不是腰在世界坐标系中的位置和速度，而是腰在支撑脚坐标系中的位置和速度
bool WalkStateEstimate::estWaistPosVelInWorld(Robot_Data* robotdata, int FootType) {
  Vector3d posTorso = Vector3d::Zero();
  Vector3d velTorso = Vector3d::Zero();
  Vector3d velTorso_filt = Vector3d::Zero();
  Vector3d velTorso_filt2 = Vector3d::Zero();
  switch (FootType) {
    case 0:  // Left Sole Stance
      posTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() -
                 robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
      velTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() -
                 robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose();
      break;
    case 1:  // Right Sole Stance
      posTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() -
                 robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
      velTorso = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() -
                 robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose();
      break;
    default:
      break;
  }

  int filterType = 0;
  if (filterType == 0) {
    velTorso_filt = lowpass->mFilter(velTorso);
  } else {
    Eigen::Vector3d aWorld = Vector3d::Zero();
    double trust;
    trust = std::min(robotdata->t / robotdata->Tc, 1.0);
    trust = std::min(trust, std::min(1.0 + (robotdata->T - robotdata->t) / robotdata->Tc, 1.0));
    trust = std::max(trust, 0.0);
    trust = 1.0;
    aWorld = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(robotdata->q_a(5)) * robotdata->imuAcc;
    aWorld(2) = aWorld(2) - 9.81;
    velTorso_filt = accKalman->mFilter(velTorso, aWorld, trust);
    robotdata->temp_worldacc = aWorld;
  }
  // Eigen::Vector3d aWorld = Vector3d::Zero();
  // double trust;
  // trust = 1.0;
  // aWorld =
  // rotX(robotdata->q_a(3))*rotY(robotdata->q_a(4))*rotZ(robotdata->q_a(5))*robotdata->imuAcc;
  // aWorld(2) = aWorld(2) - 9.81;
  // // velTorso_filt = accKalman->mFilter(velTorso, aWorld, trust);
  robotdata->temp_kalman.head(3) = velTorso;
  velTorso_filt2 = lowpass->mFilter(velTorso);
  robotdata->temp_kalman.tail(3) = velTorso_filt2;
  // robotdata->temp.tail(3) = accKalman->mFilter(velTorso, aWorld, trust);

  robotdata->q_a.head(3) = posTorso;
  robotdata->q_dot_a.head(3) = velTorso_filt;
  robotdata->observeData.head(3) = robotdata->q_dot_a.head(3);
  // robotdata->temp = velTorso;
  // robotdata->odometer = posTorso + robotdata->foot_odometer;
  return true;
}

bool WalkStateEstimate::grfEstimating(Robot_Data* robotdata) {
  robotdata->grf_pre = robotdata->grf;

  Eigen::MatrixXd J_grf = Eigen::MatrixXd::Zero(12, 12);
  Eigen::VectorXd nonlinear = Eigen::VectorXd::Zero(robotdata->ndof);
  J_grf.block(0, 0, 6, 12) = robotdata->task_card_set[robotdata->left_foot_id]->jacobi.block(0, 6, 6, 12);
  J_grf.block(6, 0, 6, 12) = robotdata->task_card_set[robotdata->right_foot_id]->jacobi.block(0, 6, 6, 12);
  RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), robotdata->q_a, robotdata->q_dot_a, nonlinear);
  robotdata->grf =
      J_grf.transpose().colPivHouseholderQr().solve(nonlinear.segment(6, 12) - robotdata->tau_a.segment(6, 12));

  // std::cout<<"grf"<<robotdata->grf.transpose()<<std::endl;
  return true;
}

bool WalkStateEstimate::stateMachine(Robot_Data* robotdata) {
  // grfEstimating(robotdata);

  robotdata->t = robotdata->time - robotdata->t_switch;
  robotdata->touch_index_pre = robotdata->touch_index;

  if ((robotdata->touch_index_pre == 4 &&
       (robotdata->grf(11) - robotdata->grf_pre(11) >= robotdata->grf_lb || robotdata->grf(11) > 0.3 * robotdata->MG) &&
       robotdata->t > robotdata->Td_ratio * robotdata->T + robotdata->Tc) ||
      (robotdata->touch_index_pre > 2 && robotdata->t > robotdata->T + robotdata->Tc - 0.5 * robotdata->dt)) {
    robotdata->T = robotdata->Td;
    robotdata->t_switch = robotdata->time;
    robotdata->stance_index = 1;
    robotdata->t = robotdata->time - robotdata->t_switch;
    robotdata->touch_index = 1;
    robotdata->step = robotdata->step + 1;
    robotdata->t_ftd = robotdata->T;
    robotdata->foot_odometer = robotdata->foot_odometer +
                               robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->flag_ankle = 0;
  }
  if (robotdata->touch_index_pre == 1 && robotdata->grf(11) >= robotdata->grf_ub) {
    robotdata->touch_index = 2;
    robotdata->t_ftd = robotdata->t;
  }
  if ((robotdata->touch_index_pre == 2 &&
       (robotdata->grf(5) - robotdata->grf_pre(5) >= robotdata->grf_lb || robotdata->grf(5) > 0.3 * robotdata->MG) &&
       robotdata->t > robotdata->Td_ratio * robotdata->T + robotdata->Tc) ||
      (robotdata->touch_index_pre < 3 && robotdata->t > robotdata->T + robotdata->Tc - 0.5 * robotdata->dt)) {
    robotdata->T = robotdata->Td;
    robotdata->t_switch = robotdata->time;
    robotdata->stance_index = 0;
    robotdata->t = robotdata->time - robotdata->t_switch;
    robotdata->touch_index = 3;
    robotdata->step = robotdata->step + 1;
    robotdata->t_ftd = robotdata->T;
    robotdata->foot_odometer =
        robotdata->foot_odometer + robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->flag_ankle = 0;
  }
  if (robotdata->touch_index_pre == 3 && robotdata->grf(5) >= robotdata->grf_ub) {
    robotdata->touch_index = 4;
    robotdata->t_ftd = robotdata->t;
  }

  if ((robotdata->touch_index == 4 && fabs(robotdata->grf(7) - robotdata->grf_pre(7)) >= 2.0) ||
      (robotdata->touch_index == 2 && fabs(robotdata->grf(1) - robotdata->grf_pre(1)) >= 2.0)) {
    if (robotdata->t > 0.7 * robotdata->T + robotdata->Tc) {
      robotdata->flag_ankle = 1;
    }
  }

  // if ((robotdata->touch_index_pre == 4 && robotdata->grf(11) -
  // robotdata->grf_pre(11) >= robotdata->grf_lb && robotdata->t >
  // 0.5*robotdata->T )){
  //     robotdata->t_switch = robotdata->time;
  //     robotdata->stance_index = 1;
  //     robotdata->t = robotdata->time - robotdata->t_switch;
  //     robotdata->touch_index = 1;
  //     robotdata->step = robotdata->step + 1;
  // }
  // if (robotdata->touch_index_pre == 1 && robotdata->grf(11) >=
  // robotdata->grf_ub){
  //     robotdata->touch_index = 2;
  // }
  // if ((robotdata->touch_index_pre == 2 && robotdata->grf(5) -
  // robotdata->grf_pre(5) >= robotdata->grf_lb && robotdata->t >
  // 0.5*robotdata->T )){
  //     robotdata->t_switch = robotdata->time;
  //     robotdata->stance_index = 0;
  //     robotdata->t = robotdata->time - robotdata->t_switch;
  //     robotdata->touch_index = 3;
  //     robotdata->step = robotdata->step + 1;
  // }
  // if (robotdata->touch_index_pre == 3 && robotdata->grf(5) >=
  // robotdata->grf_ub){
  //     robotdata->touch_index = 4;
  // }

  robotdata->s = (robotdata->t - robotdata->Tc) / robotdata->T;
  robotdata->s = std::max(std::min(robotdata->s, 1.0), 0.0);
  return true;
}

bool WalkStateEstimate::generalStateMachine(Robot_Data* robotdata) {
  for (int i = 0; i < 2; ++i) {
    robotdata->st_leg_pre(i) = robotdata->st_leg(i);
    robotdata->t_leg(i) = robotdata->time - robotdata->t0_leg(i);

    if (robotdata->st_leg_pre(i) == 1 && robotdata->t_leg(i) > robotdata->tst_leg(i) - 0.5 * robotdata->dt) {
      robotdata->st_leg(i) = 0;
    }
    if (robotdata->st_leg_pre(i) == 0 &&
        (robotdata->grf(5 + i * 6) > 0.2 * robotdata->MG ||
         robotdata->grf(5 + i * 6) - robotdata->grf_pre(5 + i * 6) >= 60.0) &&
        robotdata->t_leg(i) > robotdata->tst_leg(i) + robotdata->Td_ratio * robotdata->tsw_leg(i)) {
      robotdata->td_leg(i) = 1;
    }
    if (robotdata->st_leg_pre(i) == 0 &&
        robotdata->t_leg(i) > robotdata->tst_leg(i) + robotdata->tsw_leg(i) - 0.5 * robotdata->dt) {
      robotdata->late_leg(i) = 1;
    }
    if (robotdata->st_leg_pre(i) == 0 &&
        robotdata->t_leg(i) >
            robotdata->tst_leg(i) + robotdata->Td_ratio * robotdata->tsw_leg(i) - 0.5 * robotdata->dt &&
        (robotdata->td_leg(i) == 1 ||
         robotdata->t_leg(i) > robotdata->tst_leg(i) + robotdata->tsw_leg(i) - 0.5 * robotdata->dt)) {
      robotdata->st_leg(i) = 1;
      robotdata->t0_leg(i) = robotdata->time;
      double dt =
          robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) > 0
              ? robotdata->offset_leg(1 - i) - robotdata->offset_leg(i)
              : robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) + robotdata->tst_leg(i) + robotdata->tsw_leg(i);
      robotdata->t0_leg(1 - i) = robotdata->time - dt;
      robotdata->t_leg(i) = 0.0;
      robotdata->t_leg(1 - i) = dt;
      robotdata->late_leg(i) = 0;
      robotdata->td_leg(i) = 0;
      robotdata->isAnkleTouch(i) = 0;
      robotdata->step++;
    }

    if ((robotdata->st_leg(i) == 0 && (fabs(robotdata->grf(6 * i + 1) - robotdata->grf_pre(6 * i + 1)) >= 2.0 ||
                                       fabs(robotdata->grf(6 * i + 1)) >= 6.0))) {
      if (robotdata->t_leg(i) > robotdata->tst_leg(i) + 0.7 * robotdata->tsw_leg(i)) {
        robotdata->isAnkleTouch(i) = 1;
      }
    }

    if (robotdata->t_leg(i) < robotdata->Tc_leg - 0.5 * robotdata->dt) {
      robotdata->s_leg(i) = robotdata->t_leg(i) / robotdata->Tc_leg;
    } else if (robotdata->t_leg(i) < robotdata->tst_leg(i) - 0.5 * robotdata->dt) {
      robotdata->s_leg(i) = 1.0;
    } else if (robotdata->t_leg(i) < robotdata->tst_leg(i) + robotdata->Tc_leg - 0.5 * robotdata->dt) {
      robotdata->s_leg(i) = 1.0 - (robotdata->t_leg(i) - robotdata->tst_leg(i)) / robotdata->Tc_leg;
    } else {
      robotdata->s_leg(i) = 0.0;
    }
  }
  robotdata->phase_pre = robotdata->phase;
  if (robotdata->phase_pre == 1 && robotdata->st_leg(0) == 1 && robotdata->st_leg(1) == 1) {
    robotdata->phase = 2;
  }
  if (robotdata->phase_pre == 1 && robotdata->st_leg(0) == 0 && robotdata->st_leg(1) == 0) {
    robotdata->phase = 3;
  }
  if (robotdata->phase_pre < 4 && robotdata->st_leg(0) == 1 && robotdata->st_leg(1) == 0) {
    robotdata->phase = 4;
    robotdata->stance_index = 0;
  }
  if (robotdata->phase_pre == 4 && robotdata->st_leg(0) == 1 && robotdata->st_leg(1) == 1) {
    robotdata->phase = 5;
  }
  if (robotdata->phase_pre == 4 && robotdata->st_leg(0) == 0 && robotdata->st_leg(1) == 0) {
    robotdata->phase = 6;
  }
  if (robotdata->phase_pre > 3 && robotdata->phase_pre < 7 && robotdata->st_leg(0) == 0 && robotdata->st_leg(1) == 1) {
    robotdata->phase = 1;
    robotdata->stance_index = 1;
  }
  return true;
}

// lhj notice: update in walking
bool WalkStateEstimate::estWaistPosVelInWorld(Robot_Data* robotdata) {
  Vector3d velTorso_filt = Vector3d::Zero();
  Vector3d posTorso_l, velTorso_l, posTorso_r, velTorso_r;

  posTorso_l = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() -
               robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
  velTorso_l = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() -
               robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose();

  posTorso_r = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose() -
               robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
  velTorso_r = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose() -
               robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose();

  Eigen::Vector3d aWorld = Vector3d::Zero();
  Eigen::Vector2d trust;

  trust = robotdata->s_leg;

  aWorld = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(robotdata->q_a(5)) * robotdata->imuAcc;
  aWorld(2) = aWorld(2) - 9.81;
  // Eigen::VectorXd meas_Output = Eigen::VectorXd::Zero(8);
  // meas_Output << velTorso_l, posTorso_l(2), velTorso_r, posTorso_r(2);
  // Eigen::VectorXd est_State = Eigen::VectorXd::Zero(4);
  // est_State = leggedKalman->mFilter(meas_Output, aWorld, trust);
  // velTorso_filt = est_State.head(3);
  Eigen::VectorXd meas_Output = Eigen::VectorXd::Zero(12);
  meas_Output << posTorso_l, velTorso_l, posTorso_r, velTorso_r;
  Eigen::VectorXd est_State = Eigen::VectorXd::Zero(10);
  est_State = leggedKalman->mFilter(meas_Output, aWorld, trust);
  velTorso_filt = est_State.segment(3, 3);

  robotdata->p_odometer = est_State.head(2);
  robotdata->f_odometer = est_State.tail(4);

  if (robotdata->st_leg(0) == 1) {
    robotdata->q_a.head(3) = posTorso_l;
  } else if (robotdata->st_leg(1) == 1) {
    robotdata->q_a.head(3) = posTorso_r;
  }

  robotdata->pBodyf_a.head(3) = posTorso_l;
  robotdata->pBodyf_a.tail(3) = posTorso_r;
  robotdata->vBodyf_a.head(3) = velTorso_l;
  robotdata->vBodyf_a.tail(3) = velTorso_r;

  robotdata->pFootb_a.head(3) = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose() -
                                robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
  robotdata->pFootb_a.tail(3) = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose() -
                                robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
  robotdata->vFootb_a.head(3) = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose() -
                                robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
  robotdata->vFootb_a.tail(3) = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose() -
                                robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
  // robotdata->q_a(2) = est_State(3);
  robotdata->q_a(2) = est_State(2);
  robotdata->q_dot_a.head(3) = velTorso_filt;
  robotdata->observeData.head(3) = robotdata->q_dot_a.head(3);
  return true;
}

// lhj : calculate the com momentum
VectorNd WalkStateEstimate::estCOMStateMomentum(Robot_Data* robotdata) {
  VectorNd angularMomentumAndDot = VectorNd::Zero(6, 1);
  VectorNd jointAcc = VectorNd::Zero(generalized_coordinates, 1);
  Vector3d angularMomentum_filt;
  Scalar mass;      // 机器人总质量
  Vector3d comPos;  // 质心位置
  Vector3d comVel;

  // lhj: use comPos&comVel(as output) or x_a(as input)?
  Utils::CalcCenterOfMass(*(robotdata->robot_model), robotdata->q_a, robotdata->q_dot_a, &(robotdata->q_ddot_a), mass,
                          comPos, &comVel, NULL, &angularMomentum, &angularMomentumDot, false);

  // 侧向落地存在突变，中值滤波处理
  angularMomentum(1) = this->updateDataAndProcessPulse(angularMomentum(1), dataDeque, windowSize, threshold);
  angularMomentumAndDot.head(3) = angularMomentum;
  angularMomentumAndDot.tail(3) = angularMomentumDot;

  robotdata->TotalMass = mass;
  robotdata->comMomentum = angularMomentumAndDot;

#ifdef MOMENTUM_FILT
  robotdata->comMomentum = lowpass2->mFilter(robotdata->comMomentum);
#endif

  robotdata->observeData.segment(3, 6) = robotdata->comMomentum;
  return robotdata->comMomentum;
}

// Function to calculate the median of a deque
double WalkStateEstimate::calculateMedian(const std::deque<double>& data) {
  if (data.empty()) {
    return 0.0;  // Return 0 if the deque is empty
  }

  std::deque<double> sortedData = data;             // Copy the data to sort it
  std::sort(sortedData.begin(), sortedData.end());  // Sort the data

  size_t size = sortedData.size();
  if (size % 2 == 0) {
    // If even number of elements, take the average of the middle two
    return (sortedData[size / 2 - 1] + sortedData[size / 2]) / 2.0;
  } else {
    // If odd number of elements, take the middle one
    return sortedData[size / 2];
  }
}

// Function to update the data and process pulse if necessary
double WalkStateEstimate::updateDataAndProcessPulse(double newDataPoint, std::deque<double>& dataDeque, int windowSize,
                                                    double threshold) {
  const int maxDequeSize = 2 * windowSize;  // Set the maximum deque size to twice the window size

  // Add the new data point to the deque
  dataDeque.push_back(newDataPoint);

  // If the deque exceeds the maximum size, remove the oldest data point
  if (dataDeque.size() > maxDequeSize) {
    dataDeque.pop_front();
  }

  // Check if the last data point is a pulse
  if (dataDeque.back() > threshold) {
    std::deque<double> window;

    // Collect the window around the current point
    for (int i = -windowSize / 2; i <= windowSize / 2; ++i) {
      size_t index = dataDeque.size() - 1 + i;
      if (index < dataDeque.size() && index >= 0) {
        window.push_back(dataDeque[index]);
      }
    }

    // Calculate the median of the window
    double median = calculateMedian(window);

    // Remove the last element and insert the median
    dataDeque.pop_back();
    dataDeque.push_back(median);

    // Return the processed value
    return median;
  }

  // If no pulse detected, return the original data point
  return newDataPoint;
}