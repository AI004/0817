#ifndef WALKSTATEESTIMATE
#define WALKSTATEESTIMATE

#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <algorithm>
#include <deque>
#include "../../RobotControl/include/KalmanFilter.h"
#include "../../RobotControl/include/LowPassFilter.h"
#include "../../RobotControl/include/Robot_Data.h"
#include "AccKalmanFilter.h"
#include "LeggedKalmanFilter.h"
#include "planTools.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class WalkStateEstimate {
 public:
  WalkStateEstimate();
  bool estWaistPosVelInWorld(Robot_Data* robotdata, int FootType);
  bool grfEstimating(Robot_Data* robotdata);
  bool stateMachine(Robot_Data* robotdata);
  bool estWaistPosVelInWorld(Robot_Data* robotdata);
  bool generalStateMachine(Robot_Data* robotdata);
  // est COM state
  VectorNd estCOMStateMomentum(Robot_Data* robotdata);
  // pulse data filter
  double calculateMedian(const std::deque<double>& data);
  double updateDataAndProcessPulse(double newDataPoint, std::deque<double>& dataDeque, int windowSize,
                                   double threshold);

 private:
  LowPassFilter* lowpass;
  LowPassFilter* lowpass2;
  AccKalmanFilter* accKalman;
  LeggedKalmanFilter* leggedKalman;

  VectorNd qd_last_;
  Math::Vector3d angularMomentum;
  Math::Vector3d angularMomentumDot;

  int windowSize;                // Window size for the filter
  double threshold;              // Threshold for detecting pulses
  std::deque<double> dataDeque;  // Deque to store recent data points
};

#endif  // STATEESTIMATE