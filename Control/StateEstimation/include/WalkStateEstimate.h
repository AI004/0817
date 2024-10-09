#ifndef WALKSTATEESTIMATE
#define WALKSTATEESTIMATE

#include "../../RobotControl/include/KalmanFilter.h"
#include "../../RobotControl/include/LowPassFilter.h"
#include "../../RobotControl/include/Robot_Data.h"
#include "AccKalmanFilter.h"
#include "LeggedKalmanFilter.h"
#include "planTools.h"
#include <Eigen/Dense>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class WalkStateEstimate {
public:
  WalkStateEstimate();
  bool estWaistPosVelInWorld(Robot_Data *robotdata, int FootType);
  bool grfEstimating(Robot_Data *robotdata);
  bool stateMachine(Robot_Data *robotdata);
  bool estWaistPosVelInWorld(Robot_Data *robotdata);
  bool generalStateMachine(Robot_Data *robotdata);

private:
  LowPassFilter *lowpass;
  AccKalmanFilter *accKalman;
  LeggedKalmanFilter *leggedKalman;
};

#endif // STATEESTIMATE