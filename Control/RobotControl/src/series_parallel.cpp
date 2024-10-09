#include "series_parallel.h"

#include "Eigen/Eigen"

namespace PND {
namespace Algorithm {
Eigen::Vector2d PosParaToSeries(Eigen::Vector2d para_positions) {
  Eigen::Vector2d series_positions;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  series_positions(0) = 0.5 * para_positions(0) + 0.5 * para_positions(1);
  series_positions(1) = -0.5 * para_positions(0) + 0.5 * para_positions(1);
#else
  series_positions(0) = 0.5 * para_positions(0) + 0.5 * para_positions(1);
  series_positions(1) = 0.5 * para_positions(0) - 0.5 * para_positions(1);
#endif
  return series_positions;
}
Eigen::Vector2d PosSeriesToPara(Eigen::Vector2d series_positions) {
  Eigen::Vector2d para_positions;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  para_positions(0) = series_positions(0) - series_positions(1);
  para_positions(1) = series_positions(0) + series_positions(1);
#else
  para_positions(0) = series_positions(0) + series_positions(1);
  para_positions(1) = series_positions(0) - series_positions(1);
#endif
  return para_positions;
}
Eigen::Vector2d VelParaToSeries(Eigen::Vector2d para_velocity) {
  Eigen::Vector2d series_velocity;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  series_velocity(0) = 0.5 * para_velocity(0) + 0.5 * para_velocity(1);
  series_velocity(1) = -0.5 * para_velocity(0) + 0.5 * para_velocity(1);
#else
  series_velocity(0) = 0.5 * para_velocity(0) + 0.5 * para_velocity(1);
  series_velocity(1) = 0.5 * para_velocity(0) - 0.5 * para_velocity(1);
#endif
  return series_velocity;
}
Eigen::Vector2d VelSeriesToPara(Eigen::Vector2d series_velocity) {
  Eigen::Vector2d para_velocity;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  para_velocity(0) = series_velocity(0) - series_velocity(1);
  para_velocity(1) = series_velocity(0) + series_velocity(1);
#else
  para_velocity(0) = series_velocity(0) + series_velocity(1);
  para_velocity(1) = series_velocity(0) - series_velocity(1);
#endif
  return para_velocity;
}
Eigen::Vector2d TauParaToSeries(Eigen::Vector2d para_torques) {
  Eigen::Vector2d series_torques;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  series_torques(0) = para_torques(0) + para_torques(1);
  series_torques(1) = -para_torques(0) + para_torques(1);
#else
  series_torques(0) = para_torques(0) + para_torques(1);
  series_torques(1) = para_torques(0) - para_torques(1);
#endif
  return series_torques;
}
Eigen::Vector2d TauSeriesToPara(Eigen::Vector2d series_torques) {
  Eigen::Vector2d para_torques;
#if defined(_CLEAN_ROBOT_LEFT_ARM)
  para_torques(0) = 0.5 * series_torques(0) - 0.5 * series_torques(1);
  para_torques(1) = 0.5 * series_torques(0) + 0.5 * series_torques(1);
#else
  para_torques(0) = 0.5 * series_torques(0) + 0.5 * series_torques(1);
  para_torques(1) = 0.5 * series_torques(0) - 0.5 * series_torques(1);
#endif
  return para_torques;
}

}  // namespace Algorithm
}  // namespace PND
