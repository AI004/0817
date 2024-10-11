#include "series_parallel.h"

#include "Eigen/Eigen"

namespace PND {
namespace Algorithm {
Eigen::Vector4d PosParaToSeries(Eigen::Vector4d para_positions) {
  Eigen::Vector4d series_positions;

  series_positions(0) = 0.5 * para_positions(0) + 0.5 * para_positions(1);
  series_positions(1) = -0.5 * para_positions(0) + 0.5 * para_positions(1);

  series_positions(2) = 0.5 * para_positions(2) + 0.5 * para_positions(3);
  series_positions(3) = 0.5 * para_positions(2) - 0.5 * para_positions(3);

  return series_positions;
}
Eigen::Vector4d PosSeriesToPara(Eigen::Vector4d series_positions) {
  Eigen::Vector4d para_positions;

  para_positions(0) = series_positions(0) - series_positions(1);
  para_positions(1) = series_positions(0) + series_positions(1);

  para_positions(2) = series_positions(2) + series_positions(3);
  para_positions(3) = series_positions(2) - series_positions(3);

  return para_positions;
}
Eigen::Vector4d VelParaToSeries(Eigen::Vector4d para_velocity) {
  Eigen::Vector4d series_velocity;

  series_velocity(0) = 0.5 * para_velocity(0) + 0.5 * para_velocity(1);
  series_velocity(1) = -0.5 * para_velocity(0) + 0.5 * para_velocity(1);

  series_velocity(2) = 0.5 * para_velocity(2) + 0.5 * para_velocity(3);
  series_velocity(3) = 0.5 * para_velocity(2) - 0.5 * para_velocity(3);

  return series_velocity;
}
Eigen::Vector4d VelSeriesToPara(Eigen::Vector4d series_velocity) {
  Eigen::Vector4d para_velocity;

  para_velocity(0) = series_velocity(0) - series_velocity(1);
  para_velocity(1) = series_velocity(0) + series_velocity(1);

  para_velocity(2) = series_velocity(2) + series_velocity(3);
  para_velocity(3) = series_velocity(2) - series_velocity(3);

  return para_velocity;
}
Eigen::Vector4d TauParaToSeries(Eigen::Vector4d para_torques) {
  Eigen::Vector4d series_torques;

  series_torques(0) = para_torques(0) + para_torques(1);
  series_torques(1) = -para_torques(0) + para_torques(1);

  series_torques(2) = para_torques(2) + para_torques(3);
  series_torques(3) = para_torques(2) - para_torques(3);

  return series_torques;
}
Eigen::Vector4d TauSeriesToPara(Eigen::Vector4d series_torques) {
  Eigen::Vector4d para_torques;

  para_torques(0) = 0.5 * series_torques(0) - 0.5 * series_torques(1);
  para_torques(1) = 0.5 * series_torques(0) + 0.5 * series_torques(1);

  para_torques(2) = 0.5 * series_torques(2) + 0.5 * series_torques(3);
  para_torques(3) = 0.5 * series_torques(2) - 0.5 * series_torques(3);

  return para_torques;
}

}  // namespace Algorithm
}  // namespace PND
