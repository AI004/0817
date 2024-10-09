#pragma once
#include "Eigen/Eigen"
namespace PND {
namespace Algorithm {
Eigen::Vector2d PosParaToSeries(Eigen::Vector2d para_positions);
Eigen::Vector2d PosSeriesToPara(Eigen::Vector2d series_positions);
Eigen::Vector2d VelParaToSeries(Eigen::Vector2d para_velocity);
Eigen::Vector2d VelSeriesToPara(Eigen::Vector2d series_velocity);
Eigen::Vector2d TauParaToSeries(Eigen::Vector2d para_torques);
Eigen::Vector2d TauSeriesToPara(Eigen::Vector2d series_torques);

} // namespace Algorithm
} // namespace PND