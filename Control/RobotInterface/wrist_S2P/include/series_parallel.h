#pragma once
#include "Eigen/Eigen"
namespace PND {
namespace Algorithm {
Eigen::Vector4d PosParaToSeries(Eigen::Vector4d para_positions);
Eigen::Vector4d PosSeriesToPara(Eigen::Vector4d series_positions);
Eigen::Vector4d VelParaToSeries(Eigen::Vector4d para_velocity);
Eigen::Vector4d VelSeriesToPara(Eigen::Vector4d series_velocity);
Eigen::Vector4d TauParaToSeries(Eigen::Vector4d para_torques);
Eigen::Vector4d TauSeriesToPara(Eigen::Vector4d series_torques);

} // namespace Algorithm
} // namespace PND