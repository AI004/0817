#ifndef UNIPLAN_H
#define UNIPLAN_H

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
#include "Robot_Data.h"
#include "basicfunction.h"
#include "planTools.h"

class uniPlan {
 private:
  double vZInit;
  double vZEnd;
  double zMid;
  int firstFlag;
  Eigen::VectorXd xStand;
  Eigen::Vector3d offSetL;
  Eigen::Vector3d offSetR;
  Eigen::VectorXd offSet = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qCmd_pre = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qDotCmd_pre = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd jointP = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd jointD = Eigen::VectorXd::Zero(12);
  Eigen::Vector3d pFoot, vFoot;
  double hd, hd_tgt;
  double lambda;
  double vFoot_late;  // lambda = sqrt(g/h);
  double carrybox_pOffset_x;
  std::vector<double> pOffset;
  Eigen::Vector3d pTorso_td, vTorso_td, vTorso_td_filt, pFootStrike;

  bool use_ang_mom_lip_model_ = false;
  Eigen::VectorXd WaistPosVelRelateToLeftFoot_;  // in world frame
  Eigen::Vector3d WaistPosRelateToLeftFoot_;
  Eigen::Vector3d WaistVelRelateToLeftFoot_;
  Eigen::VectorXd WaistPosVelRelateToRightFoot_;
  Eigen::Vector3d WaistPosRelateToRightFoot_;
  Eigen::Vector3d WaistVelRelateToRightFoot_;
  Eigen::Vector3d AngularMomentumAtLeftFoot_;
  Eigen::Vector3d AngularMomentumAtRightFoot_;

  double angular_momentun_y_eos_;
  double angular_momentun_x_eos_;
  Eigen::Matrix3d rot_matrix_;

  double momentum_factor_;
  double run_factor_;  // for running state, adjust the lateral foot length

  Eigen::Vector3d LeftFootPosRelativeToWaist_;
  Eigen::Vector3d LeftFootVelRelativeToWaist_;
  Eigen::Vector3d RightFootPosRelativeToWaist_;
  Eigen::Vector3d RightFootVelRelativeToWaist_;

  double m_;   // total mass
  double hx_;  // const com hight
  double hy_;  // const com hight
  double lx_;  // lip coefficient
  double ly_;  // lip coefficient

 public:
  uniPlan();
  void readParas(QString path, Robot_Data* robotdata);
  void init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_, Robot_Data* robotdata);
  bool footholdPlan(Robot_Data* robotdata);
  bool footholdPlan_AM(Robot_Data* robotdata);
  bool calVCmd(Robot_Data* robotdata);
  bool footPlan(Robot_Data* robotdata);
  bool armPlan(Robot_Data* robotdata);
  bool bodyPlan(Robot_Data* robotdata);
  bool kinePlan(Robot_Data* robotdata);
  bool wbcUpdate(Robot_Data* robotdata);
  bool locoPlan(Robot_Data* robotdata, Eigen::VectorXd& qCmd_, Eigen::VectorXd& qDotCmd_);

  void CalcWaistPosVelRelativeToFoot(Robot_Data* robotdata);
  void CalcAngularMomentumAtFoot(Robot_Data* robotdata);
};
#endif  // UNIPLAN_H