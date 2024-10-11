#ifndef GAIT_PLAN_H
#define GAIT_PLAN_H

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

class gaitPlan {
 private:
  double vZInit;
  double vZEnd;
  double zMid;
  int firstFlag;
  Eigen::VectorXd xStand;
  Eigen::Vector3d offSetL;
  Eigen::Vector3d offSetR;
  Eigen::VectorXd qCmd = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qDotCmd = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qCmd_pre = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qDotCmd_pre = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd jointP = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd jointD = Eigen::VectorXd::Zero(12);
  Eigen::Vector3d pFoot, vFoot;
  double hd;
  double lambda;  // lambda = sqrt(g/h);
  std::vector<double> pOffset;
  Eigen::Vector3d pTorso_td, vTorso_td, vTorso_td_filt, pFootStrike;

 public:
  gaitPlan();
  void readParas(QString path, Robot_Data* robotdata);
  void init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_, Robot_Data* robotdata);
  bool predTouchState(Robot_Data* robotdata);
  bool walkPlan(Robot_Data* robotdata, Eigen::VectorXd& qCmd_, Eigen::VectorXd& qDotCmd_);
  bool swingPlan(Robot_Data* robotdata);
  bool torsoPlan(Robot_Data* robotdata);
  bool cart2Joint(Robot_Data* robotdata);
  bool calVCmd(Robot_Data* robotdata);
  bool calFoothold(Robot_Data* robotdata);
  bool prePlan(Robot_Data* robotdata);
  bool setWBC(Robot_Data* robotdata);
  bool setPD(Robot_Data* robotdata);
};
#endif  // GAIT_PLAN_H