#ifndef UNIPLAN_H
#define UNIPLAN_H

#include "Robot_Data.h"
#include "basicfunction.h"
#include "planTools.h"
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
  double vFoot_late; // lambda = sqrt(g/h);
  double carrybox_pOffset_x;
  std::vector<double> pOffset;
  Eigen::Vector3d pTorso_td, vTorso_td, vTorso_td_filt, pFootStrike;

public:
  uniPlan();
  void readParas(QString path, Robot_Data *robotdata);
  void init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_,
            Eigen::VectorXd xStand_, Robot_Data *robotdata);
  bool footholdPlan(Robot_Data *robotdata);
  bool calVCmd(Robot_Data *robotdata);
  bool footPlan(Robot_Data *robotdata);
  bool armPlan(Robot_Data *robotdata);
  bool bodyPlan(Robot_Data *robotdata);
  bool kinePlan(Robot_Data *robotdata);
  bool wbcUpdate(Robot_Data *robotdata);
  bool locoPlan(Robot_Data *robotdata, Eigen::VectorXd &qCmd_,
                Eigen::VectorXd &qDotCmd_);
};
#endif // UNIPLAN_H