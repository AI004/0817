#include "gaitPlan.h"

gaitPlan::gaitPlan() {}
void gaitPlan::init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_, Robot_Data* robotdata) {
  qCmd = qCmd_;
  qDotCmd = qDotCmd_;
  firstFlag = 1;
  vZInit = 0.0;
  vZEnd = -0.0;
  zMid = robotdata->foot_zMid;
  xStand.resize(6);
  xStand = xStand_;
  offSetL << 0.0, 0.105, 0.0;
  offSetR << 0.0, -0.105, 0.0;
  jointP << 3000., 3000., 467., 312., 582., 582.;
  jointD << 30., 30., 15., 15., 11.64, 11.64;

  hd = robotdata->hd;
  pOffset.resize(3);
  pOffset[0] = robotdata->poffset_x;
  pOffset[1] = robotdata->poffset_y;
  pOffset[2] = robotdata->poffset_z;  // stand has these paras too.
  vTorso_td_filt << 0.0, 0.0, 0.0;
}
bool gaitPlan::walkPlan(Robot_Data* robotdata, Eigen::VectorXd& qCmd_, Eigen::VectorXd& qDotCmd_) {
  if (robotdata->carryBoxState == 1) {
    pOffset[0] = robotdata->poffset_x + 0.02;
  } else {
    pOffset[0] = robotdata->poffset_x;
  }

  gaitPlan::calVCmd(robotdata);
  gaitPlan::predTouchState(robotdata);
  gaitPlan::calFoothold(robotdata);
  gaitPlan::prePlan(robotdata);

  gaitPlan::swingPlan(robotdata);
  gaitPlan::torsoPlan(robotdata);
  gaitPlan::cart2Joint(robotdata);
  gaitPlan::setWBC(robotdata);
  gaitPlan::setPD(robotdata);

  qCmd_ = qCmd;
  qDotCmd_ = qDotCmd;

  if (robotdata->t < robotdata->Tc - 0.5 * robotdata->dt) {
    qCmd_ = (1.0 - robotdata->sc) * robotdata->qCmd_td + robotdata->sc * qCmd;
    qDotCmd_ = (1.0 - robotdata->sc) * robotdata->qDotCmd_td + robotdata->sc * robotdata->q_dot_a.tail(12);
  } else if (robotdata->t < 2 * robotdata->Tc - 0.5 * robotdata->dt) {
    double sc2 = (robotdata->t - robotdata->Tc) / robotdata->Tc;
    qDotCmd_ = (1.0 - sc2) * robotdata->q_dot_a.tail(12) + sc2 * qDotCmd;
  }
  return true;
}
bool gaitPlan::torsoPlan(Robot_Data* robotdata) {
  robotdata->rTorso_d = basicfunction::RotX(-0.15 * robotdata->avg_vx * robotdata->vyaw);

  if (robotdata->t < robotdata->Tc - 0.5 * robotdata->dt) {
    robotdata->pTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->pTorso_tgt(2) = robotdata->pTorso_td(2);
    robotdata->vTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
    robotdata->vTorso_tgt(2) = 0.0;
    robotdata->pTorso_ini = robotdata->pTorso_tgt;
    robotdata->vTorso_ini = robotdata->vTorso_tgt;
    robotdata->rTorso_tgt = robotdata->rTorso_d;
  } else {
    robotdata->pTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->vTorso_tgt = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
    Thirdpoly(robotdata->pTorso_ini(2), robotdata->vTorso_ini(2), robotdata->pTorso_end(2), robotdata->vTorso_end(2),
              robotdata->T, robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pTorso_tgt(2),
              robotdata->vTorso_tgt(2));
    // Thirdpoly(robotdata->pTorso_tgt(2), robotdata->vTorso_tgt(2),
    // robotdata->pTorso_end(2), robotdata->vTorso_end(2), robotdata->T,
    // robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pTorso_tgt(2) ,
    // robotdata->vTorso_tgt(2));

    robotdata->rTorso_tgt = robotdata->rTorso_d;
  }
  robotdata->pTorso_kin_tgt = robotdata->pTorso_tgt;
  robotdata->vTorso_kin_tgt = robotdata->vTorso_tgt;

  // robotdata->vTorso_tgt(0) = robotdata->vCmd(0);
  // robotdata->vTorso_tgt(1) = 0.*(robotdata->vCmd(1) + robotdata->vCmd(2));
  // if(robotdata->stance_index==0)
  //     robotdata->pTorso_tgt(1) = -0.1;
  // else
  //     robotdata->pTorso_tgt(1) = 0.1;

  return true;
}

bool gaitPlan::predTouchState(Robot_Data* robotdata) {
  if (robotdata->carryBoxState == 1) {
    pOffset[0] = robotdata->poffset_x + 0.02;
  } else {
    pOffset[0] = robotdata->poffset_x;
  }
  double e1, e2, c1, c2;

  lambda = std::sqrt(9.81 / (hd + pOffset[2]));
  double eta_com = 0.05;
  robotdata->v_com_filt = (1.0 - eta_com) * robotdata->v_com_filt + eta_com * robotdata->v_com;

  e1 = std::exp(lambda * (robotdata->T + robotdata->Tc - robotdata->t));
  e2 = std::exp(-lambda * (robotdata->T + robotdata->Tc - robotdata->t));

  // c1 = 0.5*(robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) +
  // pOffset[0] + 1.0/lambda*robotdata->q_dot_a(0)); c2 =
  // 0.5*(robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) +
  // pOffset[0] - 1.0/lambda*robotdata->q_dot_a(0));

  // pTorso_td(0) = c1*e1 + c2*e2 - pOffset[0] -
  // robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) +
  // robotdata->q_a(0); vTorso_td(0) = lambda*(c1*e1 - c2*e2);

  // c1 = 0.5*(robotdata->q_a(1) + pOffset[1]
  // + 1.0/lambda*robotdata->q_dot_a(1)); c2 = 0.5*(robotdata->q_a(1) +
  // pOffset[1] - 1.0/lambda*robotdata->q_dot_a(1));

  // pTorso_td(1) = c1*e1 + c2*e2;
  // vTorso_td(1) = lambda*(c1*e1 - c2*e2);

  //-------- use constant COM offset----------//
  c1 = 0.5 * (robotdata->q_a(0) + pOffset[0] + 1.0 / lambda * robotdata->q_dot_a(0));
  c2 = 0.5 * (robotdata->q_a(0) + pOffset[0] - 1.0 / lambda * robotdata->q_dot_a(0));

  pTorso_td(0) = c1 * e1 + c2 * e2 - pOffset[0];
  vTorso_td(0) = lambda * (c1 * e1 - c2 * e2);

  c1 = 0.5 * (robotdata->q_a(1) + pOffset[1] + 1.0 / lambda * robotdata->q_dot_a(1));
  c2 = 0.5 * (robotdata->q_a(1) + pOffset[1] - 1.0 / lambda * robotdata->q_dot_a(1));

  pTorso_td(1) = c1 * e1 + c2 * e2;
  vTorso_td(1) = lambda * (c1 * e1 - c2 * e2);

  // filter
  double ita = 0.024;
  if (robotdata->t < 0.3 * robotdata->T + robotdata->Tc) {
    ita = 0.08;
  } else {
    ita = 0.08;
  }
  vTorso_td_filt(0) = ita * vTorso_td(0) + (1.0 - ita) * vTorso_td_filt(0);
  vTorso_td_filt(1) = ita * vTorso_td(1) + (1.0 - ita) * vTorso_td_filt(1);
  // if(robotdata->t < 0.5*robotdata->dt){
  //     vTorso_td_filt(1) = vTorso_td(1);
  // }
  robotdata->temp2.head(3) = vTorso_td_filt;
  robotdata->temp2.tail(3) = vTorso_td;
  vTorso_td = vTorso_td_filt;
  // vx_com mean
  int N = robotdata->T / robotdata->dt;
  double ite_vx = 1.0 / N;
  robotdata->avg_vx = ite_vx * robotdata->v_com(0) + (1.0 - ite_vx) * robotdata->avg_vx;

  robotdata->odometer_avg = ite_vx * robotdata->odometer + (1.0 - ite_vx) * robotdata->odometer_avg;

  return true;
}
bool gaitPlan::setPD(Robot_Data* robotdata) {
  double ratio = 0.9;

  robotdata->q_factor.head(12) << (1.0 - ratio * robotdata->sL) * Eigen::VectorXd::Ones(6),
      (1.0 - ratio * robotdata->sR) * Eigen::VectorXd::Ones(6);
  robotdata->q_dot_factor.head(12) << (1.0 - ratio * robotdata->sL) * Eigen::VectorXd::Ones(6),
      (1.0 - ratio * robotdata->sR) * Eigen::VectorXd::Ones(6);

  robotdata->q_factor.segment(4, 2) << (1.0 - robotdata->sL), (1.0 - robotdata->sL);
  robotdata->q_dot_factor.segment(4, 2) << (1.0 - robotdata->sL), (1.0 - robotdata->sL);
  robotdata->q_factor.segment(10, 2) << (1.0 - robotdata->sR), (1.0 - robotdata->sR);
  robotdata->q_dot_factor.segment(10, 2) << (1.0 - robotdata->sR), (1.0 - robotdata->sR);

  if (robotdata->step < 1) {
    if (robotdata->stance_index == 1) {
      robotdata->q_factor.head(12) << (1.0 - ratio * robotdata->sL) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio) * Eigen::VectorXd::Ones(6);
      robotdata->q_dot_factor.head(12) << (1.0 - ratio * robotdata->sL) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio) * Eigen::VectorXd::Ones(6);
      robotdata->q_factor.segment(10, 2) << 0.0, 0.0;
      robotdata->q_dot_factor.segment(10, 2) << 0.0, 0.0;
      robotdata->q_factor.segment(4, 2) << (1.0 - ratio * robotdata->sL), (1.0 - ratio * robotdata->sL);
      robotdata->q_dot_factor.segment(4, 2) << (1.0 - ratio * robotdata->sL), (1.0 - ratio * robotdata->sL);
    } else {
      robotdata->q_factor.head(12) << (1.0 - ratio) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio * robotdata->sR) * Eigen::VectorXd::Ones(6);
      robotdata->q_dot_factor.head(12) << (1.0 - ratio) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio * robotdata->sR) * Eigen::VectorXd::Ones(6);
      robotdata->q_factor.segment(4, 2) << 0.0, 0.0;
      robotdata->q_dot_factor.segment(4, 2) << 0.0, 0.0;
      robotdata->q_factor.segment(10, 2) << (1.0 - ratio * robotdata->sR), (1.0 - ratio * robotdata->sR);
      robotdata->q_dot_factor.segment(10, 2) << (1.0 - ratio * robotdata->sR), (1.0 - ratio * robotdata->sR);
    }
  }
  return true;
}

bool gaitPlan::calFoothold(Robot_Data* robotdata) {
  double d2, sigma1, sigma2, kp_star;
  d2 = robotdata->vCmd(1) + robotdata->vCmd(2);

  sigma1 = robotdata->sigma_ratio_x * lambda / std::tanh(0.5 * robotdata->Td * lambda);
  sigma2 = robotdata->sigma_ratio_y * lambda * std::tanh(0.5 * robotdata->Td * lambda);

  kp_star = 1. / lambda / std::sinh(robotdata->Td * lambda);

  pFootStrike(0) =
      vTorso_td(0) / sigma1 + robotdata->kp_ratio_x * kp_star * (vTorso_td(0) - robotdata->vCmd(0)) + pOffset[0];

  // pFootStrike(0) =  vTorso_td(0)/sigma1 +
  // 0.8*kp_star*(vTorso_td(0)-robotdata->vCmd(0)) + pOffset[0] +
  // robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) -
  // robotdata->q_a(0);

  if (robotdata->stance_index == 1) {
    pFootStrike(1) = (vTorso_td(1) - d2) / sigma2 -
                     robotdata->kp_ratio_y * kp_star * (vTorso_td(1) - robotdata->vCmd(1)) + pOffset[1];
  } else {
    pFootStrike(1) = (vTorso_td(1) - d2) / sigma2 -
                     robotdata->kp_ratio_y * kp_star * (vTorso_td(1) - robotdata->vCmd(2)) + pOffset[1];
  }

  // change period T
  // if(robotdata->s > 0.3 && robotdata->s<0.8 && ((robotdata->stance_index==1
  // && pFootStrike(1) + robotdata->q_a(1)<0.101) || (robotdata->stance_index==0
  // && pFootStrike(1) + robotdata->q_a(1)>-0.101) || fabs(pFootStrike(1)) >
  // 0.249)){
  if (robotdata->s > 0.3 && robotdata->s < 0.8 &&
      (fabs(pFootStrike(0)) > robotdata->foot_limit_x || fabs(pFootStrike(1)) < robotdata->foot_limit_yin ||
       fabs(pFootStrike(1)) > robotdata->foot_limit_yout)) {
    robotdata->T = robotdata->Td2;
    robotdata->t = robotdata->s * robotdata->T + robotdata->Tc;
    robotdata->t_switch = robotdata->time - robotdata->t;

    gaitPlan::predTouchState(robotdata);

    pFootStrike(0) =
        vTorso_td(0) / sigma1 + robotdata->kp_ratio_x * kp_star * (vTorso_td(0) - robotdata->vCmd(0)) + pOffset[0];
    // pFootStrike(0) =  vTorso_td(0)/sigma1 +
    // 0.8*kp_star*(vTorso_td(0)-robotdata->vCmd(0)) + pOffset[0] +
    // robotdata->task_card_set[robotdata->com_task_id]->X_a(0,3) -
    // robotdata->q_a(0);

    if (robotdata->stance_index == 1) {
      pFootStrike(1) = (vTorso_td(1) - d2) / sigma2 -
                       robotdata->kp_ratio_y * kp_star * (vTorso_td(1) - robotdata->vCmd(1)) + pOffset[1];
    } else {
      pFootStrike(1) = (vTorso_td(1) - d2) / sigma2 -
                       robotdata->kp_ratio_y * kp_star * (vTorso_td(1) - robotdata->vCmd(2)) + pOffset[1];
    }
  }

  pFootStrike(2) = 0.0;

  if (robotdata->stance_index == 1) {
    if (pFootStrike(1) < robotdata->foot_limit_yin) pFootStrike(1) = robotdata->foot_limit_yin;
    if (pFootStrike(1) > robotdata->foot_limit_yout) pFootStrike(1) = robotdata->foot_limit_yout;
  } else {
    if (pFootStrike(1) > -robotdata->foot_limit_yin) pFootStrike(1) = -robotdata->foot_limit_yin;
    if (pFootStrike(1) < -robotdata->foot_limit_yout) pFootStrike(1) = -robotdata->foot_limit_yout;
  }
  if (pFootStrike(0) > robotdata->foot_limit_x) {
    pFootStrike(0) = robotdata->foot_limit_x;
  }
  if (pFootStrike(0) < -robotdata->foot_limit_x) {
    pFootStrike(0) = -robotdata->foot_limit_x;
  }
  return true;
}
bool gaitPlan::prePlan(Robot_Data* robotdata) {
  if (robotdata->stance_index == 0) {
    pFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    vFoot = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose();
  } else {
    pFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
    vFoot = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose();
  }
  if (robotdata->t < 0.5 * robotdata->dt) {
    robotdata->qCmd_td = qCmd;
    robotdata->qDotCmd_td = qDotCmd;

    robotdata->pTorso_td = (-robotdata->pFootb_tgt);
    robotdata->vTorso_td = (-robotdata->vFootb_tgt);

    robotdata->pFootb_td = (-robotdata->pTorso_tgt);
    robotdata->vFootb_td = (-robotdata->vTorso_tgt);

    robotdata->pFoot_td = pFoot;
    robotdata->vFoot_td = vFoot;

    robotdata->pArm_td = robotdata->pArm_tgt;
    robotdata->vArm_td = robotdata->vArm_tgt;
  }

  robotdata->pFootb_end(0) = pFootStrike(0);
  robotdata->pFootb_end(1) = pFootStrike(1);
  robotdata->pFootb_end(2) = pFootStrike(2) - robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 5);

  robotdata->temp.head(3) = pFootStrike;

  robotdata->vFootb_end(0) = -vTorso_td(0);
  robotdata->vFootb_end(1) = 0.0;    //-vTorso_td(1);
  robotdata->vFootb_end(2) = vZEnd;  //-robotdata->task_card_set[robotdata->body_task_id]->X_a(1,5);

  robotdata->temp.block(3, 0, 3, 1) = robotdata->vFootb_end;

  robotdata->pTorso_end(2) = hd;
  robotdata->vTorso_end(2) = 0.0;

  robotdata->rTorso = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(0.0);
  robotdata->rTorso_d = Eigen::Matrix3d::Identity();

  double sc, sL, sR, sfc, sfL, sfR;
  if (robotdata->t < robotdata->Tc - 0.5 * robotdata->dt) {
    sc = robotdata->t / robotdata->Tc;
  } else {
    sc = 1.0;
  }
  if (robotdata->stance_index == 0) {
    sL = sc;
    sR = 1.0 - sc;
  } else {
    sR = sc;
    sL = 1.0 - sR;
  }
  robotdata->sL = sL;
  robotdata->sR = sR;
  robotdata->sc = sc;

  if (robotdata->t < robotdata->t_ftd - 0.5 * robotdata->dt) {
    sfc = 0.0;
  } else if (robotdata->t < robotdata->Tc + robotdata->t_ftd - 0.5 * robotdata->dt) {
    sfc = (robotdata->t - robotdata->t_ftd) / robotdata->Tc;
  } else {
    sfc = 1.0;
  }
  if (robotdata->stance_index == 0) {
    sfL = sfc;
    sfR = 1.0 - sfc;
  } else {
    sfR = sfc;
    sfL = 1.0 - sfR;
  }

  robotdata->sfL = sfL;
  robotdata->sfR = sfR;
  robotdata->sfc = sfc;
  return true;
}
bool gaitPlan::setWBC(Robot_Data* robotdata) {
  // Weights
  Eigen::VectorXd weight_FootForce = Eigen::VectorXd::Zero(12);
  weight_FootForce << 0.1, 0.1, 0.1, 0.1, 0.1, 0.001, 0.1, 0.1, 0.1, 0.1, 0.1, 0.001;
  for (int i = 0; i < 12; ++i) {
    robotdata->WF1(i, i) = weight_FootForce(i);
  }

  robotdata->WF2 = 0.001 * Eigen::MatrixXd::Identity(12, 12);
  robotdata->WF2(5, 5) = 0.0001;
  robotdata->WF2(11, 11) = 0.0001;

  // robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
  // 100., 0., 0., 100.;
  robotdata->task_card_set[robotdata->body_task_id]->weight << robotdata->roll_weight, robotdata->pitch_weight, 100.,
      robotdata->px_weight, robotdata->py_weight, 100.;
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      (robotdata->sL * 1000. + robotdata->sR * 100.) * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      (robotdata->sR * 1000. + robotdata->sL * 100.) * Eigen::VectorXd::Ones(6);

  // Bounds
  robotdata->tau_ub << 82.5, 82.5, 150.0, 150.0, 32.0, 32.0, 82.5, 82.5, 150.0, 150.0, 32.0, 32.0, 82.5, 82.5, 82.5,
      18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0;
  robotdata->tau_lb = -robotdata->tau_ub;

  // robotdata->tau_ub.tail(6) =  (1.0 - sL)*robotdata->tau_ub.tail(6) +
  // sL*torSw.tail(6); robotdata->tau_lb.tail(6) =  (1.0 -
  // sL)*robotdata->tau_lb.tail(6) + sL*torSw.tail(6); robotdata->tau_ub.head(6)
  // =  (1.0 - sR)*robotdata->tau_ub.head(6) + sR*torSw.head(6);
  // robotdata->tau_lb.head(6) =  (1.0 - sR)*robotdata->tau_lb.head(6) +
  // sR*torSw.head(6); if(robotdata->step < 1){
  //     robotdata->tau_ub.head(6) =  torSw.head(6);
  //     robotdata->tau_lb.head(6) =  torSw.head(6);
  // }

  double my_inf = 1000.0;
  robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, robotdata->sL * 1.5 * robotdata->MG, my_inf, my_inf,
      my_inf, my_inf, my_inf, robotdata->sR * 1.5 * robotdata->MG;
  // if(robotdata->step < 1){
  //     if(robotdata->stance_index==1){
  //         robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, 0.0,
  //         my_inf, my_inf, my_inf, my_inf, my_inf, 1.5*robotdata->MG;
  //     }
  //     else{
  //         robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf,
  //         my_inf, 1.5*robotdata->MG, my_inf, my_inf, my_inf, my_inf, my_inf,
  //         0.0;
  //     }
  // }
  robotdata->GRF_lb = -robotdata->GRF_ub;
  robotdata->GRF_lb(5) = 0.0;
  robotdata->GRF_lb(11) = 0.0;
  // ankle control
  double akRoll_tor = 0.0;
  double akPitch_tor = 0.0;
  if (robotdata->touch_index == 4) {
    akPitch_tor = 24.0 * (robotdata->vCmd[0] - vTorso_td[0]);
    akRoll_tor = -18.0 * (robotdata->vCmd[2] - vTorso_td[1]);
  } else if (robotdata->touch_index == 2) {
    akPitch_tor = 24.0 * (robotdata->vCmd[0] - vTorso_td[0]);
    akRoll_tor = -18.0 * (robotdata->vCmd[1] - vTorso_td[1]);
  } else {
    akPitch_tor = 0.0;
    akRoll_tor = 0.0;
  }

  akPitch_tor = std::min(7.0, std::max(-7.0, akPitch_tor));
  akRoll_tor = std::min(2.0, std::max(-2.0, akRoll_tor));

  if (robotdata->stance_index == 0) {
    robotdata->GRF_ub(0) = robotdata->sL * akRoll_tor;
    robotdata->GRF_ub(1) = robotdata->sL * akPitch_tor;
    robotdata->GRF_lb(0) = robotdata->sL * akRoll_tor;
    robotdata->GRF_lb(1) = robotdata->sL * akPitch_tor;
    robotdata->GRF_ub(6) = 0.0;
    robotdata->GRF_ub(7) = 0.0;
    robotdata->GRF_lb(6) = 0.0;
    robotdata->GRF_lb(7) = 0.0;
  } else {
    robotdata->GRF_ub(6) = robotdata->sR * akRoll_tor;
    robotdata->GRF_ub(7) = robotdata->sR * akPitch_tor;
    robotdata->GRF_lb(6) = robotdata->sR * akRoll_tor;
    robotdata->GRF_lb(7) = robotdata->sR * akPitch_tor;
    robotdata->GRF_ub(0) = 0.0;
    robotdata->GRF_ub(1) = 0.0;
    robotdata->GRF_lb(0) = 0.0;
    robotdata->GRF_lb(1) = 0.0;
  }
  return true;
}

bool gaitPlan::swingPlan(Robot_Data* robotdata) {
  if (robotdata->t < robotdata->Tc - 0.5 * robotdata->dt) {
    robotdata->pFootb_tgt =
        pFoot - robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->vFootb_tgt =
        Eigen::VectorXd::Zero(3);  //(vFoot -
                                   // robotdata->task_card_set[robotdata->body_id]->X_a.block(1,3,1,3).transpose());
    robotdata->pFootb_ini = robotdata->pFootb_tgt;
    robotdata->vFootb_ini = robotdata->vFootb_tgt;
    robotdata->rFoot_tgt = robotdata->rTorso;
  } else {
    for (int i = 0; i < 2; i++) {
      // Thirdpoly(robotdata->pFootb_ini(i), robotdata->vFootb_ini(i),
      // robotdata->pFootb_end(i), robotdata->vFootb_end(i), robotdata->T,
      // robotdata->t - robotdata->Tc + robotdata->dt, robotdata->pFootb_tgt(i) ,
      // robotdata->vFootb_tgt(i));
      Thirdpoly(robotdata->pFootb_tgt(i), robotdata->vFootb_tgt(i), robotdata->pFootb_end(i), robotdata->vFootb_end(i),
                robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pFootb_tgt(i),
                robotdata->vFootb_tgt(i));
    }
    double xDDotTrans;
    // TriPointsQuintic(robotdata->T, robotdata->t - robotdata->Tc +
    // robotdata->dt, robotdata->pFootb_ini(2), robotdata->vFootb_ini(2),
    // zMid+robotdata->pFootb_end(2), 0., robotdata->pFootb_end(2),
    // robotdata->vFootb_end(2), robotdata->pFootb_tgt(2),
    // robotdata->vFootb_tgt(2), xDDotTrans);
    // foot z replan
    double tMid_ = 0.5 * robotdata->T - 0.5 * robotdata->Tc + 0.5 * robotdata->t;
    double zMid_, vMid_;
    TriPointsQuintic(robotdata->T, tMid_, robotdata->pFootb_ini(2), robotdata->vFootb_ini(2),
                     zMid + robotdata->pFootb_end(2), 0., robotdata->pFootb_end(2), robotdata->vFootb_end(2), zMid_,
                     vMid_, xDDotTrans);
    TriPointsQuintic(robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pFootb_tgt(2),
                     robotdata->vFootb_tgt(2), zMid_, vMid_, robotdata->pFootb_end(2), robotdata->vFootb_end(2),
                     robotdata->pFootb_tgt(2), robotdata->vFootb_tgt(2), xDDotTrans);

    robotdata->rFoot_tgt = robotdata->rTorso;
  }

  // ankle pose plan
  if (robotdata->t < 0.5 * robotdata->dt) {
    // if(robotdata->vCmd(0)>-0.2){
    if (robotdata->stance_index == 0) {
      robotdata->rFoot_d = basicfunction::RotZ(-robotdata->rotateAngle) * basicfunction::RotY(M_PI / 60.);
    } else {
      robotdata->rFoot_d = basicfunction::RotZ(robotdata->rotateAngle) * basicfunction::RotY(M_PI / 60.);
    }
    // }else{
    //     robotdata->rFoot_d = basicfunction::RotY(M_PI/60.);
    // }
  }

  if (robotdata->stance_index == 0) {
    basicfunction::Euler_XYZToMatrix(
        robotdata->rFoot_l, robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 0, 1, 3).transpose());
    if (robotdata->t < 2.0 * robotdata->Tc) {
      basicfunction::Euler_XYZToMatrix(
          robotdata->rFoot_r, robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 0, 1, 3).transpose());
      robotdata->rFoot_rtd = robotdata->rFoot_r;
    } else {
      Eigen::Vector3d omiga_d, acc_d;
      quaternionInterp(robotdata->rFoot_rtd, robotdata->rFoot_d, 0.8 * robotdata->T - 2.0 * robotdata->Tc,
                       robotdata->t - 2.0 * robotdata->Tc + robotdata->dt, robotdata->rFoot_r, omiga_d, acc_d);
      // robotdata->rFoot_r = Eigen::Matrix3d::Identity();
    }
  } else {
    basicfunction::Euler_XYZToMatrix(
        robotdata->rFoot_r, robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 0, 1, 3).transpose());
    if (robotdata->t < 2.0 * robotdata->Tc) {
      basicfunction::Euler_XYZToMatrix(
          robotdata->rFoot_l, robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 0, 1, 3).transpose());
      robotdata->rFoot_ltd = robotdata->rFoot_l;
    } else {
      Eigen::Vector3d omiga_d, acc_d;
      quaternionInterp(robotdata->rFoot_ltd, robotdata->rFoot_d, 0.8 * robotdata->T - 2.0 * robotdata->Tc,
                       robotdata->t - 2.0 * robotdata->Tc + robotdata->dt, robotdata->rFoot_l, omiga_d, acc_d);
      // robotdata->rFoot_l = Eigen::Matrix3d::Identity();
    }
  }

  // arm swing plan
  if (robotdata->t < 0.8 * robotdata->T + robotdata->Tc) {
    if (robotdata->stance_index == 1) {
      Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), -(robotdata->pFootb_end(0) - 0.0) / 0.8 + 0.42, 0.0,
                0.8 * robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0),
                robotdata->vArm_tgt(0));
      Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), (robotdata->pFootb_end(0) - 0.0) / 0.8 + 0.42, 0.0,
                0.8 * robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1),
                robotdata->vArm_tgt(1));
    } else {
      Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0), (robotdata->pFootb_end(0) - 0.0) / 0.8 + 0.42, 0.0,
                0.8 * robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0),
                robotdata->vArm_tgt(0));
      Thirdpoly(robotdata->pArm_tgt(1), robotdata->vArm_tgt(1), -(robotdata->pFootb_end(0) - 0.0) / 0.8 + 0.42, 0.0,
                0.8 * robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(1),
                robotdata->vArm_tgt(1));
    }
    robotdata->pArm_tgt(0) = std::min(std::max(robotdata->pArm_tgt(0), 0.0), 1.0);
    robotdata->pArm_tgt(1) = std::min(std::max(robotdata->pArm_tgt(1), 0.0), 1.0);
  }

  // if(robotdata->stance_index==1){
  //         Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0),
  //         -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T +
  //         robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0)
  //         , robotdata->vArm_tgt(0)); Thirdpoly(robotdata->pArm_tgt(1),
  //         robotdata->vArm_tgt(1), (robotdata->pFootb_end(0)-0.0)/0.8 + 0.5,
  //         0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt,
  //         robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
  // }else{
  //         Thirdpoly(robotdata->pArm_tgt(0), robotdata->vArm_tgt(0),
  //         (robotdata->pFootb_end(0)-0.0)/0.8 + 0.5, 0.0, robotdata->T +
  //         robotdata->Tc - robotdata->t, robotdata->dt, robotdata->pArm_tgt(0)
  //         , robotdata->vArm_tgt(0)); Thirdpoly(robotdata->pArm_tgt(1),
  //         robotdata->vArm_tgt(1), -(robotdata->pFootb_end(0)-0.0)/0.8 + 0.5,
  //         0.0, robotdata->T + robotdata->Tc - robotdata->t, robotdata->dt,
  //         robotdata->pArm_tgt(1) , robotdata->vArm_tgt(1));
  // }
  // robotdata->pArm_tgt(0) =
  // std::min(std::max(robotdata->pArm_tgt(0),0.0),1.0); robotdata->pArm_tgt(1)
  // = std::min(std::max(robotdata->pArm_tgt(1),0.0),1.0);

  return true;
}

bool gaitPlan::calVCmd(Robot_Data* robotdata) {
  if (robotdata->time < 0.5 * robotdata->dt) {
    robotdata->odometer_d = robotdata->odometer;
  }
  // joystick
  if ((fabs(robotdata->vCmd_joystick(0)) > 0.1)) {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0009)) {
      robotdata->vCmd(0) += 0.0009 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
                            fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  } else {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0015)) {
      robotdata->vCmd(0) += 0.0015 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
                            fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  }

  robotdata->odometer_d(1) += robotdata->vCmd_offset_joystick(1);
  robotdata->vy = 0.1 * (robotdata->odometer_d(1) - robotdata->odometer_avg(1));
  //
  double vy_offset = robotdata->vy_offset;
  if (robotdata->avg_vx > 0.1) {
    vy_offset = std::max(0.1, robotdata->vy_offset - robotdata->vy_change_ratio * (fabs(robotdata->avg_vx) - 0.1));
  }
  if (robotdata->avg_vx < -0.1) {
    vy_offset = std::max(0.1, robotdata->vy_offset + robotdata->vy_change_ratio * (fabs(robotdata->avg_vx) - 0.1));
  }

  robotdata->vCmd(1) = vy_offset + robotdata->vy;
  robotdata->vCmd(2) = -vy_offset + robotdata->vy;
  return true;
}

bool gaitPlan::cart2Joint(Robot_Data* robotdata) {
  Eigen::Vector3d PosL, PosR;
  Eigen::Vector3d RPY(0.0, 0.0, 0.0);
  Eigen::VectorXd qIKL = Eigen::VectorXd::Zero(6), qIKR = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd xIKL_dot = Eigen::VectorXd::Zero(6), xIKR_dot = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd qIKL_dot = Eigen::VectorXd::Zero(6), qIKR_dot = Eigen::VectorXd::Zero(6);
  if (robotdata->stance_index == 0) {
    PosL = robotdata->rTorso_tgt.transpose() * (-robotdata->pTorso_kin_tgt) - offSetL;
    PosR = robotdata->rFoot_tgt.transpose() * (robotdata->pFootb_tgt) - offSetR;
    oneLegIK2(PosL, robotdata->rTorso_tgt.transpose() * robotdata->rFoot_l, qIKL);
    oneLegIK2(PosR, robotdata->rFoot_tgt.transpose() * robotdata->rFoot_r, qIKR);
  } else {
    PosL = robotdata->rFoot_tgt.transpose() * (robotdata->pFootb_tgt) - offSetL;
    PosR = robotdata->rTorso_tgt.transpose() * (-robotdata->pTorso_kin_tgt) - offSetR;
    oneLegIK2(PosL, robotdata->rFoot_tgt.transpose() * robotdata->rFoot_l, qIKL);
    oneLegIK2(PosR, robotdata->rTorso_tgt.transpose() * robotdata->rFoot_r, qIKR);
  }
  qCmd_pre = qCmd;

  qCmd.head(6) = qIKL;
  qCmd.tail(6) = qIKR;
  // todo velocity ik
  Eigen::VectorXd qCmd_full = Eigen::VectorXd::Zero(18);
  qCmd_full.tail(12) = qCmd;
  Eigen::MatrixXd J_6D;
  J_6D.setZero(6, robotdata->ndof);
  if (robotdata->stance_index == 0) {
    // left foot
    qCmd_full.segment(3, 3) = matrixtoeulerxyz_(robotdata->rTorso_tgt);
    RigidBodyDynamics::CalcPointJacobian6D(
        *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->left_foot_id]->joint_id,
        robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
    xIKL_dot.setZero();
    xIKL_dot(2) = -robotdata->vyaw;
    xIKL_dot.tail(3) = -robotdata->vTorso_kin_tgt;
    qIKL_dot = J_6D.block(0, 6, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIKL_dot);
    // right foot
    qCmd_full.segment(3, 3) = matrixtoeulerxyz_(robotdata->rFoot_tgt);
    RigidBodyDynamics::CalcPointJacobian6D(
        *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->right_foot_id]->joint_id,
        robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
    xIKR_dot.setZero();
    xIKR_dot.tail(3) = robotdata->vFootb_tgt;
    qIKR_dot = J_6D.block(0, 12, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIKR_dot);
  } else {
    // left foot
    qCmd_full.segment(3, 3) = matrixtoeulerxyz_(robotdata->rFoot_tgt);
    RigidBodyDynamics::CalcPointJacobian6D(
        *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->left_foot_id]->joint_id,
        robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
    xIKL_dot.setZero();
    xIKL_dot.tail(3) = robotdata->vFootb_tgt;
    qIKL_dot = J_6D.block(0, 6, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIKL_dot);
    // right foot
    qCmd_full.segment(3, 3) = matrixtoeulerxyz_(-robotdata->rTorso_tgt);
    RigidBodyDynamics::CalcPointJacobian6D(
        *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->right_foot_id]->joint_id,
        robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
    xIKR_dot.setZero();
    xIKR_dot(2) = -robotdata->vyaw;
    xIKR_dot.tail(3) = -robotdata->vTorso_kin_tgt;
    qIKR_dot = J_6D.block(0, 12, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIKR_dot);
  }
  robotdata->qdotcmd_temp.head(6) = qIKL_dot;
  robotdata->qdotcmd_temp.tail(6) = qIKR_dot;

  qDotCmd.head(6) = qIKL_dot;
  qDotCmd.tail(6) = qIKR_dot;
  // if(!firstFlag)
  //     qDotCmd = (qCmd - qCmd_pre)/robotdata->dt;
  // firstFlag = 0;
  return true;
}

void gaitPlan::readParas(QString path, Robot_Data* robotdata) {
  // read the json file
  QFile loadFile(path);
  if (!loadFile.open(QIODevice::ReadOnly)) {
    qDebug() << "could't open projects json";
    return;
  }

  QByteArray allData = loadFile.readAll();
  loadFile.close();

  QJsonParseError jsonerror;
  QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

  if (jsonerror.error != QJsonParseError::NoError) {
    qDebug() << "json error!" << jsonerror.errorString();
    return;
  }

  if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError) {
    if (doc.isObject()) {
      QJsonObject object = doc.object();
      QJsonObject::iterator it = object.begin();
      // read the dimesion of the variables
      while (it != object.end()) {
        if (it.key() == "period") {
          robotdata->T = it.value().toDouble();
        }
        if (it.key() == "period_d") {
          robotdata->Td = it.value().toDouble();
        }
        if (it.key() == "period2") {
          robotdata->Td2 = it.value().toDouble();
        }
        if (it.key() == "early_terminate") {
          robotdata->Td_ratio = it.value().toDouble();
        }
        if (it.key() == "height") {
          robotdata->hd = it.value().toDouble();
        }
        if (it.key() == "foot_height") {
          robotdata->foot_zMid = it.value().toDouble();
        }
        if (it.key() == "com_x") {
          robotdata->poffset_x = it.value().toDouble();
        }
        if (it.key() == "com_y") {
          robotdata->poffset_y = it.value().toDouble();
        }
        if (it.key() == "com_z") {
          robotdata->poffset_z = it.value().toDouble();
        }
        if (it.key() == "vel_y_osci") {
          robotdata->vy_offset = it.value().toDouble();
        }
        if (it.key() == "vel_y_vari") {
          robotdata->vy_change_ratio = it.value().toDouble();
        }
        if (it.key() == "vel_x_FFratio") {
          robotdata->sigma_ratio_x = it.value().toDouble();
        }
        if (it.key() == "vel_y_FFratio") {
          robotdata->sigma_ratio_y = it.value().toDouble();
        }
        if (it.key() == "vel_x_FBratio") {
          robotdata->kp_ratio_x = it.value().toDouble();
        }
        if (it.key() == "vel_y_FBratio") {
          robotdata->kp_ratio_y = it.value().toDouble();
        }
        if (it.key() == "x_bound") {
          robotdata->foot_limit_x = it.value().toDouble();
        }
        if (it.key() == "y_bound_in") {
          robotdata->foot_limit_yin = it.value().toDouble();
        }
        if (it.key() == "y_bound_out") {
          robotdata->foot_limit_yout = it.value().toDouble();
        }
        if (it.key() == "z_bound") {
          robotdata->foot_limit_z = it.value().toDouble();
        }
        if (it.key() == "rx_factor") {
          robotdata->roll_weight = it.value().toDouble();
        }
        if (it.key() == "ry_factor") {
          robotdata->pitch_weight = it.value().toDouble();
        }
        if (it.key() == "px_factor") {
          robotdata->px_weight = it.value().toDouble();
        }
        if (it.key() == "py_factor") {
          robotdata->py_weight = it.value().toDouble();
        }
        it++;
      }
    }
  }
}
