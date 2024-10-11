#include "uniPlan.h"

uniPlan::uniPlan() {
  WaistPosVelRelateToLeftFoot_ = Eigen::VectorXd::Zero(6);
  WaistPosVelRelateToRightFoot_ = Eigen::VectorXd::Zero(6);
  WaistPosRelateToLeftFoot_ = Eigen::Vector3d::Zero(3);
  WaistPosRelateToRightFoot_ = Eigen::Vector3d::Zero(3);
  WaistVelRelateToLeftFoot_ = Eigen::Vector3d::Zero(3);
  WaistVelRelateToRightFoot_ = Eigen::Vector3d::Zero(3);
  AngularMomentumAtLeftFoot_.setZero();
  AngularMomentumAtRightFoot_.setZero();

  angular_momentun_x_eos_ = 0.0;
  angular_momentun_y_eos_ = 0.0;

  run_factor_ = 1.0;
  momentum_factor_ = 1.0;

  LeftFootPosRelativeToWaist_ = Eigen::Vector3d::Zero(3);
  RightFootPosRelativeToWaist_ = Eigen::Vector3d::Zero(3);
  LeftFootVelRelativeToWaist_ = Eigen::Vector3d::Zero(3);
  RightFootVelRelativeToWaist_ = Eigen::Vector3d::Zero(3);
  rot_matrix_ = Eigen::Matrix3d::Identity(3, 3);
}
void uniPlan::readParas(QString path, Robot_Data* robotdata) {
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
        if (it.key() == "carrybox_com_x") {
          robotdata->carrybox_poffset_x = it.value().toDouble();
        }
        if (it.key() == "carrybox_com_x") {
          robotdata->carrybox_poffset_x = it.value().toDouble();
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
        if (it.key() == "use_AM_calcFootHold") {
          robotdata->use_AM = it.value().toDouble();
          if (robotdata->use_AM == 1) {
            robotdata->use_AM_flag = true;
          } else {
            robotdata->use_AM_flag = false;
          }
        }
        if (it.key() == "l_start_offset") {
          robotdata->l_start_offset = it.value().toDouble();
        }
        if (it.key() == "r_start_offset") {
          robotdata->r_start_offset = it.value().toDouble();
        }
        if (it.key() == "forward_offset") {
          robotdata->forward_offset = it.value().toDouble();
        }
        if (it.key() == "mocap_with_hand") {
          robotdata->mocap_with_hand = it.value().toDouble();
          if (robotdata->mocap_with_hand == 1) {
            robotdata->is_mocap_with_hand = true;
          } else {
            robotdata->is_mocap_with_hand = false;
          }
        }
        if (it.key() == "momentum_factor") {
          robotdata->momentum_factor = it.value().toDouble();
        }
        if (it.key() == "use_IMU_correction") {
          double i = it.value().toDouble();
          if (i == 1) {
            robotdata->use_IMU_correction = true;
          }
          if (i == 0) {
            robotdata->use_IMU_correction = false;
          }
        }
        it++;
      }
    }
  }
}
void uniPlan::init(Eigen::VectorXd qCmd_, Eigen::VectorXd qDotCmd_, Eigen::VectorXd xStand_, Robot_Data* robotdata) {
  qCmd = qCmd_;
  qDotCmd = qDotCmd_;
  firstFlag = 1;
  vZInit = 0.0;
  vZEnd = -0.0;
  zMid = robotdata->foot_zMid;
  xStand.resize(6);
  xStand = xStand_;
  offSetL << 0.0, 0.15285, -0.06658;
  offSetR << 0.0, -0.15285, -0.06658;
  offSet << 0.0, 0.15285, -0.06658, 0.0, -0.15285, -0.06658;
  jointP << 3000., 3000., 467., 312., 582., 582.;
  jointD << 30., 30., 15., 15., 11.64, 11.64;

  hd = robotdata->hd;
  pOffset.resize(3);
  carrybox_pOffset_x = robotdata->carrybox_poffset_x;
  pOffset[0] = robotdata->poffset_x;
  pOffset[1] = robotdata->poffset_y;
  pOffset[2] = robotdata->poffset_z;  // stand has these paras too.
  vTorso_td_filt << 0.0, 0.0, 0.0;

  // lambda = std::sqrt(9.81/(hd+pOffset[2]));
  // robotdata->pTorso_begin(2) = hd;
  // robotdata->vTorso_begin(2) = 0.0;
  // robotdata->pTorso_final(2) = hd;
  // robotdata->vTorso_final(2) = 0.0;

  // lambda = std::sqrt(9.81/0.599);
  // robotdata->pTorso_begin(2) = 0.75 - pOffset[2];
  // robotdata->vTorso_begin(2) = -0.2948;
  // robotdata->pTorso_final(2) = 0.75 - pOffset[2];
  // robotdata->vTorso_final(2) = 0.2948;

  // lambda = std::sqrt(9.81/0.576);
  // robotdata->pTorso_begin(2) = 0.77 - pOffset[2];
  // robotdata->vTorso_begin(2) = -0.3926;
  // robotdata->pTorso_final(2) = 0.77 - pOffset[2];
  // robotdata->vTorso_final(2) = 0.3926;

  // lambda = std::sqrt(9.81/0.4756);
  // robotdata->pTorso_begin(2) = 0.78 - pOffset[2];
  // robotdata->vTorso_begin(2) = -0.6868;
  // robotdata->pTorso_final(2) = 0.78 - pOffset[2];
  // robotdata->vTorso_final(2) = 0.6868;

  // lambda = std::sqrt(9.81/0.4756);
  // robotdata->t0_leg << -0.38, -0.07;      //start time of every gait
  // cycle, gait circle contains stance, swing sequentially, so it's
  // the time starts swing. robotdata->offset_leg << 0.38, 0.07;
  // //offset of every gait cycle, initial t0_leg = 0 - offset_leg
  // robotdata->tst_leg << 0.24, 0.24; robotdata->tsw_leg << 0.38,
  // 0.38; robotdata->st_leg << 0.0, 1.0; robotdata->st_leg_pre <<
  // 0.0, 1.0; robotdata->T_leg = 0.62; robotdata->pTorso_begin(2) =
  // 0.78 - robotdata->poffset_z; robotdata->vTorso_begin(2) = -0.6868;
  // robotdata->pTorso_final(2) = 0.78 - robotdata->poffset_z;
  // robotdata->vTorso_final(2) = 0.6868;
  // robotdata->arm_style_d = 1.0;

  lambda = std::sqrt(9.81 / (hd + pOffset[2]));
  robotdata->t0_leg << -0.4,
      -0.0;  // start time of every gait cycle, gait circle contains stance,
             // swing sequentially, so it's the time starts swing.
  robotdata->offset_leg << 0.4,
      0.0;  // offset of every gait cycle, initial t0_leg = 0 - offset_leg
  robotdata->tst_leg << 0.4, 0.4;
  robotdata->tsw_leg << 0.4, 0.4;
  robotdata->st_leg << 0.0, 1.0;
  robotdata->st_leg_pre << 0.0, 1.0;
  robotdata->T_leg = 0.8;
  robotdata->pTorso_begin(2) = hd;
  robotdata->vTorso_begin(2) = 0.0;
  robotdata->pTorso_final(2) = hd;
  robotdata->vTorso_final(2) = 0.0;
  robotdata->arm_style_d = 0.0;
  if (robotdata->stance_index == 0) {
    robotdata->t0_leg << -0.0,
        -0.4;  // start time of every gait cycle, gait circle contains stance,
               // swing sequentially, so it's the time starts swing.
    robotdata->offset_leg << 0.0,
        0.4;  // offset of every gait cycle, initial t0_leg = 0 - offset_leg
    robotdata->st_leg << 1.0, 0.0;
    robotdata->st_leg_pre << 1.0, 0.0;
  }
  robotdata->vy_offset = 0.26;
  robotdata->foot_zMid = 0.06;
  // robotdata->rotateAngle = M_PI/22.;

  // lambda = std::sqrt(9.81/(hd+pOffset[2]));
  // robotdata->t0_leg << -0.38, -0.0;      //start time of every gait cycle,
  // gait circle contains stance, swing sequentially, so it's the time starts
  // swing. robotdata->offset_leg << 0.38, 0.0;   //offset of every gait cycle,
  // initial t0_leg = 0 - offset_leg robotdata->tst_leg << 0.38, 0.38;
  // robotdata->tsw_leg << 0.38, 0.38;
  // robotdata->st_leg << 0.0, 1.0;
  // robotdata->st_leg_pre << 0.0, 1.0;
  // robotdata->T_leg = 0.8;
  // robotdata->pTorso_begin(2) = hdw;
  // robotdata->vTorso_begin(2) = 0.0;
  // robotdata->pTorso_final(2) = hd;
  // robotdata->vTorso_final(2) = 0.0;
  // robotdata->arm_style_d = 0.0;
  // if(robotdata->stance_index==0){
  //     robotdata->t0_leg << -0.0, -0.38;      //start time of every gait
  //     cycle, gait circle contains stance, swing sequentially, so it's the
  //     time starts swing. robotdata->offset_leg << 0.0, 0.38;   //offset of
  //     every gait cycle, initial t0_leg = 0 - offset_leg robotdata->st_leg
  //     << 1.0, 0.0; robotdata->st_leg_pre << 1.0, 0.0;
  // }
  // robotdata->vy_offset = 0.38;

  robotdata->rTorso_ref.setIdentity();
  robotdata->eulerFoot_kin_ref(2) = robotdata->rotateAngle;
  robotdata->eulerFoot_kin_ref(5) = -robotdata->rotateAngle;
}
bool uniPlan::calVCmd(Robot_Data* robotdata) {
  // joystick
  // if((fabs(robotdata->vCmd_joystick(0))>0.1)){
  //     if((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))>0.0009))
  //     {
  //         robotdata->vCmd(0) +=  0.0009*(robotdata->vCmd_joystick(0) -
  //         robotdata->vCmd(0))/fabs(robotdata->vCmd_joystick(0) -
  //         robotdata->vCmd(0));
  //     }else{
  //         robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
  //     }
  // }else{
  //     if((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0))>0.0015))
  //     {
  //         robotdata->vCmd(0) +=  0.0015*(robotdata->vCmd_joystick(0) -
  //         robotdata->vCmd(0))/fabs(robotdata->vCmd_joystick(0) -
  //         robotdata->vCmd(0));
  //     }else{
  //         robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
  //     }
  // }

  if ((fabs(robotdata->vCmd_joystick(0)) > 0.1)) {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0015)) {
      robotdata->vCmd(0) += 0.0015 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
                            fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  } else {
    if ((fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) > 0.0025)) {
      robotdata->vCmd(0) += 0.0025 * (robotdata->vCmd_joystick(0) - robotdata->vCmd(0)) /
                            fabs(robotdata->vCmd_joystick(0) - robotdata->vCmd(0));
    } else {
      robotdata->vCmd(0) = robotdata->vCmd_joystick(0);
    }
  }

  // odometer mean
  //  lhj: 为什么这里的速度不用vCmd_joystick而是使用vCmd_offset_joystick？
  int N = robotdata->T_leg / robotdata->dt;
  double ite_vx = 2.0 / N;
  robotdata->odometer_avg.head(2) = ite_vx * robotdata->p_odometer + (1.0 - ite_vx) * robotdata->odometer_avg.head(2);
  robotdata->avg_vx = ite_vx * robotdata->q_dot_a(0) + (1.0 - ite_vx) * robotdata->avg_vx;
  if (robotdata->time < 0.5 * robotdata->dt) {
    robotdata->odometer_avg.head(2) = robotdata->p_odometer;
    robotdata->odometer_d.head(2) = robotdata->odometer_avg.head(2);
    robotdata->avg_vx = 0;
  }
  robotdata->odometer_d(1) += robotdata->vCmd_offset_joystick(1);
  if (fabs(robotdata->odometer_d(1) - robotdata->odometer_avg(1)) > 0.3) {
    double error = robotdata->odometer_d(1) - robotdata->odometer_avg(1);
    robotdata->odometer_d(1) = error / fabs(error) * 0.3 + robotdata->odometer_avg(1);
  }
  robotdata->vy = 0.15 * (robotdata->odometer_d(1) - robotdata->odometer_avg(1));
  // if(robotdata->standcmd==true){
  //       robotdata->vy =  0.0*(robotdata->odometer_d(1) -
  //       robotdata->odometer_avg(1));;
  // }
  robotdata->vy = std::min(0.05, std::max(robotdata->vy, -0.05));
  // robotdata->vy = robotdata->vCmd_joystick(1);//0.0;

  double vy_offset = robotdata->vy_offset;
  if (robotdata->gaitMode == 1) {
    vy_offset = robotdata->vy_offset * run_factor_;
  } else if (robotdata->gaitMode == 0) {
    vy_offset = robotdata->vy_offset;
    if (robotdata->avg_vx > 0.1) {
      vy_offset = std::max(0.1, robotdata->vy_offset - robotdata->vy_change_ratio * (fabs(robotdata->avg_vx) - 0.1));
    }
  }
  // if (robotdata->avg_vx > 0.1) {
  //   vy_offset = std::max(0.1, robotdata->vy_offset - robotdata->vy_change_ratio * (fabs(robotdata->avg_vx) - 0.1));
  // }
  // if (robotdata->avg_vx < -0.1) {
  //   vy_offset = std::max(0.1, robotdata->vy_offset -
  //   robotdata->vy_change_ratio * (fabs(robotdata->avg_vx) - 0.1));
  // }

  robotdata->vCmd(1) = vy_offset + robotdata->vy;
  robotdata->vCmd(2) = -vy_offset + robotdata->vy;
  return true;
}
bool uniPlan::footholdPlan(Robot_Data* robotdata) {
  // set initial postion, velocity and rest time for prediction
  double vx0, px0, vy0, py0, t_rest;
  for (int i = 0; i < 2; i++) {
    if (robotdata->t_leg(i) < 0.5 * robotdata->dt) {
      robotdata->v0_leg(i * 3) = robotdata->vtd_filt_leg(0);
      robotdata->v0_leg(i * 3 + 1) = robotdata->vtd_filt_leg(1);
      if (robotdata->time < 0.5 * robotdata->dt) {
        robotdata->l_leg(i * 3) = -robotdata->q_a(0);
        robotdata->l_leg(i * 3 + 1) = -robotdata->q_a(1);
      }
    }
  }

  if (robotdata->phase == 1) {
    vx0 = robotdata->q_dot_a(0) + robotdata->impactMomentum(1) * robotdata->impactFactor(1);
    px0 = robotdata->q_a(0) + pOffset[0];
    vy0 = robotdata->q_dot_a(1);
    py0 = robotdata->q_a(1) + pOffset[1];
    t_rest = robotdata->tst_leg(1) - robotdata->t_leg(1);
  }
  if (robotdata->phase == 3) {
    vx0 = robotdata->q_dot_a(0) + robotdata->impactMomentum(1) * robotdata->impactFactor(1);
    px0 = -robotdata->l_leg(0) + pOffset[0];
    vy0 = robotdata->q_dot_a(1);
    py0 = -robotdata->l_leg(1) + pOffset[1];
    t_rest = robotdata->tst_leg(0);
  }
  if (robotdata->phase == 4) {
    vx0 = robotdata->q_dot_a(0) + robotdata->impactMomentum(1) * robotdata->impactFactor(1);
    px0 = robotdata->q_a(0) + pOffset[0];
    vy0 = robotdata->q_dot_a(1);
    py0 = robotdata->q_a(1) + pOffset[1];
    t_rest = robotdata->tst_leg(0) - robotdata->t_leg(0);
  }
  if (robotdata->phase == 6) {
    vx0 = robotdata->q_dot_a(0) + robotdata->impactMomentum(1) * robotdata->impactFactor(1);
    px0 = -robotdata->l_leg(3) + pOffset[0];
    vy0 = robotdata->q_dot_a(1);
    py0 = -robotdata->l_leg(4) + pOffset[1];
    t_rest = robotdata->tst_leg(1);
  }

  // predict position, velocity at stance phase end
  double e1, e2, c1, c2;
  // lambda = std::sqrt(9.81/(hd+pOffset[2]));
  e1 = std::exp(lambda * t_rest);
  e2 = std::exp(-lambda * t_rest);

  c1 = 0.5 * (px0 + 1.0 / lambda * vx0);
  c2 = 0.5 * (px0 - 1.0 / lambda * vx0);
  robotdata->ptd_leg(0) = c1 * e1 + c2 * e2 - pOffset[0];
  robotdata->vtd_leg(0) = lambda * (c1 * e1 - c2 * e2);

  c1 = 0.5 * (py0 + 1.0 / lambda * vy0);
  c2 = 0.5 * (py0 - 1.0 / lambda * vy0);
  robotdata->ptd_leg(1) = c1 * e1 + c2 * e2 - pOffset[1];
  robotdata->vtd_leg(1) = lambda * (c1 * e1 - c2 * e2);

  double ita = 0.08;
  robotdata->vtd_filt_leg(0) = ita * robotdata->vtd_leg(0) + (1.0 - ita) * robotdata->vtd_filt_leg(0);
  robotdata->vtd_filt_leg(1) = ita * robotdata->vtd_leg(1) + (1.0 - ita) * robotdata->vtd_filt_leg(1);

  // calculate foothold location relative to waist
  double d2, sigma1, sigma2, kp_star, T_next;
  d2 = robotdata->vCmd(1) + robotdata->vCmd(2);
  if (robotdata->phase == 1 || robotdata->phase == 6) {
    T_next = robotdata->tst_leg(0);
    sigma1 = robotdata->sigma_ratio_x * lambda / std::tanh(0.5 * T_next * lambda);
    sigma2 = robotdata->sigma_ratio_y * lambda * std::tanh(0.5 * T_next * lambda);
    kp_star = 1. / lambda / std::sinh(T_next * lambda);

    robotdata->l_leg(0) = robotdata->vtd_filt_leg(0) / sigma1 +
                          robotdata->kp_ratio_x * kp_star * (robotdata->vtd_filt_leg(0) - robotdata->vCmd(0)) +
                          pOffset[0];
    if (!robotdata->use_AM_flag) {
      robotdata->l_leg(1) = (robotdata->vtd_filt_leg(1) - d2) / sigma2 -
                            robotdata->kp_ratio_y * kp_star * (robotdata->vtd_filt_leg(1) - robotdata->vCmd(1)) +
                            pOffset[1];
      robotdata->l_leg(1) += robotdata->pTorso_final(2) * std::tan(robotdata->eulerTorso_ref(0));
      // robotdata->l_leg(1) =
      //     std::min(robotdata->foot_limit_yout, std::max(robotdata->foot_limit_yin, robotdata->l_leg(1)));
    }
    robotdata->l_leg(2) = 0.0;
    robotdata->l_leg(0) = std::min(robotdata->foot_limit_x, std::max(-robotdata->foot_limit_x, robotdata->l_leg(0)));
    if (robotdata->impactMomentum(0) == 0.0 && robotdata->step != 0) {
      robotdata->l_leg(1) =
          std::min(robotdata->foot_limit_yout, std::max(robotdata->foot_limit_yin, robotdata->l_leg(1)));
    } else {
      robotdata->l_leg(1) = std::min(robotdata->foot_limit_yout * 2, std::max(0.0, robotdata->l_leg(1)));
    }
    robotdata->impactMomentum(0) = 0.0;
    robotdata->impactMomentum(1) = 0.0;
  }
  if (robotdata->phase == 3 || robotdata->phase == 4) {
    T_next = robotdata->tst_leg(1);
    sigma1 = robotdata->sigma_ratio_x * lambda / std::tanh(0.5 * T_next * lambda);
    sigma2 = robotdata->sigma_ratio_y * lambda * std::tanh(0.5 * T_next * lambda);
    kp_star = 1. / lambda / std::sinh(T_next * lambda);

    robotdata->l_leg(3) = robotdata->vtd_filt_leg(0) / sigma1 +
                          robotdata->kp_ratio_x * kp_star * (robotdata->vtd_filt_leg(0) - robotdata->vCmd(0)) +
                          pOffset[0];
    if (!robotdata->use_AM_flag) {
      robotdata->l_leg(4) = (robotdata->vtd_filt_leg(1) - d2) / sigma2 -
                            robotdata->kp_ratio_y * kp_star * (robotdata->vtd_filt_leg(1) - robotdata->vCmd(2)) +
                            pOffset[1];
      robotdata->l_leg(4) += robotdata->pTorso_final(2) * std::tan(robotdata->eulerTorso_ref(0));
      // robotdata->l_leg(4) =
      //     std::min(-robotdata->foot_limit_yin, std::max(-robotdata->foot_limit_yout, robotdata->l_leg(4)));
    }
    robotdata->l_leg(5) = 0.0;
    robotdata->l_leg(3) = std::min(robotdata->foot_limit_x, std::max(-robotdata->foot_limit_x, robotdata->l_leg(3)));
    if (robotdata->impactMomentum(0) == 0.0 && robotdata->step != 0) {
      robotdata->l_leg(4) =
          std::max(-robotdata->foot_limit_yout, std::min(-robotdata->foot_limit_yin, robotdata->l_leg(4)));
    } else {
      robotdata->l_leg(4) = std::max(-robotdata->foot_limit_yout * 2, std::min(0.0, robotdata->l_leg(4)));
    }
    robotdata->impactMomentum(0) = 0.0;
    robotdata->impactMomentum(1) = 0.0;
  }

  return true;
}

// lhj:foothold plan use AM
bool uniPlan::footholdPlan_AM(Robot_Data* robotdata) {
  m_ = robotdata->TotalMass;
  hx_ = robotdata->hd;
  hy_ = robotdata->hd;
  lx_ = std::sqrt(9.8015 / hx_);
  ly_ = std::sqrt(9.8015 / hy_);
  // remaining time to end of step(eos)
  double time_to_eos, total_swing_time, Lx_des_eos, Ly_des_eos;
  double waist_pos_in_support_foot[2] = {0.0, 0.0};
  double waist_vel_in_support_foot[2] = {0.0, 0.0};
  double support_foot_angular_mom[2] = {0.0, 0.0};
  double alpha = 0.9;

  total_swing_time = robotdata->tst_leg(0);
  if (robotdata->phase == 1) {
    time_to_eos = robotdata->tst_leg(1) - robotdata->t_leg(1);
  }
  if (robotdata->phase == 3) {
    time_to_eos = robotdata->tst_leg(0);
  }
  if (robotdata->phase == 4) {
    time_to_eos = robotdata->tst_leg(0) - robotdata->t_leg(0);
  }
  if (robotdata->phase == 6) {
    time_to_eos = robotdata->tst_leg(1);
  }

  this->CalcWaistPosVelRelativeToFoot(robotdata);
  this->CalcAngularMomentumAtFoot(robotdata);

  // Ly_des_eos = -0.05 * m_ * hx_ * lx_ * robotdata->vCmd_joystick[0];
  Ly_des_eos = m_ * hx_ * (robotdata->vCmd_joystick(0) + robotdata->vCmd_offset_joystick(0));
  if (robotdata->phase == 1 || robotdata->phase == 6) {
    waist_pos_in_support_foot[0] = WaistPosRelateToRightFoot_(0);
    waist_pos_in_support_foot[1] = WaistPosRelateToRightFoot_(1);
    waist_vel_in_support_foot[0] = WaistVelRelateToRightFoot_(0);
    waist_vel_in_support_foot[1] = WaistVelRelateToRightFoot_(1);
    support_foot_angular_mom[0] =
        -(AngularMomentumAtLeftFoot_(0) + robotdata->impactMomentum(0) * robotdata->impactFactor(0));
    support_foot_angular_mom[1] = AngularMomentumAtRightFoot_(1);

    /*************Calculate predicted forward foot hold*****************/
    // angular_momentun_y_eos_ = m_ * hx_ * lx_ * std::sinh(lx_ * time_to_eos) * waist_pos_in_support_foot[0] +
    //                           support_foot_angular_mom[1] * std::cosh(lx_ * time_to_eos);

    // Ly_des_eos =
    //     m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time) *
    //         (waist_pos_in_support_foot[0] + robotdata->vCmd_joystick(0) * robotdata->dt - robotdata->forward_offset)
    //         +
    //     angular_momentun_y_eos_ * std::cosh(lx_ * total_swing_time);

    // robotdata->l_leg(0) = -((1 - alpha) * Ly_des_eos / (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time)) +
    //                         (alpha - std::cosh(lx_ * total_swing_time)) * angular_momentun_y_eos_ /
    //                             (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time)));

    // robotdata->l_leg(0) = -(Ly_des_eos - std::cosh(lx_ * total_swing_time) * angular_momentun_y_eos_) /
    //                       (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time));

    /*************Calculate predicted lateral foot hold*****************/
    angular_momentun_x_eos_ =
        momentum_factor_ * m_ * hy_ * ly_ * std::sinh(ly_ * time_to_eos) * waist_pos_in_support_foot[1] +
        support_foot_angular_mom[0] * std::cosh(ly_ * time_to_eos);

    Lx_des_eos = 0.5 * m_ * hy_ * ly_ *
                 (std::abs(WaistPosVelRelateToRightFoot_(1) - WaistPosVelRelateToLeftFoot_(1)) +
                  robotdata->vCmd_joystick(1) * 2 + robotdata->vCmd_offset_joystick(1) - robotdata->l_start_offset) *
                 std::sinh(ly_ * total_swing_time) / (1.0 + std::cosh(ly_ * total_swing_time));

    if (robotdata->use_AM_flag) {
      robotdata->l_leg(1) = -(Lx_des_eos - std::cosh(ly_ * total_swing_time) * angular_momentun_x_eos_) /
                            (m_ * hy_ * ly_ * std::sinh(ly_ * total_swing_time));
      robotdata->l_leg(1) += robotdata->pTorso_final(2) * std::tan(robotdata->eulerTorso_ref(0));
    }
    robotdata->l_leg(2) = 0.0;
    // if (robotdata->impactFactor > 5 && robotdata->step == 0) {
    //   robotdata->l_leg(1) = std::min(robotdata->foot_limit_yout * 5, std::max(0.0, robotdata->l_leg(1)));
    // } else {
    //   robotdata->l_leg(1) =
    //       std::min(robotdata->foot_limit_yout, std::max(robotdata->foot_limit_yin, robotdata->l_leg(1)));
    // }
    if (robotdata->impactMomentum(0) == 0.0 && robotdata->step != 0) {
      robotdata->l_leg(1) =
          std::min(robotdata->foot_limit_yout, std::max(robotdata->foot_limit_yin, robotdata->l_leg(1)));
    } else {
      robotdata->l_leg(1) = std::min(robotdata->foot_limit_yout * 2, std::max(0.0, robotdata->l_leg(1)));
    }
    robotdata->impactMomentum(0) = 0.0;
  } else if (robotdata->phase == 3 || robotdata->phase == 4) {
    waist_pos_in_support_foot[0] = WaistPosRelateToLeftFoot_(0);
    waist_pos_in_support_foot[1] = WaistPosRelateToLeftFoot_(1);
    waist_vel_in_support_foot[0] = WaistVelRelateToLeftFoot_(0);
    waist_vel_in_support_foot[1] = WaistVelRelateToLeftFoot_(1);
    support_foot_angular_mom[0] =
        -(AngularMomentumAtLeftFoot_(0) + robotdata->impactMomentum(0) * robotdata->impactFactor(0));
    support_foot_angular_mom[1] = AngularMomentumAtLeftFoot_(1);

    /*************Calculate predicted forward foot hold*****************/
    // angular_momentun_y_eos_ = m_ * hx_ * lx_ * std::sinh(lx_ * time_to_eos) * waist_pos_in_support_foot[0] +
    //                           support_foot_angular_mom[1] * std::cosh(lx_ * time_to_eos);

    // Ly_des_eos =
    //     m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time) *
    //         (waist_pos_in_support_foot[0] + robotdata->vCmd_joystick(0) * robotdata->dt -
    //         robotdata->forward_offset)
    //         +
    //     angular_momentun_y_eos_ * std::cosh(lx_ * total_swing_time);

    // robotdata->l_leg(3) = -(Ly_des_eos - std::cosh(lx_ * total_swing_time) * angular_momentun_y_eos_) /
    //                       (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time));

    // robotdata->l_leg(3) = -((1 - alpha) * Ly_des_eos / (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time)) +
    //                         (alpha - std::cosh(lx_ * total_swing_time)) * angular_momentun_y_eos_ /
    //                             (m_ * hx_ * lx_ * std::sinh(lx_ * total_swing_time)));

    /*************Calculate predicted lateral foot hold*****************/
    angular_momentun_x_eos_ =
        momentum_factor_ * m_ * hy_ * ly_ * std::sinh(ly_ * time_to_eos) * waist_pos_in_support_foot[1] +
        support_foot_angular_mom[0] * std::cosh(ly_ * time_to_eos);

    Lx_des_eos = -0.5 * m_ * hy_ * ly_ *
                 (std::abs(WaistPosVelRelateToRightFoot_(1) - WaistPosVelRelateToLeftFoot_(1)) -
                  robotdata->vCmd_joystick(1) * 2 + robotdata->vCmd_offset_joystick(1) - robotdata->r_start_offset) *
                 std::sinh(ly_ * total_swing_time) / (1.0 + std::cosh(ly_ * total_swing_time));
    if (robotdata->use_AM_flag) {
      robotdata->l_leg(4) = -(Lx_des_eos - std::cosh(ly_ * total_swing_time) * angular_momentun_x_eos_) /
                            (m_ * hy_ * ly_ * std::sinh(ly_ * total_swing_time));
      robotdata->l_leg(4) += robotdata->pTorso_final(2) * std::tan(robotdata->eulerTorso_ref(0));
    }
    robotdata->l_leg(5) = 0.0;

    if (robotdata->impactMomentum(0) == 0.0 && robotdata->step != 0) {
      robotdata->l_leg(4) =
          std::max(-robotdata->foot_limit_yout, std::min(-robotdata->foot_limit_yin, robotdata->l_leg(4)));
    } else {
      robotdata->l_leg(4) = std::max(-robotdata->foot_limit_yout * 2, std::min(0.0, robotdata->l_leg(4)));
    }
    robotdata->impactMomentum(0) = 0.0;
  }
  return true;
}

bool uniPlan::footPlan(Robot_Data* robotdata) {
  robotdata->pFoot_a.head(3) = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 3).transpose();
  robotdata->pFoot_a.tail(3) = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 3).transpose();
  robotdata->vFoot_a.head(3) = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(1, 3, 1, 3).transpose();
  robotdata->vFoot_a.tail(3) = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(1, 3, 1, 3).transpose();
  robotdata->rFoot_a.head(3) = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 0, 1, 3).transpose();
  robotdata->rFoot_a.tail(3) = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 0, 1, 3).transpose();

  for (int i = 0; i < 2; i++) {
    if (robotdata->st_leg(i) == 0) {
      robotdata->pFootb_final(0 + 3 * i) = robotdata->l_leg(0 + 3 * i);
      robotdata->pFootb_final(1 + 3 * i) = robotdata->l_leg(1 + 3 * i);
      robotdata->pFootb_final(2 + 3 * i) =
          robotdata->l_leg(2 + 3 * i) - robotdata->task_card_set[robotdata->body_task_id]->X_a(0, 5);

      robotdata->vFootb_final(0 + 3 * i) = -robotdata->vtd_filt_leg(0);
      robotdata->vFootb_final(1 + 3 * i) = 0.0;
      robotdata->vFootb_final(2 + 3 * i) = 0.0;  //-robotdata->task_card_set[robotdata->body_task_id]->X_a(1,5);

      if (robotdata->time < 0.5 * robotdata->dt) {
        robotdata->pFootb_ref.segment(3 * i, 3) =
            robotdata->pFoot_a.segment(3 * i, 3) -
            robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
        robotdata->vFootb_ref.segment(3 * i, 3) =
            robotdata->vFoot_a.segment(3 * i, 3) -
            robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
      }
      if (robotdata->t_leg(i) < robotdata->tst_leg(i) + 0.5 * robotdata->dt) {
        robotdata->pFootb_ref.segment(3 * i, 3) =
            robotdata->pFoot_a.segment(3 * i, 3) -
            robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
        robotdata->vFootb_ref.segment(3 * i, 3) =
            robotdata->vFoot_a.segment(3 * i, 3) -
            robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
        robotdata->pFootb_begin.segment(3 * i, 3) = robotdata->pFootb_ref.segment(3 * i, 3);
        robotdata->vFootb_begin.segment(3 * i, 3) = robotdata->vFootb_ref.segment(3 * i, 3);
      } else if (robotdata->t_leg(i) < robotdata->tst_leg(i) + robotdata->tsw_leg(i) - 0.5 * robotdata->dt) {
        for (int j = 0; j < 2; j++) {
          Thirdpoly(robotdata->pFootb_ref(j + 3 * i), robotdata->vFootb_ref(j + 3 * i),
                    robotdata->pFootb_final(j + 3 * i), robotdata->vFootb_final(j + 3 * i),
                    robotdata->tst_leg(i) + robotdata->tsw_leg(i) - robotdata->t_leg(i), robotdata->dt,
                    robotdata->pFootb_ref(j + 3 * i), robotdata->vFootb_ref(j + 3 * i));
        }
        double xDDotTrans;
        // foot z replan
        double tMid_ = 0.5 * (robotdata->t_leg(i) - robotdata->tst_leg(i)) + 0.5 * robotdata->tsw_leg(i);
        double zMid_, vMid_;
        TriPointsQuintic(robotdata->tsw_leg(i), tMid_, 0.5 * robotdata->tsw_leg(i), robotdata->pFootb_begin(2 + 3 * i),
                         robotdata->vFootb_begin(2 + 3 * i), robotdata->foot_zMid + robotdata->pFootb_final(2 + 3 * i),
                         0., robotdata->pFootb_final(2 + 3 * i), robotdata->vFootb_final(2 + 3 * i), zMid_, vMid_,
                         xDDotTrans);
        TriPointsQuintic(robotdata->tst_leg(i) + robotdata->tsw_leg(i) - robotdata->t_leg(i), robotdata->dt,
                         robotdata->pFootb_ref(2 + 3 * i), robotdata->vFootb_ref(2 + 3 * i), zMid_, vMid_,
                         robotdata->pFootb_final(2 + 3 * i), robotdata->vFootb_final(2 + 3 * i),
                         robotdata->pFootb_ref(2 + 3 * i), robotdata->vFootb_ref(2 + 3 * i), xDDotTrans);
      }
    } else {
      robotdata->pFootb_ref.segment(3 * i, 3) =
          robotdata->pFoot_a.segment(3 * i, 3) -
          robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
      // robotdata->pFootb_ref.segment(3*i,3) = -robotdata->rTorso_ref;
      robotdata->vFootb_ref.segment(3 * i, 3) =
          robotdata->vFoot_a.segment(3 * i, 3) -
          robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
    }
  }

  // //ankle pose plan
  // for(int i=0; i<2; i++){
  //     if(robotdata->st_leg(i)==0){
  //         robotdata->rFoot_ref.block(3*i,0,3,3) =
  //         Eigen::Matrix3d::Identity();
  //     }
  //     else{
  //         Eigen::Matrix3d rFoot_a;
  //         basicfunction::Euler_XYZToMatrix(rFoot_a,
  //         robotdata->rFoot_a.segment(3*i,3));
  //         robotdata->rFoot_ref.block(3*i,0,3,3) = rFoot_a;
  //     }
  // }

  // ankle pose plan
  robotdata->rFoot_d.block(0, 0, 3, 3) = basicfunction::RotZ(robotdata->rotateAngle);  //*basicfunction::RotY(M_PI/60.);
  robotdata->rFoot_d.block(3, 0, 3, 3) =
      basicfunction::RotZ(-robotdata->rotateAngle);  //*basicfunction::RotY(M_PI/60.);

  Eigen::Matrix3d rFoot_a;
  for (int i = 0; i < 2; i++) {
    if (robotdata->st_leg(i) == 0) {
      if (robotdata->t_leg(i) < robotdata->tst_leg(i) + 2.0 * robotdata->Tc) {
        Eigen::Matrix3d r_temp;
        basicfunction::Euler_XYZToMatrix(r_temp, robotdata->rFoot_a.segment(3 * i, 3));
        robotdata->rFoot_ref.block(3 * i, 0, 3, 3) = r_temp;
        robotdata->rFoot_td.block(3 * i, 0, 3, 3) = r_temp;
      } else if (robotdata->t_leg(i) < robotdata->tst_leg(i) + 0.8 * robotdata->tsw_leg(i) - 0.5 * robotdata->dt) {
        Eigen::Vector3d omiga_d, acc_d;
        Eigen::Matrix3d rFoot;
        quaternionInterp(robotdata->rFoot_td.block(3 * i, 0, 3, 3), robotdata->rFoot_d.block(3 * i, 0, 3, 3),
                         0.8 * robotdata->tsw_leg(i) - 2.0 * robotdata->Tc,
                         robotdata->t_leg(i) - robotdata->tst_leg(i) - 2.0 * robotdata->Tc + robotdata->dt, rFoot,
                         omiga_d, acc_d);
        // quaternionInterp(robotdata->rFoot_td.block(3*i,0,3,3),
        // robotdata->rFoot_d.block(3*i,0,3,3), 0.3*robotdata->tsw_leg(i),
        // robotdata->t_leg(i) - robotdata->tst_leg(i) -
        // 0.5*robotdata->tsw_leg(i) + robotdata->dt,
        //                                 rFoot, omiga_d, acc_d);
        robotdata->rFoot_ref.block(3 * i, 0, 3, 3) = rFoot;
      }
      // if(robotdata->isAnkleTouch(i) == 1.0){
      //     basicfunction::Euler_XYZToMatrix(rFoot_a,
      //     robotdata->rFoot_a.segment(3*i,3));
      //     robotdata->rFoot_ref.block(3*i,0,3,3) = rFoot_a;
      // }
      robotdata->rFoot_kin_ref.block(3 * i, 0, 3, 3) = robotdata->rFoot_ref.block(3 * i, 0, 3, 3);
    } else {
      // basicfunction::Euler_XYZToMatrix(rFoot_a,
      // robotdata->rFoot_a.segment(3*i,3));
      // robotdata->rFoot_ref.block(3*i,0,3,3) = rFoot_a;
      if (robotdata->t_leg(i) < 0.5 * robotdata->dt) {
        robotdata->eulerFoot_kin_ref(3 * i + 2) = (1.0 - 2.0 * i) * robotdata->rotateAngle;
      }
      robotdata->eulerFoot_kin_ref(3 * i + 2) -= robotdata->dt * robotdata->vyaw;
      robotdata->rFoot_kin_ref.block(3 * i, 0, 3, 3) = basicfunction::RotZ(robotdata->eulerFoot_kin_ref(3 * i + 2));
    }
  }

  return true;
}
bool uniPlan::armPlan(Robot_Data* robotdata) {
  for (int i = 0; i < 2; i++) {
    if (robotdata->st_leg(i) == 0) {
      if (robotdata->t_leg(i) < robotdata->tst_leg(i) + 0.8 * robotdata->tsw_leg(i) - 0.5 * robotdata->dt) {
        Thirdpoly(robotdata->pArm_tgt(1 - i), robotdata->vArm_tgt(1 - i),
                  (robotdata->pFootb_final(0 + 3 * i) - 0.0) / 2.0 + 0.42, 0.0,
                  robotdata->tst_leg(i) + 0.8 * robotdata->tsw_leg(i) - robotdata->t_leg(i), robotdata->dt,
                  robotdata->pArm_tgt(1 - i), robotdata->vArm_tgt(1 - i));
      }
    } else {
      if (robotdata->t_leg(i) < 0.8 * robotdata->tst_leg(i) - 0.5 * robotdata->dt) {
        Thirdpoly(robotdata->pArm_tgt(1 - i), robotdata->vArm_tgt(1 - i), (-robotdata->ptd_leg(0) - 0.0) / 2.0 + 0.42,
                  0.0, 0.8 * robotdata->tst_leg(i) - robotdata->t_leg(i), robotdata->dt, robotdata->pArm_tgt(1 - i),
                  robotdata->vArm_tgt(1 - i));
      }
    }
  }
  robotdata->pArm_tgt(0) = std::min(std::max(robotdata->pArm_tgt(0), 0.0), 1.0);
  robotdata->pArm_tgt(1) = std::min(std::max(robotdata->pArm_tgt(1), 0.0), 1.0);
  // robotdata->pArm_tgt(0) = 0.42;
  // robotdata->pArm_tgt(1) = 0.42;
  return true;
}
bool uniPlan::bodyPlan(Robot_Data* robotdata) {
  int i;
  if (robotdata->phase == 1) {
    robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3) =
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3) -
        robotdata->pFoot_a.tail(3).transpose();
    i = 1;
  } else if (robotdata->phase == 4) {
    robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3) =
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3) -
        robotdata->pFoot_a.head(3).transpose();
    i = 0;
  }

  if (robotdata->t_leg(i) < 0.5 * robotdata->dt) {
    robotdata->pTorso_ref = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->vTorso_ref = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
    robotdata->vTorso_ref.head(2) = robotdata->v0_leg.segment(3 * i, 2);
    robotdata->pTorso_ref_kin = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 3).transpose();
    robotdata->vTorso_ref_kin = robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 3).transpose();
    robotdata->pTorso_ref(2) = robotdata->pTorso_begin(2);
    robotdata->vTorso_ref(2) = robotdata->vTorso_begin(2);
  } else if (robotdata->t_leg(i) < robotdata->tst_leg(i) - 0.5 * robotdata->dt) {
    robotdata->pTorso_ref.head(2) =
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 2).transpose();
    robotdata->vTorso_ref.head(2) =
        robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 2).transpose();
    double e1, e2, c1, c2;
    e1 = std::exp(lambda * robotdata->t_leg(i));
    e2 = std::exp(-lambda * robotdata->t_leg(i));

    c1 = 0.5 * (-robotdata->l_leg(3 * i) + pOffset[0] + 1.0 / lambda * robotdata->v0_leg(3 * i));
    c2 = 0.5 * (-robotdata->l_leg(3 * i) + pOffset[0] - 1.0 / lambda * robotdata->v0_leg(3 * i));
    robotdata->pTorso_ref(0) = c1 * e1 + c2 * e2 - pOffset[0];
    robotdata->vTorso_ref(0) = lambda * (c1 * e1 - c2 * e2);
    c1 = 0.5 * (-robotdata->l_leg(3 * i + 1) + pOffset[1] + 1.0 / lambda * robotdata->v0_leg(3 * i + 1));
    c2 = 0.5 * (-robotdata->l_leg(3 * i + 1) + pOffset[1] - 1.0 / lambda * robotdata->v0_leg(3 * i + 1));
    robotdata->pTorso_ref(1) = c1 * e1 + c2 * e2 - pOffset[1];
    robotdata->vTorso_ref(1) = lambda * (c1 * e1 - c2 * e2);

    double t_rest = 0.5 * robotdata->t_leg(i) + 0.5 * robotdata->tst_leg(i);  // - robotdata->t_leg(i);
    // lambda = std::sqrt(9.81/(hd+pOffset[2]));
    e1 = std::exp(lambda * t_rest);
    e2 = std::exp(-lambda * t_rest);

    c1 = 0.5 * (robotdata->pTorso_begin(2) + pOffset[2] - 9.81 / (lambda * lambda) +
                1.0 / lambda * robotdata->vTorso_begin(2));
    c2 = 0.5 * (robotdata->pTorso_begin(2) + pOffset[2] - 9.81 / (lambda * lambda) -
                1.0 / lambda * robotdata->vTorso_begin(2));
    double zMid = c1 * e1 + c2 * e2 - pOffset[2] + 9.81 / (lambda * lambda);
    double vMid = lambda * (c1 * e1 - c2 * e2);
    double xDDotTrans;
    TriPointsQuintic(robotdata->tst_leg(i) - robotdata->t_leg(i), robotdata->dt, robotdata->pTorso_ref(2),
                     robotdata->vTorso_ref(2), zMid, vMid, robotdata->pTorso_final(2), robotdata->vTorso_final(2),
                     robotdata->pTorso_ref(2), robotdata->vTorso_ref(2), xDDotTrans);
  }
  robotdata->pTorso_ref_kin.head(2) =
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(0, 3, 1, 2).transpose();
  robotdata->vTorso_ref_kin.head(2) =
      robotdata->task_card_set[robotdata->body_task_id]->X_a.block(1, 3, 1, 2).transpose();
  robotdata->pTorso_ref_kin(2) = robotdata->pTorso_ref(2);
  robotdata->vTorso_ref_kin(2) = robotdata->vTorso_ref(2);
  // if(robotdata->t_leg(i) < robotdata->tst_leg(i) - 0.5*robotdata->dt){
  //     double e1, e2, c1, c2;
  //     // lambda = std::sqrt(9.81/(hd+pOffset[2]));
  //     e1 = std::exp(lambda*robotdata->t_leg(i));
  //     e2 = std::exp(-lambda*robotdata->t_leg(i));

  //     c1 = 0.5*(robotdata->pTorso_begin(2) + pOffset[2]
  //     - 9.81/(lambda*lambda) + 1.0/lambda*robotdata->vTorso_begin(2)); c2 =
  //     0.5*(robotdata->pTorso_begin(2) + pOffset[2] - 9.81/(lambda*lambda)
  //     - 1.0/lambda*robotdata->vTorso_begin(2)); double zMid = c1*e1 + c2*e2 -
  //     pOffset[2] + 9.81/(lambda*lambda); double vMid = lambda*(c1*e1 -
  //     c2*e2); robotdata->pTorso_ref(2) = zMid; robotdata->vTorso_ref(2) =
  //     vMid;
  // }
  robotdata->rTorso = rotX(robotdata->q_a(3)) * rotY(robotdata->q_a(4)) * rotZ(0.0);
  // robotdata->rTorso_ref = Eigen::Matrix3d::Identity();
  robotdata->coriolis_force = robotdata->q_dot_a(0) * robotdata->q_dot_a(5);
  robotdata->coriolis_force_filt = 0.01 * robotdata->coriolis_force + 0.99 * robotdata->coriolis_force_filt;
  robotdata->eulerTorso_ref(0) = std::atan(-robotdata->coriolis_force_filt / 9.81);
  // robotdata->eulerTorso_d(0) = -0.12*robotdata->q_dot_a(0)*robotdata->vyaw;
  // robotdata->eulerTorso_ref = 0.005*robotdata->eulerTorso_d +
  // 0.995*robotdata->eulerTorso_ref;
  robotdata->rTorso_ref =
      rotX(robotdata->eulerTorso_ref(0)) * rotY(robotdata->eulerTorso_ref(1)) * rotZ(robotdata->eulerTorso_ref(2));
  // robotdata->pTorso_ref(2) = robotdata->pTorso_ref(2) *
  // std::cos(robotdata->eulerTorso_ref(0));
  return true;
}
bool uniPlan::kinePlan(Robot_Data* robotdata) {
  Eigen::Vector3d Pos;
  Eigen::VectorXd qIK = Eigen::VectorXd::Zero(6), qIK_dot = Eigen::VectorXd::Zero(6),
                  xIK_dot = Eigen::VectorXd::Zero(6);

  for (int i = 0; i < 2; i++) {
    if (robotdata->t_leg(i) < 0.5 * robotdata->dt) {
      qCmd_pre = qCmd;
      qDotCmd_pre = qDotCmd;
    }
    if (robotdata->st_leg(i) == 0 && robotdata->t_leg(i) < robotdata->tst_leg(i) + 0.5 * robotdata->dt) {
      qCmd_pre = qCmd;
      qDotCmd_pre = qDotCmd;
    }
    if (robotdata->st_leg(i) == 0) {
      Pos = robotdata->rTorso.transpose() * (robotdata->pFootb_ref.segment(3 * i, 3)) - offSet.segment(3 * i, 3);
      oneLegIK_new(i * 1.0, Pos, robotdata->rTorso.transpose() * robotdata->rFoot_ref.block(3 * i, 0, 3, 3), qIK);
      qCmd.segment(6 * i, 6) = qIK;
    } else {
      // qCmd.segment(6*i,6) = robotdata->q_a.segment(6+6*i,6);
      Pos = robotdata->rTorso_ref.transpose() * (-robotdata->pTorso_ref_kin) - offSet.segment(3 * i, 3);
      // oneLegIK2(Pos,
      // robotdata->rTorso_ref.transpose()*robotdata->rFoot_ref.block(3*i,0,3,3),
      // qIK); oneLegIK2(Pos, robotdata->rTorso_ref.transpose() *
      // robotdata->rFoot_kin_ref.block(3 * i, 0, 3, 3), qIK);
      oneLegIK_new(i * 1.0, Pos, robotdata->rTorso_ref.transpose() * robotdata->rFoot_kin_ref.block(3 * i, 0, 3, 3),
                   qIK);
      qCmd.segment(6 * i, 6) = qIK;
    }
  }

  Eigen::VectorXd qCmd_full = Eigen::VectorXd::Zero(18);
  qCmd_full.tail(12) = qCmd;
  Eigen::MatrixXd J_6D;
  J_6D.setZero(6, robotdata->ndof);
  for (int i = 0; i < 2; i++) {
    if (robotdata->st_leg(i) == 0) {
      qCmd_full.segment(3, 3) = matrixtoeulerxyz_(robotdata->rTorso);
      if (i == 1) {
        RigidBodyDynamics::CalcPointJacobian6D(
            *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->right_foot_id]->joint_id,
            robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
      } else {
        RigidBodyDynamics::CalcPointJacobian6D(
            *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->left_foot_id]->joint_id,
            robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
      }
      xIK_dot.setZero();
      xIK_dot.tail(3) = robotdata->vFootb_ref.segment(3 * i, 3);
      qIK_dot = J_6D.block(0, 6 * i + 6, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIK_dot);
      qDotCmd.segment(6 * i, 6) = qIK_dot;
    } else {
      // qDotCmd.segment(6*i,6) = robotdata->q_dot_a.segment(6+6*i,6);
      qCmd_full.segment(3, 3) = matrixtoeulerxyz_(robotdata->rTorso_ref);
      if (i == 1) {
        RigidBodyDynamics::CalcPointJacobian6D(
            *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->right_foot_id]->joint_id,
            robotdata->task_card_set[robotdata->right_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
      } else {
        RigidBodyDynamics::CalcPointJacobian6D(
            *(robotdata->robot_model), qCmd_full, robotdata->task_card_set[robotdata->left_foot_id]->joint_id,
            robotdata->task_card_set[robotdata->left_foot_id]->T_offset.block(0, 3, 3, 1), J_6D, false);
      }
      xIK_dot.setZero();
      xIK_dot.tail(3) = robotdata->vFootb_ref.segment(3 * i, 3);
      xIK_dot(2) = -robotdata->vyaw;
      qIK_dot = J_6D.block(0, 6 * i + 6, 6, 6).completeOrthogonalDecomposition().pseudoInverse() * (xIK_dot);
      qDotCmd.segment(6 * i, 6) = qIK_dot;
    }
  }
  return true;
}
bool uniPlan::wbcUpdate(Robot_Data* robotdata) {
  // Weights
  Eigen::VectorXd weight_FootForce = Eigen::VectorXd::Zero(12);
  weight_FootForce << 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001;
  for (int i = 0; i < 12; ++i) {
    robotdata->WF1(i, i) = weight_FootForce(i);
  }

  robotdata->WF2 = 0.001 * Eigen::MatrixXd::Identity(12, 12);
  robotdata->WF2(5, 5) = 0.0001;
  robotdata->WF2(11, 11) = 0.0001;

  // robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
  // 100., 0., 0., 100.;
  double weight_position = 1.0 - (1.0 - robotdata->st_leg(0)) * (1.0 - robotdata->st_leg(1));
  robotdata->task_card_set[robotdata->body_task_id]->weight << robotdata->roll_weight, robotdata->pitch_weight, 100.,
      robotdata->px_weight, robotdata->py_weight, 100.;
  robotdata->task_card_set[robotdata->body_task_id]->weight =
      weight_position * robotdata->task_card_set[robotdata->body_task_id]->weight;
  robotdata->task_card_set[robotdata->left_foot_id]->weight =
      (robotdata->s_leg(0) * 1000. + (1.0 - robotdata->s_leg(0)) * 100.) * Eigen::VectorXd::Ones(6);
  robotdata->task_card_set[robotdata->right_foot_id]->weight =
      (robotdata->s_leg(1) * 1000. + (1.0 - robotdata->s_leg(1)) * 100.) * Eigen::VectorXd::Ones(6);

  // Bounds
  robotdata->tau_lb = Eigen::VectorXd::Zero(actor_num);
  robotdata->tau_ub = Eigen::VectorXd::Zero(actor_num);
  if (adam_type == ADAM_TYPE::AdamLite) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5,
        18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0;
  } else if (adam_type == ADAM_TYPE::AdamStandard) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5,
        18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0, 18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 2.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus23) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5,
        18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus29) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5,
        18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus53) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5,
        18.0, 18.0, 18.0, 18.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 18.0, 18.0,
        18.0, 18.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  } else if (adam_type == ADAM_TYPE::AdamLiteSimple) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0;
    robotdata->tau_lb = -robotdata->tau_ub;
  } else if (adam_type == ADAM_TYPE::DuckDuck) {
    robotdata->tau_ub << 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 180.0, 150.0, 90.0, 180.0, 60.0, 50.0, 82.5, 82.5, 82.5;
    robotdata->tau_lb = -robotdata->tau_ub;
  }

  double my_inf = 1000.0;
  robotdata->GRF_ub << my_inf, my_inf, my_inf, my_inf, my_inf, robotdata->s_leg(0) * 3.0 * robotdata->MG, my_inf,
      my_inf, my_inf, my_inf, my_inf, robotdata->s_leg(1) * 3.0 * robotdata->MG;
  robotdata->GRF_lb = -robotdata->GRF_ub;
  robotdata->GRF_lb(5) = 0.0;
  robotdata->GRF_lb(11) = 0.0;

  // ankle control
  double akRoll_tor = 0.0;
  double akPitch_tor = 0.0;
  if (robotdata->phase == 4) {
    akPitch_tor = 24.0 * (robotdata->vCmd[0] - robotdata->vtd_filt_leg(0));
    akRoll_tor = -18.0 * (robotdata->vCmd[2] - robotdata->vtd_filt_leg(1));
  } else if (robotdata->phase == 1) {
    akPitch_tor = 24.0 * (robotdata->vCmd[0] - robotdata->vtd_filt_leg(0));
    akRoll_tor = -18.0 * (robotdata->vCmd[1] - robotdata->vtd_filt_leg(1));
  } else {
    akPitch_tor = 0.0;
    akRoll_tor = 0.0;
  }
  // if (robotdata->phase == 4 || robotdata->phase == 1) {
  //   akPitch_tor = 50.0 * (robotdata->vTorso_ref(0) - robotdata->q_dot_a(0));
  //   // akRoll_tor = -300.0 * (robotdata->vTorso_ref(1) -
  //   robotdata->q_dot_a(1)); akRoll_tor = -300.0*(robotdata->pTorso_ref(1) -
  //   robotdata->q_a(1)) - 50.0 * (robotdata->vTorso_ref(1) -
  //   robotdata->q_dot_a(1));
  // } else {
  //   akPitch_tor = 0.0;
  //   akRoll_tor = 0.0;
  // }
  double deltx_f = 0.115;
  double deltx_b = 0.015;
  double delty = 0.015;
  double force_z = 0.8 * robotdata->contactforce(6 * robotdata->stance_index + 5);
  akPitch_tor = std::min(force_z * deltx_b, std::max(-force_z * deltx_f, akPitch_tor));
  akRoll_tor = std::min(force_z * delty, std::max(-force_z * delty, akRoll_tor));

  if (robotdata->phase == 4) {
    robotdata->GRF_ub(0) = robotdata->s_leg(0) * akRoll_tor;
    robotdata->GRF_ub(1) = robotdata->s_leg(0) * akPitch_tor;
    robotdata->GRF_lb(0) = robotdata->s_leg(0) * akRoll_tor;
    robotdata->GRF_lb(1) = robotdata->s_leg(0) * akPitch_tor;
    robotdata->GRF_ub(6) = 0.0;
    robotdata->GRF_ub(7) = 0.0;
    robotdata->GRF_lb(6) = 0.0;
    robotdata->GRF_lb(7) = 0.0;
  } else if (robotdata->phase == 1) {
    robotdata->GRF_ub(6) = robotdata->s_leg(1) * akRoll_tor;
    robotdata->GRF_ub(7) = robotdata->s_leg(1) * akPitch_tor;
    robotdata->GRF_lb(6) = robotdata->s_leg(1) * akRoll_tor;
    robotdata->GRF_lb(7) = robotdata->s_leg(1) * akPitch_tor;
    robotdata->GRF_ub(0) = 0.0;
    robotdata->GRF_ub(1) = 0.0;
    robotdata->GRF_lb(0) = 0.0;
    robotdata->GRF_lb(1) = 0.0;
  }

  // PD factor
  double ratio = 0.9;

  robotdata->q_factor << (1.0 - ratio * robotdata->s_leg(0)) * Eigen::VectorXd::Ones(6),
      (1.0 - ratio * robotdata->s_leg(1)) * Eigen::VectorXd::Ones(6);
  robotdata->q_dot_factor << (1.0 - ratio * robotdata->s_leg(0)) * Eigen::VectorXd::Ones(6),
      (1.0 - ratio * robotdata->s_leg(1)) * Eigen::VectorXd::Ones(6);

  robotdata->q_factor.segment(4, 2) << (1.0 - robotdata->s_leg(0)), (1.0 - robotdata->s_leg(0));
  robotdata->q_dot_factor.segment(4, 2) << (1.0 - robotdata->s_leg(0)), (1.0 - robotdata->s_leg(0));
  robotdata->q_factor.segment(10, 2) << (1.0 - robotdata->s_leg(1)), (1.0 - robotdata->s_leg(1));
  robotdata->q_dot_factor.segment(10, 2) << (1.0 - robotdata->s_leg(1)), (1.0 - robotdata->s_leg(1));

  if (robotdata->step < 1) {
    if (robotdata->phase == 1) {
      robotdata->q_factor << (1.0 - ratio * robotdata->s_leg(0)) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio) * Eigen::VectorXd::Ones(6);
      robotdata->q_dot_factor << (1.0 - ratio * robotdata->s_leg(0)) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio) * Eigen::VectorXd::Ones(6);
      robotdata->q_factor.segment(10, 2) << 0.0, 0.0;
      robotdata->q_dot_factor.segment(10, 2) << 0.0, 0.0;
      robotdata->q_factor.segment(4, 2) << (1.0 - ratio * robotdata->s_leg(0)), (1.0 - ratio * robotdata->s_leg(0));
      robotdata->q_dot_factor.segment(4, 2) << (1.0 - ratio * robotdata->s_leg(0)), (1.0 - ratio * robotdata->s_leg(0));
    } else if (robotdata->phase == 4) {
      robotdata->q_factor << (1.0 - ratio) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio * robotdata->s_leg(1)) * Eigen::VectorXd::Ones(6);
      robotdata->q_dot_factor << (1.0 - ratio) * Eigen::VectorXd::Ones(6),
          (1.0 - ratio * robotdata->s_leg(1)) * Eigen::VectorXd::Ones(6);
      robotdata->q_factor.segment(4, 2) << 0.0, 0.0;
      robotdata->q_dot_factor.segment(4, 2) << 0.0, 0.0;
      robotdata->q_factor.segment(10, 2) << (1.0 - ratio * robotdata->s_leg(1)), (1.0 - ratio * robotdata->s_leg(1));
      robotdata->q_dot_factor.segment(10, 2) << (1.0 - ratio * robotdata->s_leg(1)),
          (1.0 - ratio * robotdata->s_leg(1));
    }
  }

  robotdata->q_factor(2) = 1.0;
  robotdata->q_dot_factor(2) = 1.0;
  robotdata->q_factor(8) = 1.0;
  robotdata->q_dot_factor(8) = 1.0;
  return true;
}
bool uniPlan::locoPlan(Robot_Data* robotdata, Eigen::VectorXd& qCmd_, Eigen::VectorXd& qDotCmd_) {
  if (robotdata->carryBoxState == 1) {
    if (robotdata->step_calibration_flag2) {
      std::cout << "robotdata->step_calibration_x_offset:" << robotdata->step_calibration_x_offset << "\t\t"
                << "carrybox_pOffset_x:" << carrybox_pOffset_x << std::endl;

      carrybox_pOffset_x = carrybox_pOffset_x + robotdata->step_calibration_x_offset * 0.06 + 0.0001;
      robotdata->carrybox_poffset_x = carrybox_pOffset_x;
      robotdata->step_calibration_flag2 = false;
    } else {
      pOffset[0] = robotdata->carrybox_poffset_x;
    }
  } else {
    if (robotdata->step_calibration_flag2) {
      std::cout << "robotdata->step_calibration_x_offset:" << robotdata->step_calibration_x_offset << "\t\t"
                << "pOffset[0]:" << pOffset[0] << std::endl;

      pOffset[0] = pOffset[0] + robotdata->step_calibration_x_offset * 0.06 + 0.0001;
      robotdata->poffset_x = pOffset[0];
      robotdata->step_calibration_flag2 = false;
    } else {
      pOffset[0] = robotdata->poffset_x;
    }
  }

  calVCmd(robotdata);
  footholdPlan(robotdata);
  footholdPlan_AM(robotdata);
  footPlan(robotdata);
  bodyPlan(robotdata);
  armPlan(robotdata);
  kinePlan(robotdata);
  wbcUpdate(robotdata);
  // armPlan(robotdata);

  qCmd_ = qCmd;
  qDotCmd_ = qDotCmd;
  double sc;
  for (int i = 0; i < 2; i++) {
    if (robotdata->t_leg(i) < robotdata->Tc_leg - 0.5 * robotdata->dt) {
      sc = robotdata->t_leg(i) / robotdata->Tc_leg;
      qCmd_.segment(6 * i, 6) = sc * qCmd.segment(6 * i, 6) + (1.0 - sc) * qCmd_pre.segment(6 * i, 6);
      qDotCmd_.segment(6 * i, 6) = sc * qDotCmd.segment(6 * i, 6) + (1.0 - sc) * qDotCmd_pre.segment(6 * i, 6);
    }
    if (robotdata->st_leg(i) == 0 &&
        robotdata->t_leg(i) < robotdata->tst_leg(i) + robotdata->Tc_leg - 0.5 * robotdata->dt) {
      sc = (robotdata->t_leg(i) - robotdata->tst_leg(i)) / robotdata->Tc_leg;
      qCmd_.segment(6 * i, 6) = sc * qCmd.segment(6 * i, 6) + (1.0 - sc) * qCmd_pre.segment(6 * i, 6);
      qDotCmd_.segment(6 * i, 6) = sc * qDotCmd.segment(6 * i, 6) + (1.0 - sc) * qDotCmd_pre.segment(6 * i, 6);
    }
  }

  if (robotdata->gaitMode == 1 && robotdata->vCmd_joystick(0) > 0.0) {
    run_factor_ = 1.0 + robotdata->vCmd_joystick(0) * 0.;
  } else {
    run_factor_ = 1.0;
  }

  if (robotdata->gaitMode == 1 && robotdata->onRun == 0) {
    momentum_factor_ = robotdata->momentum_factor;
    for (int i = 0; i < 2; i++) {
      if (robotdata->t_leg(i) < 0.5 * robotdata->dt && robotdata->onRun == 0) {
        double si = robotdata->t_leg(i) / robotdata->tst_leg(i);

        lambda = std::sqrt(9.81 / 0.53);
        robotdata->t0_leg << -0.32,
            -0.00;  // start time of every gait cycle, gait circle contains
                    // stance, swing sequentially, so it's the time starts swing.
        robotdata->offset_leg << 0.32,
            0.00;  // offset of every gait cycle, initial t0_leg = 0 - offset_leg
        robotdata->tst_leg << 0.24, 0.24;
        robotdata->tsw_leg << 0.4, 0.4;
        robotdata->T_leg = 0.64;
        robotdata->pTorso_begin(2) = 0.854 - robotdata->poffset_z;
        robotdata->vTorso_begin(2) = -0.6619;
        robotdata->pTorso_final(2) = 0.854 - robotdata->poffset_z;
        robotdata->vTorso_final(2) = 0.6619 * 0.8;
        robotdata->arm_style_d = 1.0;
        robotdata->vy_offset = 0.26;
        robotdata->foot_zMid = 0.08;
        // robotdata->rotateAngle = 0.0;

        robotdata->t_leg(i) = robotdata->tst_leg(i) * si;
        robotdata->t0_leg(i) = robotdata->time - robotdata->t_leg(i);

        double dt = robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) > 0
                        ? robotdata->offset_leg(1 - i) - robotdata->offset_leg(i)
                        : robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) + robotdata->tst_leg(i) +
                              robotdata->tsw_leg(i);
        robotdata->t0_leg(1 - i) = robotdata->t0_leg(i) - dt;
        robotdata->t_leg(1 - i) = robotdata->time - robotdata->t0_leg(i) + dt;
        robotdata->onRun = true;
        robotdata->onWalk = false;
      }
    }
  } else if (robotdata->gaitMode == 0 && robotdata->onWalk == 0) {
    momentum_factor_ = 1.0;
    run_factor_ = 1.0;
    for (int i = 0; i < 2; i++) {
      if (robotdata->t_leg(i) < 0.5 * robotdata->tst_leg(i) + 0.5 * robotdata->dt &&
          robotdata->t_leg(i) > 0.5 * robotdata->tst_leg(i) - 0.5 * robotdata->dt && robotdata->onWalk == 0) {
        double si = robotdata->t_leg(i) / robotdata->tst_leg(i);

        lambda = std::sqrt(9.81 / (hd + pOffset[2]));
        robotdata->t0_leg << -0.4,
            -0.0;  // start time of every gait cycle, gait circle contains
                   // stance, swing sequentially, so it's the time starts swing.
        robotdata->offset_leg << 0.4,
            0.0;  // offset of every gait cycle, initial t0_leg = 0 - offset_leg
        robotdata->tst_leg << 0.4, 0.4;
        robotdata->tsw_leg << 0.4, 0.4;
        robotdata->T_leg = 0.8;
        robotdata->pTorso_begin(2) = hd;
        // lhj: change the body z, from est to hd
        // robotdata->pTorso_begin(2) = robotdata->pTorso_ref(2);
        robotdata->vTorso_begin(2) = 0.0;
        robotdata->pTorso_final(2) = hd;
        robotdata->vTorso_final(2) = 0.0;
        robotdata->arm_style_d = 0.0;
        robotdata->vy_offset = 0.26;
        robotdata->foot_zMid = 0.06;
        // robotdata->rotateAngle = M_PI/22.;

        robotdata->t_leg(i) = robotdata->tst_leg(i) * si;
        robotdata->t0_leg(i) = robotdata->time - robotdata->t_leg(i);

        double dt = robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) > 0
                        ? robotdata->offset_leg(1 - i) - robotdata->offset_leg(i)
                        : robotdata->offset_leg(1 - i) - robotdata->offset_leg(i) + robotdata->tst_leg(i) +
                              robotdata->tsw_leg(i);
        robotdata->t0_leg(1 - i) = robotdata->t0_leg(i) - dt;
        robotdata->t_leg(1 - i) = robotdata->time - robotdata->t0_leg(i) + dt;
        robotdata->onRun = false;
        robotdata->onWalk = true;
      }
    }
  } else {
  }

  return true;
}

void uniPlan::CalcWaistPosVelRelativeToFoot(Robot_Data* robotdata) {
  basicfunction::Euler_ZYXToMatrix(rot_matrix_, robotdata->imu9D.head(3));

  // std::cout << "pFootb_ref" << robotdata->pFootb_ref.transpose() << std::endl;
  // std::cout << "vFootb_ref" << robotdata->vFootb_ref.transpose() << std::endl;
  // std::cout << "pBodyf_a" << robotdata->pBodyf_a.transpose() << std::endl;
  // std::cout << "vFootb_a" << robotdata->vFootb_a.transpose() << std::endl;

  // left foot
  LeftFootPosRelativeToWaist_ = robotdata->pFootb_a.head(3);
  LeftFootVelRelativeToWaist_ = robotdata->vFootb_a.head(3);
  // WaistPosRelateToLeftFoot_ = -rot_matrix_ * LeftFootPosRelativeToWaist_;
  // WaistVelRelateToLeftFoot_ =
  //     -rot_matrix_ *
  //     (basicfunction::skew(LeftFootPosRelativeToWaist_) * robotdata->imu9D.segment(3, 3) +
  //     LeftFootVelRelativeToWaist_);

  WaistPosRelateToLeftFoot_ = robotdata->pBodyf_a.head(3);
  WaistVelRelateToLeftFoot_ = robotdata->vBodyf_a.head(3);
  // WaistPosVelRelateToLeftFoot_ = WaistPosRelateToLeftFoot_, WaistVelRelateToLeftFoot_;

  // right foot
  RightFootPosRelativeToWaist_ = robotdata->pFootb_a.tail(3);
  RightFootVelRelativeToWaist_ = robotdata->vFootb_a.tail(3);
  // WaistPosRelateToRightFoot_ = -rot_matrix_ * RightFootPosRelativeToWaist_;
  // WaistVelRelateToRightFoot_ =
  //     -rot_matrix_ * (basicfunction::skew(RightFootPosRelativeToWaist_) * robotdata->imu9D.segment(3, 3) +
  //                     RightFootVelRelativeToWaist_);

  WaistPosRelateToRightFoot_ = robotdata->pBodyf_a.tail(3);
  WaistVelRelateToRightFoot_ = robotdata->vBodyf_a.tail(3);
  // WaistPosVelRelateToRightFoot_ = WaistPosRelateToRightFoot_, WaistVelRelateToRightFoot_;
  // std::cout << "WaistPosRelateToRightFoot_: " << WaistPosRelateToRightFoot_ << std::endl;

  // robotdata->observeData.head(3) = WaistPosRelateToLeftFoot_;
  // robotdata->observeData.segment(3, 3) = WaistPosRelateToRightFoot_;
  // robotdata->observeData.segment(6, 6) = robotdata->pBodyf_a.head(6);
}

void uniPlan::CalcAngularMomentumAtFoot(Robot_Data* robotdata) {
  Eigen::Vector3d ComLinearVel = robotdata->q_dot_a.head(3);
  AngularMomentumAtLeftFoot_ =
      robotdata->comMomentum.head(3) + robotdata->TotalMass * WaistPosRelateToLeftFoot_.cross(ComLinearVel);
  AngularMomentumAtRightFoot_ =
      robotdata->comMomentum.head(3) + robotdata->TotalMass * WaistPosRelateToRightFoot_.cross(ComLinearVel);

  // robotdata->observeData.segment(6, 6) = AngularMomentumAtLeftFoot_, AngularMomentumAtRightFoot_;
}
