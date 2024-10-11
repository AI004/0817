#include "RobotInterfaceImpl.h"

RobotInterfaceImpl::RobotInterfaceImpl() {
#ifdef STATE_ESTIMATE
  stateEstimate_ = new StateEstimate;
#endif
  error_state_ = false;
#ifndef WEBOTS
  funS2P = new functionIKID_S2P;
  jointInterface_ = new JointInterface;
#endif
}

void RobotInterfaceImpl::Init() {
  int i;
#ifdef WEBOTS
  // webots init
  humanoid.initWebots();
#else

  // joint name
  i = 0;
  joint_name_.resize(joint_num_);
  joint_name_[i] = "hipPitch_Left";
  i++;
  joint_name_[i] = "hipRoll_Left";
  i++;
  joint_name_[i] = "hipYaw_Left";
  i++;
  joint_name_[i] = "kneePitch_Left";
  i++;
  joint_name_[i] = "anklePitch_Left";
  i++;
  joint_name_[i] = "ankleRoll_Left";
  i++;

  joint_name_[i] = "hipPitch_Right";
  i++;
  joint_name_[i] = "hipRoll_Right";
  i++;
  joint_name_[i] = "hipYaw_Right";
  i++;
  joint_name_[i] = "kneePitch_Right";
  i++;
  joint_name_[i] = "anklePitch_Right";
  i++;
  joint_name_[i] = "ankleRoll_Right";
  i++;

  joint_name_[i] = "waistRoll";
  i++;
  joint_name_[i] = "waistPitch";
  i++;
  joint_name_[i] = "waistYaw";
  i++;

  joint_name_[i] = "shoulderPitch_Left";
  i++;
  joint_name_[i] = "shoulderRoll_Left";
  i++;
  joint_name_[i] = "shoulderYaw_Left";
  i++;
  joint_name_[i] = "elbow_Left";
  i++;

  if (adam_type == ADAM_TYPE::AdamLite) {
  } else if (adam_type == ADAM_TYPE::AdamStandard) {
    joint_name_[i] = "wristYaw_Left";
    i++;
    joint_name_[i] = "wristPitch_Left";
    i++;
    joint_name_[i] = "wristRoll_Left";
    i++;
  } else if (adam_type == ADAM_TYPE::StandardPlus23) {
  } else if (adam_type == ADAM_TYPE::StandardPlus29) {
    joint_name_[i] = "wristYaw_Left";
    i++;
    joint_name_[i] = "wristPitch_Left";
    i++;
    joint_name_[i] = "wristRoll_Left";
    i++;
  } else if (adam_type == ADAM_TYPE::StandardPlus53) {
  }

  joint_name_[i] = "shoulderPitch_Right";
  i++;
  joint_name_[i] = "shoulderRoll_Right";
  i++;
  joint_name_[i] = "shoulderYaw_Right";
  i++;
  joint_name_[i] = "elbow_Right";
  i++;

  if (adam_type == ADAM_TYPE::AdamLite) {
  } else if (adam_type == ADAM_TYPE::AdamStandard) {
    joint_name_[i] = "wristYaw_Right";
    i++;
    joint_name_[i] = "wristPitch_Right";
    i++;
    joint_name_[i] = "wristRoll_Right";
    i++;
  } else if (adam_type == ADAM_TYPE::StandardPlus23) {
  } else if (adam_type == ADAM_TYPE::StandardPlus29) {
    joint_name_[i] = "wristYaw_Right";
    i++;
    joint_name_[i] = "wristPitch_Right";
    i++;
    joint_name_[i] = "wristRoll_Right";
    i++;
  } else if (adam_type == ADAM_TYPE::StandardPlus53) {
  }
  // if(adam_type==ADAM_TYPE::AdamLite){
  // }else if(adam_type==ADAM_TYPE::AdamStandard){
  // }else if(adam_type==ADAM_TYPE::StandardPlus23){
  // }else if(adam_type==ADAM_TYPE::StandardPlus29){
  // }else if(adam_type==ADAM_TYPE::StandardPlus53){
  // }
  absolute_pos_init_ = Eigen::VectorXd::Zero(joint_num_);
  absolute_pos_zero_ = Eigen::VectorXd::Zero(joint_num_);
  absolute_pos_dir_ = Eigen::VectorXd::Zero(joint_num_);
  absolute_pos_gear_ratio_ = Eigen::VectorXd::Zero(joint_num_);
  motor_rotor_abs_pos_ = Eigen::VectorXd::Zero(joint_num_);
  motor_rotor_cur_pos_ = Eigen::VectorXd::Zero(joint_num_);
  joint_Kp_ = Eigen::VectorXd::Zero(joint_num_);
  joint_Kd_ = Eigen::VectorXd::Zero(joint_num_);

  ReadAbsEncoder(absolute_pos_init_, motor_rotor_cur_pos_);

  // read joint_pd_config json
  QFile loadFile_pd("Sources/config/joint_pd_config.json");
  if (!loadFile_pd.open(QIODevice::ReadOnly)) {
    qDebug() << "couldn't open joint_pd_config json";
    return;
  }

  QByteArray all_data_pd = loadFile_pd.readAll();
  loadFile_pd.close();

  QJsonParseError json_error_pd{};
  QJsonDocument doc_pd(QJsonDocument::fromJson(all_data_pd, &json_error_pd));

  if (json_error_pd.error != QJsonParseError::NoError) {
    qDebug() << "json error!" << json_error_pd.errorString();
    return;
  }

  if (!doc_pd.isNull() && json_error_pd.error == QJsonParseError::NoError) {
    if (doc_pd.isObject()) {
      QJsonObject object = doc_pd.object();
      for (int i = 0; i < joint_num_; i++) {
        bool find_flag = false;
        QJsonObject::iterator it = object.begin();
        // read the dimension of the variables
        while (it != object.end()) {
          if (it.key() == QString::fromStdString(joint_name_[i])) {
            find_flag = true;
            QJsonObject it_object = it.value().toObject();
            QJsonObject::iterator iter = it_object.begin();
            while (iter != it_object.end()) {
              // if (iter.key() == "absolute_pos_zero") {
              //   absolute_pos_zero_(i) = iter.value().toDouble();
              // }
              if (iter.key() == "control_config") {
                QJsonObject iter_object = iter.value().toObject();
                QJsonObject::iterator config_it = iter_object.begin();
                while (config_it != iter_object.end()) {
                  if (config_it.key() == "Kp") {
                    joint_Kp_[i] = config_it.value().toDouble();
                  }
                  if (config_it.key() == "Kd") {
                    joint_Kd_[i] = config_it.value().toDouble();
                  }
                  config_it++;
                }
              }
              iter++;
            }
          }
          it++;
        }
        if (!find_flag) {
          std::cout << "joint " << joint_name_[i] << " not found!" << std::endl;
        }
      }
    }
  }

  // read standard_joint_abs_config json
  // QFile loadFile_abs("Sources/config/standard_joint_abs_config.json");
  QString path = "/root/.adam/joint_abs_config.json";
  QFile loadFile_abs(path);  // change to local default path
  if (!loadFile_abs.open(QIODevice::ReadOnly)) {
    qDebug() << "couldn't open joint_abs_config json";
    return;
  }

  QByteArray all_data_abs = loadFile_abs.readAll();
  loadFile_abs.close();

  QJsonParseError json_error_abs{};
  QJsonDocument doc_abs(QJsonDocument::fromJson(all_data_abs, &json_error_abs));
  if (json_error_abs.error != QJsonParseError::NoError) {
    qDebug() << "json error!" << json_error_abs.errorString();
    return;
  }

  if (!doc_abs.isNull() && json_error_abs.error == QJsonParseError::NoError) {
    if (doc_abs.isObject()) {
      QJsonObject object = doc_abs.object();
      for (int i = 0; i < joint_num_; i++) {
        bool find_flag = false;
        QJsonObject::iterator it = object.begin();
        // read the dimension of the variables
        while (it != object.end()) {
          if (it.key() == QString::fromStdString(joint_name_[i])) {
            find_flag = true;
            QJsonObject it_object = it.value().toObject();
            QJsonObject::iterator iter = it_object.begin();
            while (iter != it_object.end()) {
              if (iter.key() == "absolute_pos_zero") {
                absolute_pos_zero_(i) = iter.value().toDouble();
              }
              if (iter.key() == "absolute_pos_dir") {
                absolute_pos_dir_(i) = iter.value().toDouble();
              }
              if (iter.key() == "absolute_pos_gear_ratio") {
                absolute_pos_gear_ratio_(i) = iter.value().toDouble();
              }
              if (iter.key() == "motor_rotor_abs_pos") {
                motor_rotor_abs_pos_(i) = iter.value().toDouble();
              }
              // if (iter.key() == "control_config") {
              //   QJsonObject iter_object = iter.value().toObject();
              //   QJsonObject::iterator config_it = iter_object.begin();
              //   while (config_it != iter_object.end()) {
              //     if (config_it.key() == "Kp") {
              //       joint_Kp_[i] = config_it.value().toDouble();
              //     }
              //     if (config_it.key() == "Kd") {
              //       joint_Kd_[i] = config_it.value().toDouble();
              //     }
              //     config_it++;
              //   }
              // }
              iter++;
            }
          }
          it++;
        }
        if (!find_flag) {
          std::cout << "joint " << joint_name_[i] << " not found!" << std::endl;
        }
      }
    }
  }

  std::cout << "motor_rotor_abs_pos_: " << motor_rotor_abs_pos_.transpose() << std::endl;
  std::cout << "motor_rotor_cur_pos_: " << motor_rotor_cur_pos_.transpose() << std::endl;
  // abs set
  Eigen::VectorXd absolute_pos = Eigen::VectorXd::Zero(joint_num_);
  // Eigen::VectorXd absolute_pos_dir_ = Eigen::VectorXd::Zero(joint_num_);
  // Eigen::VectorXd absolute_pos_gear_ratio_ =
  // Eigen::VectorXd::Zero(joint_num_);
  absolute_pos = absolute_pos_init_ - absolute_pos_zero_;
  // absolute_pos_dir_ << -1, 1, -1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1,
  // 1,
  //     1, 1, 1, 1, 1, 1;
  // absolute_pos_gear_ratio_ << 3, 3.3846, 2.7143, 1, 1, 1, 3, 3.3846, 2.7143,
  // 1,
  // 1, 1, 1, 1, 2.7143, 1, 1, 1, 1, 1, 1, 1, 1;

  for (int i = 0; i < joint_num_; i++) {
    if (absolute_pos(i) < -M_PI) {
      absolute_pos(i) = absolute_pos(i) + 2.0 * M_PI;
    } else if (absolute_pos(i) > M_PI) {
      absolute_pos(i) = absolute_pos(i) - 2.0 * M_PI;
    }
    absolute_pos(i) = absolute_pos(i) / (absolute_pos_dir_(i) * absolute_pos_gear_ratio_(i));
  }
  std::cout << "absolute_pos_init_: " << absolute_pos_init_.transpose() << std::endl;
  std::cout << "absolute_pos_zero_: " << absolute_pos_zero_.transpose() << std::endl;
  std::cout << "initial_pos: " << absolute_pos.transpose() << std::endl;
  // absolute_pos trans to parallel
  funS2P = new functionIKID_S2P();
  p_q_a_ = Eigen::VectorXd::Zero(4);
  p_q_dot_a_ = Eigen::VectorXd::Zero(4);
  p_tau_a_ = Eigen::VectorXd::Zero(4);
  p_q_d_ = Eigen::VectorXd::Zero(4);
  p_q_dot_d_ = Eigen::VectorXd::Zero(4);
  p_tau_d_ = Eigen::VectorXd::Zero(4);
  ankleOrienEst = Eigen::VectorXd::Zero(4);
  ankleOmegaEst = Eigen::VectorXd::Zero(4);
  ankleTorEst = Eigen::VectorXd::Zero(4);
  ankleOrienRef = Eigen::VectorXd::Zero(4);
  ankleOmegaRef = Eigen::VectorXd::Zero(4);
  ankleTorDes = Eigen::VectorXd::Zero(4);
  ankleOrienRef(0) = absolute_pos(4);
  ankleOrienRef(1) = absolute_pos(5);
  ankleOrienRef(2) = absolute_pos(10);
  ankleOrienRef(3) = absolute_pos(11);
  funS2P->setDes(ankleOrienRef, ankleOmegaRef);
  funS2P->calcJointPosRef();
  funS2P->setDesTorque(ankleTorDes);
  funS2P->calcJointTorDes();
  funS2P->getDes(p_q_d_, p_q_dot_d_, p_tau_d_);
  absolute_pos(4) = p_q_d_(0);
  absolute_pos(5) = p_q_d_(1);
  absolute_pos(10) = p_q_d_(2);
  absolute_pos(11) = p_q_d_(3);
  // wrist trans between series and parallel
  wristOrien_parallet = Eigen::VectorXd::Zero(4);
  wristOmega_parallet = Eigen::VectorXd::Zero(4);
  wristTor_parallet = Eigen::VectorXd::Zero(4);
  wristOrien_series = Eigen::VectorXd::Zero(4);
  wristOmega_series = Eigen::VectorXd::Zero(4);
  wristTor_series = Eigen::VectorXd::Zero(4);
  if (adam_type == ADAM_TYPE::AdamStandard || adam_type == ADAM_TYPE::StandardPlus29) {
    wristOrien_parallet(0) = absolute_pos(20);
    wristOrien_parallet(1) = absolute_pos(21);
    wristOrien_parallet(2) = absolute_pos(27);
    wristOrien_parallet(3) = absolute_pos(28);
    wristOrien_series = PND::Algorithm::PosParaToSeries(wristOrien_parallet);
    absolute_pos.segment(20, 2) = wristOrien_series.head(2);
    absolute_pos.segment(27, 2) = wristOrien_series.tail(2);
  } else {
  }
  // kp,kd sim-to-real
  Eigen::VectorXd kd_scale = Eigen::VectorXd::Zero(joint_num_);
  // qdd
  if (adam_type == ADAM_TYPE::AdamLite) {
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0;
  } else if (adam_type == ADAM_TYPE::AdamStandard) {
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus23) {
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus29) {
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  } else if (adam_type == ADAM_TYPE::StandardPlus53) {
  }
  if(adam_type==ADAM_TYPE::AdamLite){
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    kd_scale << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }
  // kd_scale << 15.99, 21.91, 1.77, 1.85, 31.25, 31.25,
  //             15.99, 22.29, 1.73, 1.85, 31.25, 31.25;
  // 21.19, 22.30, 21.50,
  // 19.80, 19.80, 19.80, 19.80,
  // 19.80, 19.80, 19.80, 19.80;
  // harm
  // kd_scale << 15.99, 21.91, 1.77, 1.85, 39.6, 39.6,
  //     15.99, 22.29, 1.73, 1.85, 39.6, 39.6,
  //     21.19, 22.30, 21.50,
  //     19.80, 19.80, 19.80, 19.80,
  //     19.80, 19.80, 19.80, 19.80;
  for (int i = 0; i < joint_num_; i++) {
    if (i == 5 || i == 11) {
      joint_Kp_(i) = joint_Kp_(i - 1);
      joint_Kd_(i) = joint_Kd_(i - 1);
    } else {
      joint_Kp_(i) = joint_Kp_(i) / joint_Kd_(i);
      joint_Kd_(i) = joint_Kd_(i) / kd_scale(i);
    }
  }
  std::cout << "joint_Kp: " << joint_Kp_.transpose() << std::endl;
  std::cout << "joint_Kd: " << joint_Kd_.transpose() << std::endl;

  // joint_Kp_.setZero();
  // joint_Kd_.setZero();
  //
  // joint_Kp_ *=0.1;
  // joint_Kd_*=0.1;
  // int a = 0;
  // std::cout << "confirm the motor para, press 1 and enter to confirm: " <<
  // std::endl; std::cin >> a; std::cout << "confirm!" << std::endl;

  std::cout << "joint_num_: " << joint_num_ << std::endl;
  joint_dir_ = Eigen::VectorXd::Zero(joint_num_);
  joint_gear_ratio_ = Eigen::VectorXd::Zero(joint_num_);
  Eigen::VectorXd abs_pos_dir_ = Eigen::VectorXd::Ones(joint_num_);
  if(adam_type==ADAM_TYPE::AdamLite){
    joint_dir_ << -1, -1, 1, 1, -1, 1,
                  1, -1, 1, -1, 1, -1,
                  1, -1, -1,
                  -1, 1, 1,-1,
                  1, 1, 1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51,
                        51, 51, 51, 51;
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    joint_dir_ << -1, -1, 1, 1, -1, 1,
                   1, -1, 1, -1, 1, -1,
                   1, -1, -1,
                  -1, 1, 1,-1, 1, -1, 1, 1,
                   1, 1, 1, 1, 1, 1, -1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                         7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51, 51;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    joint_dir_ << -1, -1, 1, 1, -1, 1,
                  1, -1, 1, -1, 1, -1,
                  1, -1, -1,
                  -1, 1, 1,-1,
                  1, 1, 1, 1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                        7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51,
                        51, 51, 51, 51;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    joint_dir_ << -1, -1, 1, 1, -1, 1,
                   1, -1, 1, -1, 1, -1,
                   1, -1, -1,
                  -1, 1, 1,-1, 1, -1, 1,
                   1, 1, 1, 1, 1, 1, -1;
    joint_gear_ratio_ << 7, 31, 51, 7, 30, 30,
                         7, 31, 51, 7, 30, 30,
                        51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51,
                        51, 51, 51, 51, 51, 51, 51;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
  }
  auto abs_pos_dir = absolute_pos_dir_;
  auto joint_names = joint_name_;// 关节名称
  auto joint_gear_ratio = joint_gear_ratio_;// 减速器减速比，从 JointInterface.cpp 拷贝
  auto joint_dir = joint_dir_;// 关节正方向，从 JointInterface.cpp 拷贝
  auto motor_enc_zero = motor_rotor_abs_pos_;// 电机端编码器绝对位置，从 配置文件 读取
  auto abs_pos_gear_ratio = absolute_pos_gear_ratio_;// 都是1，从 配置文件 读取
  auto abs_pos_zero = absolute_pos_zero_;// 全是0，从 配置文件 读取
  Eigen::VectorXd min_end_detect_pos = Eigen::VectorXd::Zero(15);// joint_num_随着模型不一样也更新
  min_end_detect_pos.setZero();
  min_end_detect_pos << -2.09, -0.78, -0.78, -0.09, -1, -0.3491,
                        -2.09, -1.57, -0.78, -0.09, -1, -0.3491,
                        -0.52, -0.78, -0.78;
  
  // auto check_zero_idx = PConfig::getInst().jointIdFromNames(
  //     {"hipRoll_Left", "hipYaw_Left", "hipPitch_Right", "hipRoll_Right", "hipYaw_Right", "waistYaw"});
  auto check_zero_idx = {1, 2, 6, 7, 8, 14};// 这里直接替换了序号

  for (const auto i : check_zero_idx) {
    double end_pos = 0.0;
    std::cout << joint_names[i] << ":" << joint_gear_ratio(i) << " " << abs_pos_gear_ratio(i) << " "
              << radToDeg(min_end_detect_pos(i)) << " " << radToDeg(motor_enc_zero(i)) << " "
              << radToDeg(abs_pos_zero(i)) << " " << joint_dir(i) << " " << abs_pos_dir(i) << " "
              << radToDeg(motor_rotor_cur_pos_(i)) << " " << radToDeg(absolute_pos_init_(i)) << std::endl;

    PndAbsCheck(joint_gear_ratio(i), abs_pos_gear_ratio(i), radToDeg(min_end_detect_pos(i)), 1.0, 0, 0,
                radToDeg(motor_enc_zero(i)), radToDeg(abs_pos_zero(i)), joint_dir(i), abs_pos_dir(i),
                radToDeg(motor_rotor_cur_pos_(i)), radToDeg(absolute_pos_init_(i)), end_pos);
    std::cout << end_pos << " " << std::endl;
    if (!std::isnan(end_pos) && end_pos < 180 / abs_pos_gear_ratio(i) && end_pos > -180 / abs_pos_gear_ratio(i)) {
      std::cout << joint_names[i] << " check zero success" << std::endl;
    } else {
      std::cout << joint_names[i] << " check zero failed" << std::endl;
      exit(1);
    }
  }

  jointInterface_->Init(absolute_pos, joint_Kp_, joint_Kd_);

#endif
  // int a;
  // std::cout << " confirm the motor para, press 1 and enter to confirm: " <<
  // std::endl; std::cin >> a; std::cout << "confirm!" << std::endl;
#ifdef STATE_ESTIMATE
  stateEstimate_->Init();
#endif
  std::cout << "Robotinterface initialized! " << std::endl;

#ifdef parallel_data
  std::string filename = "parallel.txt";
  fout_data.open(filename, std::ios::out);

#endif
}

void RobotInterfaceImpl::GetState(double t, Eigen::VectorXd imu_data, RobotData& robot_data) {
  // update joint pos vel tau

  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(joint_num_);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(joint_num_);
  Eigen::VectorXd joint_tau = Eigen::VectorXd::Zero(joint_num_);
  double communication_time = 0;
#ifdef WEBOTS
  humanoid.robot->step(TIME_STEP);
  simTime = humanoid.robot->getTime();
  humanoid.readData(simTime, robotStateSim);
  joint_pos = robotStateSim.jointPosAct;
  joint_vel = robotStateSim.jointVelAct;
  joint_tau = robot_data.tau_d_.tail(joint_num_);
  robot_data.q_a_.tail(joint_num_) = joint_pos;
  robot_data.q_dot_a_.tail(joint_num_) = joint_vel;
  robot_data.tau_a_.tail(joint_num_) = joint_tau;
  imu_data_ = robotStateSim.imu9DAct;

#else
  jointInterface_->GetState(joint_pos, joint_vel, joint_tau, communication_time);
  error_state_ = jointInterface_->joint_error_;
  robot_data.q_a_.segment(6, joint_num_) = joint_pos;
  robot_data.q_dot_a_.segment(6, joint_num_) = joint_vel;
  robot_data.tau_a_.segment(6, joint_num_) = joint_tau;
  // q_a_, q_dot_a_, tau_a_ trans to serial
  p_q_a_ << joint_pos(4), joint_pos(5), joint_pos(10), joint_pos(11);
  p_q_dot_a_ << joint_vel(4), joint_vel(5), joint_vel(10), joint_vel(11);
  p_tau_a_ << joint_tau(4), joint_tau(5), joint_tau(10), joint_tau(11);
  funS2P->setEst(p_q_a_, p_q_dot_a_, p_tau_a_);  // set parallel joint pos and
                                                 // vel
  funS2P->calcFK();                              // calc serial joint pos
  funS2P->calcIK();                              // calc jacobian at this pose and serial joint vel
  funS2P->getAnkleState(ankleOrienEst, ankleOmegaEst, ankleTorEst);
  robot_data.q_a_(10) = ankleOrienEst(0);
  robot_data.q_a_(11) = ankleOrienEst(1);
  robot_data.q_a_(16) = ankleOrienEst(2);
  robot_data.q_a_(17) = ankleOrienEst(3);
  robot_data.q_dot_a_(10) = ankleOmegaEst(0);
  robot_data.q_dot_a_(11) = ankleOmegaEst(1);
  robot_data.q_dot_a_(16) = ankleOmegaEst(2);
  robot_data.q_dot_a_(17) = ankleOmegaEst(3);
  robot_data.tau_a_(10) = ankleTorEst(0);
  robot_data.tau_a_(11) = ankleTorEst(1);
  robot_data.tau_a_(16) = ankleTorEst(2);
  robot_data.tau_a_(17) = ankleTorEst(3);
  // wrist parallet to series
  if (adam_type == ADAM_TYPE::AdamStandard || adam_type == ADAM_TYPE::StandardPlus29) {
    wristOrien_parallet << joint_pos(20), joint_pos(21), joint_pos(27), joint_pos(28);
    wristOmega_parallet << joint_vel(20), joint_vel(21), joint_vel(27), joint_vel(28);
    wristTor_parallet << joint_tau(20), joint_tau(21), joint_tau(27), joint_tau(28);
    wristOrien_series = PND::Algorithm::PosParaToSeries(wristOrien_parallet);
    wristOmega_series = PND::Algorithm::VelParaToSeries(wristOmega_parallet);
    wristTor_series = PND::Algorithm::TauParaToSeries(wristTor_parallet);
    robot_data.q_a_.segment(6 + 20, 2) = wristOrien_series.head(2);
    robot_data.q_a_.segment(6 + 27, 2) = wristOrien_series.tail(2);
    robot_data.q_dot_a_.segment(6 + 20, 2) = wristOmega_series.head(2);
    robot_data.q_dot_a_.segment(6 + 27, 2) = wristOmega_series.tail(2);
    robot_data.tau_a_.segment(6 + 20, 2) = wristTor_series.head(2);
    robot_data.tau_a_.segment(6 + 27, 2) = wristTor_series.tail(2);
  } else {
  }

  // read imu data
  imu_data_ = imu_data;
  // recorde communication_time
  robot_data.communication_time = communication_time;

#endif

  // state estimate
  // imu data trans to world coordinate
  Eigen::Vector3d ypr_a = imu_data_.head(3);
  Eigen::Matrix3d NED_R_YPR_a = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  ypr_a(0) = 0.0;
  EulerZYXToMatrix(NED_R_YPR_a, ypr_a);
  MatrixToEulerXYZ(NED_R_YPR_a, rpy);
  robot_data.q_a_.segment(3, 3) = rpy;

  Eigen::Matrix3d R_xyz_omega = Eigen::Matrix3d::Identity();
  R_xyz_omega.row(1) = RotX(rpy(0)).row(1);
  R_xyz_omega.row(2) = (RotX(rpy(0)) * RotY(rpy(1))).row(2);
  robot_data.q_dot_a_.segment(3, 3) = R_xyz_omega.transpose() * NED_R_YPR_a * imu_data_.segment(3, 3);

#ifdef STATE_ESTIMATE
  // est floating base vel
  Eigen::VectorXd leg_pos = Eigen::VectorXd::Zero(18);
  Eigen::VectorXd leg_vel = Eigen::VectorXd::Zero(18);
  Eigen::VectorXd leg_tau = Eigen::VectorXd::Zero(18);
  Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);

  leg_pos = robot_data.q_a_.head(18);
  leg_vel = robot_data.q_dot_a_.head(18);
  leg_tau = robot_data.tau_a_.head(18);
  acc = imu_data_.tail(3);

  Eigen::Vector3d estVel;
  stateEstimate_->EstVelocity(t, leg_pos, leg_vel, leg_tau, acc, estVel);

  robot_data.q_a_.head(3) << 0., 0., leg_pos(2);
  robot_data.q_dot_a_.head(3) = estVel;
#endif
  //

#ifdef parallel_data
  dataL.block(0, 0, 4, 1) = p_q_a_;
  dataL.block(4, 0, 4, 1) = p_q_dot_a_;
  dataL.block(8, 0, 4, 1) = p_tau_a_;
#endif
}

void RobotInterfaceImpl::SetCommand(RobotData& robot_data) {
#ifdef WEBOTS
  joint_Kp_ << 328.5, 200.0, 300.0, 300.0, 5.0, 0.0, 328.5, 200.0, 300.0, 300.0, 5.0, 0.0, 200.0, 200.0, 200.0, 10.0,
      10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
  joint_Kd_ << 13.14, 4.0, 6.0, 6.0, 1.25, 0.175, 13.14, 4.0, 6.0, 6.0, 1.25, 0.175, 4.0, 4.0, 4.0, 1., 1., 1., 1., 1.,
      1., 1., 1.;
  for (int i = 0; i < joint_num_; i++) {
    robot_data.tau_d_(i + 6) =
        joint_Kp_(i) * (robot_data.q_d_(i + 6) - robot_data.q_a_(i + 6)) + joint_Kd_(i) * (-robot_data.q_dot_a_(i + 6));
  }

  if (robot_data.pos_mode_) {
    humanoid.setMotorPos(robot_data.q_d_.tail(joint_num_));
  } else {
    humanoid.setMotorTau(robot_data.tau_d_.tail(joint_num_));
  }

#else
  // S2P
  ankleOrienRef(0) = robot_data.q_d_(10);
  ankleOrienRef(1) = robot_data.q_d_(11);
  ankleOrienRef(2) = robot_data.q_d_(16);
  ankleOrienRef(3) = robot_data.q_d_(17);

  ankleOmegaRef(0) = robot_data.q_dot_d_(10);
  ankleOmegaRef(1) = robot_data.q_dot_d_(11);
  ankleOmegaRef(2) = robot_data.q_dot_d_(16);
  ankleOmegaRef(3) = robot_data.q_dot_d_(17);
  // time3 = timer.currentTime() - start_time - time2 - time1;
  // // s2p
  ankleTorDes << robot_data.tau_d_(10), robot_data.tau_d_(11), robot_data.tau_d_(16), robot_data.tau_d_(17);
  funS2P->setDes(ankleOrienRef, ankleOmegaRef);
  funS2P->calcJointPosRef();
  funS2P->setDesTorque(ankleTorDes);
  funS2P->calcJointTorDes();
  funS2P->getDes(p_q_d_, p_q_dot_d_, p_tau_d_);
  robot_data.q_d_(10) = p_q_d_(0);
  robot_data.q_d_(11) = p_q_d_(1);
  robot_data.q_d_(16) = p_q_d_(2);
  robot_data.q_d_(17) = p_q_d_(3);
  robot_data.q_dot_d_(10) = p_q_dot_d_(0);
  robot_data.q_dot_d_(11) = p_q_dot_d_(1);
  robot_data.q_dot_d_(16) = p_q_dot_d_(2);
  robot_data.q_dot_d_(17) = p_q_dot_d_(3);
  robot_data.tau_d_(10) = p_tau_d_(0);
  robot_data.tau_d_(11) = p_tau_d_(1);
  robot_data.tau_d_(16) = p_tau_d_(2);
  robot_data.tau_d_(17) = p_tau_d_(3);
  // wrist series to parallet
  if (adam_type == ADAM_TYPE::AdamStandard || adam_type == ADAM_TYPE::StandardPlus29) {
    wristOrien_series << robot_data.q_d_(6 + 20), robot_data.q_d_(6 + 21), robot_data.q_d_(6 + 27),
        robot_data.q_d_(6 + 28);
    wristOmega_series << robot_data.q_dot_d_(6 + 20), robot_data.q_dot_d_(6 + 21), robot_data.q_dot_d_(6 + 27),
        robot_data.q_dot_d_(6 + 28);
    wristTor_series << robot_data.tau_d_(6 + 20), robot_data.tau_d_(6 + 21), robot_data.tau_d_(6 + 27),
        robot_data.tau_d_(6 + 28);
    wristOrien_parallet = PND::Algorithm::PosSeriesToPara(wristOrien_series);
    wristOmega_parallet = PND::Algorithm::VelSeriesToPara(wristOmega_series);
    wristTor_parallet = PND::Algorithm::TauSeriesToPara(wristTor_series);
    robot_data.q_d_.segment(6 + 20, 2) = wristOrien_parallet.head(2);
    robot_data.q_d_.segment(6 + 27, 2) = wristOrien_parallet.tail(2);
    robot_data.q_dot_d_.segment(6 + 20, 2) = wristOmega_parallet.head(2);
    robot_data.q_dot_d_.segment(6 + 27, 2) = wristOmega_parallet.tail(2);
    robot_data.tau_d_.segment(6 + 20, 2) = wristTor_parallet.head(2);
    robot_data.tau_d_.segment(6 + 27, 2) = wristTor_parallet.tail(2);
  } else {
  }
  // send command
  jointInterface_->SetCommand(robot_data.q_d_.segment(6, joint_num_), robot_data.q_dot_d_.segment(6, joint_num_),
                              robot_data.tau_d_.segment(6, joint_num_));
#endif

#ifdef parallel_data
  dataL.block(12, 0, 4, 1) = p_q_d_;
  dataL.block(16, 0, 4, 1) = p_q_dot_d_;
  dataL.block(20, 0, 4, 1) = p_tau_d_;
  for (double i : dataL) {
    fout_data << i << " ";
  }
  fout_data << std::endl;
#endif
}

Eigen::Matrix3d RobotInterfaceImpl::RotX(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << 1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c;
  return m_ret;
}
Eigen::Matrix3d RobotInterfaceImpl::RotY(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return m_ret;
}
Eigen::Matrix3d RobotInterfaceImpl::RotZ(double q) {
  Eigen::Matrix3d m_ret;
  double s = sin(q), c = cos(q);
  m_ret << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return m_ret;
}

void RobotInterfaceImpl::EulerZYXToMatrix(Eigen::Matrix3d& R, Eigen::Vector3d euler_a) {
  R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
}

void RobotInterfaceImpl::MatrixToEulerXYZ(Eigen::Matrix3d R, Eigen::Vector3d& euler) {
  euler.setZero();
  // y (-pi/2 pi/2)
  euler(1) = asin(R(0, 2));
  // z [-pi pi]
  double sinz = -R(0, 1) / cos(euler(1));
  double cosz = R(0, 0) / cos(euler(1));
  euler(2) = atan2(sinz, cosz);
  // x [-pi pi]
  double sinx = -R(1, 2) / cos(euler(1));
  double cosx = R(2, 2) / cos(euler(1));
  euler(0) = atan2(sinx, cosx);
}

RobotInterfaceImpl::~RobotInterfaceImpl() {
#ifdef STATE_ESTIMATE
  delete stateEstimate_;
#endif
#ifdef WEBOTS
  humanoid.deleteRobot();
#else
  delete jointInterface_;
  delete funS2P;
#endif
}

void RobotInterfaceImpl::DisableAllJoints() {
#ifndef WEBOTS
  jointInterface_->Disable();
#endif
}

#ifndef WEBOTS
AdamStatusCode RobotInterfaceImpl::ReadAbsEncoder(Eigen::VectorXd& init_pos, Eigen::VectorXd& motor_rotor_abs_pos) {
  // read the json file
  QFile loadFile("Sources/python_scripts/abs.json");
  if (!loadFile.open(QIODevice::ReadOnly)) {
    qDebug() << "couldn't open abs json";
    return AdamStatusCode::UNKNOW;
  }

  QByteArray allData = loadFile.readAll();
  loadFile.close();

  QJsonParseError json_error{};
  QJsonDocument doc(QJsonDocument::fromJson(allData, &json_error));

  if (json_error.error != QJsonParseError::NoError) {
    qDebug() << "json error!" << json_error.errorString();
    return AdamStatusCode::UNKNOW;
  }

  if (!doc.isNull() && json_error.error == QJsonParseError::NoError) {

    if (doc.isObject()) {
      QJsonObject object = doc.object();
      QJsonObject::iterator it = object.begin();
      // read the dimension of the variables
      while (it != object.end()) {
        if (it.key() == "10.10.10.80") {
          QJsonObject it_object = it.value().toObject();
          QJsonObject::iterator iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(0) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(0) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.81") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(1) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(1) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.82") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(2) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(2) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.83") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(3) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(3) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.84") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(4) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(4) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.85") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(5) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(5) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.60") {
          QJsonObject it_object = it.value().toObject();
          QJsonObject::iterator iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(6) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(6) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.61") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(7) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(7) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.62") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(8) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(8) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.63") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(9) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(9) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.64") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(10) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(10) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.65") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(11) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(11) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.100") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(12) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(12) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.101") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(13) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(13) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.102") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(14) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(14) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.20") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(15) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(15) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.21") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(16) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(16) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.22") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(17) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(17) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.23") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(18) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(18) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.40") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(19) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(19) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.41") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(20) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(20) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.42") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(21) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(21) = iter.value().toDouble();
            }
            iter++;
          }
        }
        if (it.key() == "10.10.10.43") {
          QJsonObject it_object = it.value().toObject();
          auto iter = it_object.begin();
          while (iter != it_object.end()) {
            if (iter.key() == "radian") {
              init_pos(22) = iter.value().toDouble();
            }
            if (iter.key() == "motor_rotor_abs_pos") {
              motor_rotor_abs_pos(22) = iter.value().toDouble();
            }
            iter++;
          }
        }
        it++;
      }
    }
  }
  return AdamStatusCode::AdamStatusSuccess;
}

PndAlgorithmStatusCode RobotInterfaceImpl::PndAbsCheck(
    double motor_rotor_enc_gear_ratio, double end_enc_gear_ratio, double min_end_detect_pos, double detect_accuracy,
    double motor_rotor_enc_noise, double end_enc_noise, double motor_rotor_enc_offset, double end_enc_offset,
    int motor_rotor_enc_dir, int end_enc_dir, double motor_rotor_enc_cur_pos, double end_enc_cur_pos, double& end_pos) {
  DoubleEncoderSystem double_encoder_system(motor_rotor_enc_gear_ratio, end_enc_gear_ratio, min_end_detect_pos,
                                            detect_accuracy, motor_rotor_enc_noise, end_enc_noise);
  double_encoder_system.SetEncoder1Offset(motor_rotor_enc_offset);
  double_encoder_system.SetEncoder2Offset(end_enc_offset);
  double_encoder_system.SetEncoder1Dir(motor_rotor_enc_dir);
  double_encoder_system.SetEncoder2Dir(end_enc_dir);
  end_pos = double_encoder_system.GetEndPos(motor_rotor_enc_cur_pos, end_enc_cur_pos);
  // std::cout << double_encoder_system.GetAllowedEncoderNoise()[0] << " " <<
  // double_encoder_system.GetAllowedEncoderNoise()[1]
  //           << std::endl;
  return PndAlgorithmSuccess;
}

double RobotInterfaceImpl::radToDeg(double radians) { return radians * (180.0 / M_PI); }
#endif