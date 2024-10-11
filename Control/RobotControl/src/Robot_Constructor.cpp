#include "../include/Robot_Constructor.h"
// #include "Robot_Constructor.h"
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
#include <vector>
#include "public_parament.h"

#include <rbdl/Model.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

Robot_Constructor::Robot_Constructor() {}
// to be done:
// friction cone constriants
void Robot_Constructor::robotconstructor(QString path, Robot_Data* robotdata) {
  // task card set
  int _npriority = -1;
  int _ntask = 0;
  // std::vector<Task*> _task_card_set;
  // sensor set
  int _nsensor;
  // std::vector<Sensor*> _sensor_set;
  // tau external
  Eigen::VectorXd _tau_ext;
  // robot type
  enum robot_type _robottype;
  // solver type
  enum Wbc_Solver_type _wbcsolver;

  // task direction selection
  std::vector<bool> _task_direction_selection;
  _task_direction_selection.resize(6);
  for (int i = 0; i < 6; i++) {
    _task_direction_selection[i] = true;
  }
  // task weight
  std::vector<double> _task_weight;
  _task_weight.resize(6);
  for (int i = 0; i < 6; i++) {
    _task_weight[i] = 1.0;
  }

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
      while (it != object.end()) {
        if (it.key() == "urdf") {
          // construct the rbdl robot model
          robotdata->robot_model = new RigidBodyDynamics::Model();
          robotdata->robot_model->gravity << 0.0, 0.0, -9.81;

          std::cout << "using urdf:" << it.value().toString().toStdString() << std::endl;

          std::string str = it.value().toString().toStdString();
          // find last /
          size_t last_slash_pos = str.find_last_of('/');
          if (last_slash_pos == std::string::npos) {
            last_slash_pos = 0;
          } else {
            last_slash_pos += 1;
          }
          // find last .
          size_t last_dot_pos = str.find_last_of('.');
          if (last_dot_pos == std::string::npos) {
            last_dot_pos = str.length();
          }
          if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "standard_plus23") {
            std::cout << "standard_plus23" << std::endl;
            adam_type = ADAM_TYPE::StandardPlus23;
            floating_base_dof = 6;
            actor_num = 23;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 4;
            hand_l_actor_num = 0;
            arm_r_actor_num = 4;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "standard_plus29") {
            std::cout << "standard_plus29" << std::endl;
            adam_type = ADAM_TYPE::StandardPlus29;
            floating_base_dof = 6;
            actor_num = 29;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 7;
            hand_l_actor_num = 0;
            arm_r_actor_num = 7;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "standard_plus53") {
            std::cout << "standard_plus53" << std::endl;
            adam_type = ADAM_TYPE::StandardPlus53;
            floating_base_dof = 6;
            actor_num = 53;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 7;
            hand_l_actor_num = 12;
            arm_r_actor_num = 7;
            hand_r_actor_num = 12;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "adam_standard") {
            std::cout << "adam_standard" << std::endl;
            adam_type = ADAM_TYPE::AdamStandard;
            floating_base_dof = 6;
            actor_num = 31;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 8;
            hand_l_actor_num = 0;
            arm_r_actor_num = 8;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "adam_lite") {
            std::cout << "adam_lite" << std::endl;
            adam_type = ADAM_TYPE::AdamLite;
            floating_base_dof = 6;
            actor_num = 23;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 4;
            hand_l_actor_num = 0;
            arm_r_actor_num = 4;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "adam_lite_simple") {
            std::cout << "adam_lite_simple" << std::endl;
            adam_type = ADAM_TYPE::AdamLiteSimple;
            floating_base_dof = 6;
            actor_num = 12;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 0;
            arm_l_actor_num = 0;
            hand_l_actor_num = 0;
            arm_r_actor_num = 0;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else if (str.substr(last_slash_pos, last_dot_pos - last_slash_pos) == "DuckDuck") {
            std::cout << "DuckDuck" << std::endl;
            adam_type = ADAM_TYPE::DuckDuck;
            floating_base_dof = 6;
            actor_num = 15;
            generalized_coordinates = floating_base_dof + actor_num;
            leg_l_actor_num = 6;
            leg_r_actor_num = 6;
            waist_actor_num = 3;
            arm_l_actor_num = 0;
            hand_l_actor_num = 0;
            arm_r_actor_num = 0;
            hand_r_actor_num = 0;
            head_actor_num = 0;
            adam_upper_actor_num = waist_actor_num + arm_l_actor_num + hand_l_actor_num + arm_r_actor_num +
                                   hand_r_actor_num + head_actor_num;
            adam_upper_except_waist_actor_num =
                arm_l_actor_num + hand_l_actor_num + arm_r_actor_num + hand_r_actor_num + head_actor_num;
          } else {
            std::cout << "model select ERROR, please check it." << std::endl;
            adam_type = -1;
            return;
          }

          RigidBodyDynamics::Addons::URDFReadFromFile(it.value().toString().toStdString().c_str(),
                                                      robotdata->robot_model, false);

          robotdata->ndof = robotdata->robot_model->dof_count;
          std::cout << "robotdata->ndof: " << robotdata->ndof << std::endl;
          robotdata->id_body.resize(robotdata->robot_model->mBodies.size());
          std::cout << "robotdata->robot_model->mBodies.size(): " << robotdata->robot_model->mBodies.size()
                    << std::endl;
          for (int i = 0; i < robotdata->id_body.size(); i++) {
            robotdata->id_body[i] = robotdata->robot_model->GetBodyId("base") + i;
          }

          robotdata->wbcsolver = Wbc_Solver_type(2);
          robotdata->robottype = robot_type(2);
        }
        it++;
      }

      it = object.begin();
      // read the dimesion of the variables
      while (it != object.end()) {
        if (it.key() == "taskNum") {
          _ntask = it.value().toInt();
        }
        it++;
      }
      // init set

      // npriority not init
      robotdata->ntask = _ntask;
      robotdata->task_card_set.resize(_ntask);
      robotdata->tau_ext.setZero(robotdata->ndof);
      // ndof_wheel not init
      // radius_wheel not init
      // length_wheel not init
      robotdata->q_lbound.setZero(robotdata->ndof, 1);
      robotdata->q_ubound.setZero(robotdata->ndof, 1);
      robotdata->qd_bound.setZero(robotdata->ndof, 1);
      robotdata->tau_bound.setZero(robotdata->ndof, 1);
      robotdata->q_lbound = Eigen::MatrixXd::Ones(robotdata->ndof, 1) * (-_INF);
      robotdata->q_ubound = Eigen::MatrixXd::Ones(robotdata->ndof, 1) * (_INF);
      robotdata->qd_bound = Eigen::MatrixXd::Ones(robotdata->ndof, 1) * (_INF);

      //            robotdata->tau_bound =
      //            Eigen::MatrixXd::Ones(robotdata->ndof,1)*(0.0);

      robotdata->q_a.setZero(robotdata->ndof, 1);
      robotdata->q_dot_a.setZero(robotdata->ndof, 1);
      robotdata->q_ddot_a.setZero(robotdata->ndof, 1);
      robotdata->tau_a.setZero(robotdata->ndof, 1);

      robotdata->q_d.setZero(robotdata->ndof, 1);
      robotdata->q_dot_d.setZero(robotdata->ndof, 1);
      robotdata->q_ddot_d.setZero(robotdata->ndof, 1);
      robotdata->tau_d.setZero(robotdata->ndof, 1);

      robotdata->q_c.setZero(robotdata->ndof, 1);
      robotdata->q_dot_c.setZero(robotdata->ndof, 1);
      robotdata->q_ddot_c.setZero(robotdata->ndof, 1);
      robotdata->tau_c.setZero(robotdata->ndof, 1);

      // PD gains
      robotdata->q_factor.setZero(robotdata->ndof - 6, 1);
      robotdata->q_dot_factor.setZero(robotdata->ndof - 6, 1);

      // read sensor array and task array
      it = object.begin();
      while (it != object.end()) {
        if (it.key() == "taskConfig") {
          QJsonArray _task_array = it.value().toArray();
          for (int i = 0; i < _task_array.size(); i++) {
            Task* _p_task = new Task();
            robotdata->task_card_set[i] = _p_task;
            robotdata->task_card_set[i]->T_offset.setIdentity();
            QJsonObject t_task_array_object = _task_array.at(i).toObject();
            QJsonObject::iterator _task_array_object = t_task_array_object.begin();
            // update task type and dimesions of the variables
            while (_task_array_object != t_task_array_object.end()) {
              if (_task_array_object.key() == "WeightRoll") {
                _task_weight[0] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "WeightPitch") {
                _task_weight[1] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "WeightYaw") {
                _task_weight[2] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "WeightX") {
                _task_weight[3] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "WeightY") {
                _task_weight[4] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "WeightZ") {
                _task_weight[5] = _task_array_object.value().toDouble();
              }

              if (_task_array_object.key() == "TaskType") {
                if (_task_array_object.value().toString() == "general") {
                  robotdata->task_card_set[i]->type = task_type::general_task;
                } else if (_task_array_object.value().toString() == "CoM") {
                  robotdata->task_card_set[i]->type = task_type::com_task;
                } else if (_task_array_object.value().toString() == "contact") {
                  robotdata->task_card_set[i]->type = task_type::contact_task;
                } else if (_task_array_object.value().toString() == "joint") {
                  robotdata->task_card_set[i]->type = task_type::joint_task;
                } else {
                  std::cout << "no such type task!" << std::endl;
                  return;
                }
              }
              // record biped robot task
              if (_task_array_object.key() == "TaskName") {
                if (_task_array_object.value().toString() == "left_foot") {
                  robotdata->left_foot_id = i;
                }

                if (_task_array_object.value().toString() == "right_foot") {
                  robotdata->right_foot_id = i;
                }

                if (_task_array_object.value().toString() == "base") {
                  robotdata->body_task_id = i;
                }

                if (_task_array_object.value().toString() == "CoM") {
                  robotdata->com_task_id = i;
                }

                if (_task_array_object.value().toString() == "torso") {
                  robotdata->chest_task_id = i;
                }

                if (_task_array_object.value().toString() == "arm_waist_joints") {
                  robotdata->upper_joints_id = i;
                }
              }

              // record start joint and joints num
              if (_task_array_object.key() == "StartJoint") {
                robotdata->task_card_set[i]->joint_id = _task_array_object.value().toInt();
              }
              if (_task_array_object.key() == "JointsDim") {
                robotdata->task_card_set[i]->dim = _task_array_object.value().toInt();
                robotdata->task_card_set[i]->dim = adam_upper_actor_num;
              }

              // end
              _task_array_object++;
            }

            if (robotdata->task_card_set[i]->type == task_type::joint_task) {
              if (robotdata->robottype == robot_type::Fixed_Base_Open_Chain) {
                robotdata->task_card_set[i]->dim = robotdata->ndof;
                robotdata->task_card_set[i]->jacobi = Eigen::MatrixXd::Identity(robotdata->ndof, robotdata->ndof);
                robotdata->task_card_set[i]->weight = Eigen::MatrixXd::Ones(robotdata->ndof, 1);
                robotdata->task_card_set[i]->jacobi_dot_q_dot = Eigen::MatrixXd::Zero(robotdata->ndof, 1);
              } else if ((robotdata->robottype == robot_type::Float_Base_Open_Chain) ||
                         (robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)) {
                int _dim = robotdata->task_card_set[i]->dim;
                robotdata->task_card_set[i]->jacobi = Eigen::MatrixXd::Zero(_dim, robotdata->ndof);
                robotdata->task_card_set[i]->jacobi.block(0, robotdata->task_card_set[i]->joint_id + 5, _dim, _dim) =
                    Eigen::MatrixXd::Identity(_dim, _dim);
                robotdata->task_card_set[i]->weight = Eigen::MatrixXd::Ones(_dim, 1);
                robotdata->task_card_set[i]->jacobi_dot_q_dot = Eigen::MatrixXd::Zero(_dim, 1);
              }

              robotdata->task_card_set[i]->contact_state_d = false;
              robotdata->task_card_set[i]->X_a.setZero(4, robotdata->task_card_set[i]->dim);
              robotdata->task_card_set[i]->X_d.setZero(4, robotdata->task_card_set[i]->dim);
              robotdata->task_card_set[i]->X_c.setZero(4, robotdata->task_card_set[i]->dim);

              robotdata->task_card_set[i]->T_offset.setIdentity();
              robotdata->task_card_set[i]->IG.setIdentity();
            } else {
              robotdata->task_card_set[i]->dim = 0;

              if (_task_direction_selection[0] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_x_theta);
              }

              if (_task_direction_selection[1] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_y_theta);
              }

              if (_task_direction_selection[2] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_z_theta);
              }

              if (_task_direction_selection[3] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_x);
              }

              if (_task_direction_selection[4] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_y);
              }

              if (_task_direction_selection[5] == true) {
                robotdata->task_card_set[i]->dim += 1;
                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_z);
              }

              robotdata->task_card_set[i]->weight.setOnes(robotdata->task_card_set[i]->dim, 1);
              int count = 0;
              for (int j = 0; j < 6; j++) {
                if (_task_direction_selection[j] == true) {
                  robotdata->task_card_set[i]->weight(count) = _task_weight[j];
                  count++;
                }
              }

              robotdata->task_card_set[i]->contact_state_d = false;
              robotdata->task_card_set[i]->X_a.setZero(4, robotdata->task_card_set[i]->dim);
              robotdata->task_card_set[i]->X_d.setZero(4, robotdata->task_card_set[i]->dim);
              robotdata->task_card_set[i]->X_c.setZero(4, robotdata->task_card_set[i]->dim);

              robotdata->task_card_set[i]->jacobi.setZero(robotdata->task_card_set[i]->dim, robotdata->ndof);
              robotdata->task_card_set[i]->jacobi_dot_q_dot.setZero(robotdata->task_card_set[i]->dim, 1);

              robotdata->task_card_set[i]->T_offset.setIdentity();
              robotdata->task_card_set[i]->IG.setIdentity();
            }

            // update task
            _task_array_object = t_task_array_object.begin();
            while (_task_array_object != t_task_array_object.end()) {
              if (_task_array_object.key() == "TaskLevel") {
                robotdata->task_card_set[i]->priority = _task_array_object.value().toInt();
              }

              if (_task_array_object.key() == "TaskID") {
                robotdata->task_card_set[i]->task_id = _task_array_object.value().toInt();
              }

              if (_task_array_object.key() == "ControlFrame") {
                if (_task_array_object.value().toString() == "world") {
                  robotdata->task_card_set[i]->frame = 1;
                } else if (_task_array_object.value().toString() == "local") {
                  robotdata->task_card_set[i]->frame = 0;
                } else {
                  std::cout << "no such wold frame!" << std::endl;
                  return;
                }
              }

              if (_task_array_object.key() == "RotMatrix") {
                robotdata->task_card_set[i]->T_offset(0, 0) = _task_array_object.value().toArray().at(0).toDouble();
                robotdata->task_card_set[i]->T_offset(0, 1) = _task_array_object.value().toArray().at(1).toDouble();
                robotdata->task_card_set[i]->T_offset(0, 2) = _task_array_object.value().toArray().at(2).toDouble();

                robotdata->task_card_set[i]->T_offset(1, 0) = _task_array_object.value().toArray().at(3).toDouble();
                robotdata->task_card_set[i]->T_offset(1, 1) = _task_array_object.value().toArray().at(4).toDouble();
                robotdata->task_card_set[i]->T_offset(1, 2) = _task_array_object.value().toArray().at(5).toDouble();

                robotdata->task_card_set[i]->T_offset(2, 0) = _task_array_object.value().toArray().at(6).toDouble();
                robotdata->task_card_set[i]->T_offset(2, 1) = _task_array_object.value().toArray().at(7).toDouble();
                robotdata->task_card_set[i]->T_offset(2, 2) = _task_array_object.value().toArray().at(8).toDouble();
              }

              if (_task_array_object.key() == "TransVector") {
                robotdata->task_card_set[i]->T_offset(0, 3) = _task_array_object.value().toArray().at(0).toDouble();
                robotdata->task_card_set[i]->T_offset(1, 3) = _task_array_object.value().toArray().at(1).toDouble();
                robotdata->task_card_set[i]->T_offset(2, 3) = _task_array_object.value().toArray().at(2).toDouble();
              }

              if (_task_array_object.key() == "SolidNum") {
                int _joint_num = _task_array_object.value().toInt();
                if (robotdata->robottype == robot_type::Fixed_Base_Open_Chain) {
                  robotdata->task_card_set[i]->joint_id = robotdata->id_body[_joint_num - 1];
                } else if (robotdata->robottype == robot_type::Float_Base_Open_Chain ||
                           robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain) {
                  robotdata->task_card_set[i]->joint_id = robotdata->id_body[_joint_num];
                } else {
                  std::cout << "no matching robot type!" << std::endl;
                  return;
                }
              }

              _task_array_object++;
            }

            // update task controller and constraints
            _task_array_object = t_task_array_object.begin();
            while (_task_array_object != t_task_array_object.end()) {
              if (_task_array_object.key() == "ControlConfig") {
                robotdata->task_card_set[i]->controller = new Controller_Lib();
                robotdata->task_card_set[i]->controller->dim = robotdata->task_card_set[i]->dim;

                QJsonObject t_task_array_object_object = _task_array_object.value().toObject();
                QJsonObject::iterator _task_array_object_object = t_task_array_object_object.begin();
                // controller type
                while (_task_array_object_object != t_task_array_object_object.end()) {
                  if (_task_array_object_object.key() == "Type") {
                    if (_task_array_object_object.value().toString() == "admittance") {
                      robotdata->task_card_set[i]->controller->controller_type = basic_controller::Admittance;
                    } else if (_task_array_object_object.value().toString() == "impedence") {
                      robotdata->task_card_set[i]->controller->controller_type = basic_controller::Impedance;
                    } else if (_task_array_object_object.value().toString() == "pd") {
                      if (robotdata->task_card_set[i]->task_selection_matrix[0] == task_direction::task_x_theta) {
                        robotdata->task_card_set[i]->controller->controller_type = basic_controller::PID_orient;
                      } else {
                        robotdata->task_card_set[i]->controller->controller_type = basic_controller::PID;
                      }

                    } else if (_task_array_object_object.value().toString() == "joint_pd") {
                      robotdata->task_card_set[i]->controller->controller_type = basic_controller::PID_joint;
                    } else if (_task_array_object_object.value().toString() == "none") {
                      robotdata->task_card_set[i]->controller->controller_type = basic_controller::None;
                    }
                  }
                  _task_array_object_object++;
                }
                // controller para
                robotdata->task_card_set[i]->controller->para.setZero(12, 1);
                _task_array_object_object = t_task_array_object_object.begin();
                while (_task_array_object_object != t_task_array_object_object.end()) {
                  if (robotdata->task_card_set[i]->controller->controller_type == basic_controller::Admittance) {
                    if (_task_array_object_object.key() == "K") {
                      robotdata->task_card_set[i]->controller->para(0, 0) =
                          _task_array_object_object.value().toDouble();
                    }

                    if (_task_array_object_object.key() == "M") {
                      robotdata->task_card_set[i]->controller->para(1, 0) =
                          _task_array_object_object.value().toDouble();
                    }

                    if (_task_array_object_object.key() == "B") {
                      robotdata->task_card_set[i]->controller->para(2, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    robotdata->task_card_set[i]->controller->alter_v.setZero(
                        2, robotdata->task_card_set[i]->controller->dim);
                    robotdata->task_card_set[i]->controller->input_data_a.setZero(
                        4, robotdata->task_card_set[i]->controller->dim);
                    robotdata->task_card_set[i]->controller->input_data_d.setZero(
                        4, robotdata->task_card_set[i]->controller->dim);
                    robotdata->task_card_set[i]->controller->output_data.setZero(
                        4, robotdata->task_card_set[i]->controller->dim);
                  }

                  if (robotdata->task_card_set[i]->controller->controller_type == basic_controller::Impedance) {
                    if (_task_array_object_object.key() == "K") {
                      robotdata->task_card_set[i]->controller->para(0, 0) =
                          _task_array_object_object.value().toDouble();
                    }

                    if (_task_array_object_object.key() == "M") {
                      robotdata->task_card_set[i]->controller->para(1, 0) =
                          _task_array_object_object.value().toDouble();
                    }

                    if (_task_array_object_object.key() == "B") {
                      robotdata->task_card_set[i]->controller->para(2, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                  }

                  if (robotdata->task_card_set[i]->controller->controller_type == basic_controller::PID ||
                      robotdata->task_card_set[i]->controller->controller_type == basic_controller::PID_orient ||
                      robotdata->task_card_set[i]->controller->controller_type == basic_controller::PID_joint) {
                    if (_task_array_object_object.key() == "Kp_1") {
                      robotdata->task_card_set[i]->controller->para(0, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kp_2") {
                      robotdata->task_card_set[i]->controller->para(1, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kp_3") {
                      robotdata->task_card_set[i]->controller->para(2, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kp_4") {
                      robotdata->task_card_set[i]->controller->para(3, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kp_5") {
                      robotdata->task_card_set[i]->controller->para(4, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kp_6") {
                      robotdata->task_card_set[i]->controller->para(5, 0) =
                          _task_array_object_object.value().toDouble();
                    }

                    if (_task_array_object_object.key() == "Kd_1") {
                      robotdata->task_card_set[i]->controller->para(6, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kd_2") {
                      robotdata->task_card_set[i]->controller->para(7, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kd_3") {
                      robotdata->task_card_set[i]->controller->para(8, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kd_4") {
                      robotdata->task_card_set[i]->controller->para(9, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kd_5") {
                      robotdata->task_card_set[i]->controller->para(10, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                    if (_task_array_object_object.key() == "Kd_6") {
                      robotdata->task_card_set[i]->controller->para(11, 0) =
                          _task_array_object_object.value().toDouble();
                    }
                  }

                  _task_array_object_object++;
                }

                // task sensor id
                robotdata->task_card_set[i]->sensor_id = 0;
              }
              _task_array_object++;
            }
          }
        }

        else if (it.key() == "imu_x_offset") {
          robotdata->imu_acc_offset[0] = it.value().toDouble();
        } else if (it.key() == "imu_y_offset") {
          robotdata->imu_acc_offset[1] = it.value().toDouble();
        } else if (it.key() == "imu_z_offset") {
          robotdata->imu_acc_offset[2] = it.value().toDouble();
        }

        it++;
      }
      std::vector<Task*>::iterator _iter;
      // update task priority
      // reset the min priority to 1

      int _min = (*robotdata->task_card_set.begin())->priority;
      for (_iter = robotdata->task_card_set.begin(); _iter != robotdata->task_card_set.end(); _iter++) {
        if (_min > (*_iter)->priority) {
          _min = (*_iter)->priority;
        }
      }
      if (_min < 1) {
        std::cout << "priority setting wrong!" << std::endl;
      }
      for (_iter = robotdata->task_card_set.begin(); _iter != robotdata->task_card_set.end(); _iter++) {
        (*_iter)->priority = (*_iter)->priority - (_min - 1);
      }
      // from 1 ++ iterate compute _npriority
      // count label
      int pre_count = 0;
      int _count = 0;
      int prio = 1;
      while (_count != (robotdata->ntask)) {
        pre_count = _count;
        for (_iter = robotdata->task_card_set.begin(); _iter != robotdata->task_card_set.end(); _iter++) {
          if ((*_iter)->priority == prio) {
            _count++;
          }
        }
        while (_count == pre_count) {
          for (_iter = robotdata->task_card_set.begin(); _iter != robotdata->task_card_set.end(); _iter++) {
            if ((*_iter)->priority > prio) {
              (*_iter)->priority = (*_iter)->priority - 1;
            }
          }
          for (_iter = robotdata->task_card_set.begin(); _iter != robotdata->task_card_set.end(); _iter++) {
            if ((*_iter)->priority == prio) {
              _count++;
            }
          }
        }
        prio++;
      }
      _npriority = prio - 1;
      robotdata->npriority = _npriority;
      // for temp

      robotdata->contactforce.resize(12, 1);
      robotdata->contactforce.setZero();

      robotdata->mpcforceref.resize(12, 1);
      robotdata->mpcforceref.setZero();
      robotdata->imu_init.setIdentity();
      robotdata->WF1 = Eigen::MatrixXd::Identity(12, 12);
      robotdata->WF2 = Eigen::MatrixXd::Identity(12, 12);
      // std::cout<<"!"<<std::endl;
      // read  wbic qp weights
      it = object.begin();
      while (it != object.end()) {
        if (it.key() == "wbcType") {
          if (it.value().toString() == "WQP") {
            robotdata->wbcsolver = Wbc_Solver_type::WQP;
          } else if (it.value().toString() == "WBIC") {
            robotdata->wbcsolver = Wbc_Solver_type::WBIC;
          } else {
            robotdata->wbcsolver = Wbc_Solver_type::WBIC;
            std::cout << "Default WBC has been set" << std::endl;
          }
        }
        if (it.key() == "weightForce") {
          for (int i = 0; i < 12; i++) {
            robotdata->WF1(i, i) = it.value().toArray().at(i).toDouble();
          }
        }
        if (it.key() == "weightDeltaForce") {
          for (int i = 0; i < 12; i++) {
            robotdata->WF2(i, i) = it.value().toArray().at(i).toDouble();
          }
        }
        it++;
      }
    }
  }
}
