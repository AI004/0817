#include <utility>

#include "../include/MomentumController.h"

MomentumController::MomentumController() {
  input_data_a.setZero(4, 6);
  input_data_d.setZero(4, 6);
  output_data.setZero(4, 6);
  para_.setZero(12);
  IG_.setZero(6, 6);

  inReflexPhase_ = false;
  inRecoveryPhase_ = true;
  inWalkRecoveryPhase_ = false;
}

void MomentumController::setInputData(Eigen::MatrixXd input_a, Eigen::MatrixXd input_d, const Eigen::MatrixXd& para,
                                      Eigen::MatrixXd IG) {
  input_data_a = std::move(input_a);
  input_data_d = std::move(input_d);
  para_ = para;
  IG_ = std::move(IG);
}

void MomentumController::controllerRun(Robot_Data* robotdata) {
  Eigen::Vector2d rightFootPos, leftFootPos, rightFootZMP, leftFootZMP, compositeZMP, footCenter;
  leftFootPos = robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 2).transpose();
  rightFootPos = robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 2).transpose();
  // 脚踝与地面的距离
  double d = -robotdata->task_card_set[robotdata->left_foot_id]->T_offset(2, 3);
  // 左右脚zmp
  leftFootZMP << (-robotdata->grf[1] - robotdata->grf[3] * d) / robotdata->grf[5],
      (robotdata->grf[0] - robotdata->grf[4] * d) / robotdata->grf[5];
  rightFootZMP << (-robotdata->grf[7] - robotdata->grf[9] * d) / robotdata->grf[11],
      (robotdata->grf[6] - robotdata->grf[10] * d) / robotdata->grf[11];
  // 单足站立默认右足，可更改为左右足均可。单足支撑footCenter位于支撑脚点中，双足支撑位于两脚正中。
  if (robotdata->inSingleStand) {
    footCenter = rightFootPos;
    footCenter(0) += robotdata->com_x_offset;
    compositeZMP = rightFootZMP;
    ZMPThreshold << 0.01, 0.01;
  } else {
    footCenter = 0.5 * (robotdata->task_card_set[robotdata->left_foot_id]->X_a.block(0, 3, 1, 2).transpose() +
                        robotdata->task_card_set[robotdata->right_foot_id]->X_a.block(0, 3, 1, 2).transpose());
    footCenter(0) += robotdata->com_x_offset;
    compositeZMP =
        ((leftFootZMP + leftFootPos) * robotdata->grf[5] + (rightFootZMP + rightFootPos) * robotdata->grf[11]) /
        (robotdata->grf[5] + robotdata->grf[11]);
    ZMPThreshold << 0.02, 0.02;
  }

  Eigen::Vector2d chestPoseAct = robotdata->task_card_set[robotdata->chest_task_id]->X_a.block(0, 0, 1, 2).transpose();
  Eigen::Vector2d chestPoseActDot =
      robotdata->task_card_set[robotdata->chest_task_id]->X_a.block(1, 0, 1, 2).transpose();

  // lower CoM height when disturbed
  if (robotdata->momentumTurnOn) {
    if (inReflexPhase || inRecoveryPhase) {
      input_data_d(0, 5) -= 0.1 * chestPoseAct.norm();
    }
  }

  // linear momentum control
  // x_
  output_data(0, 3) = input_data_d(0, 3);
  output_data(0, 4) = input_data_d(0, 4);
  output_data(0, 5) = input_data_d(0, 5);
  // x_dot
  output_data(1, 3) = input_data_d(1, 3);
  output_data(1, 4) = input_data_d(1, 4);
  output_data(1, 5) = input_data_d(1, 5);
  // x_ddot
  output_data(2, 3) = input_data_d(2, 3) + para_(9) * (input_data_d(1, 3) - input_data_a(1, 3)) +
                      IG_(5, 5) * para_(3) * (input_data_d(0, 3) - input_data_a(0, 3));
  output_data(2, 4) = input_data_d(2, 4) + para_(10) * (input_data_d(1, 4) - input_data_a(1, 4)) +
                      IG_(5, 5) * para_(4) * (input_data_d(0, 4) - input_data_a(0, 4));
  output_data(2, 5) = input_data_d(2, 5) + para_(11) * (input_data_d(1, 5) - input_data_a(1, 5)) +
                      IG_(5, 5) * para_(5) * (input_data_d(0, 5) - input_data_a(0, 5));

  // angular momentum control
  // x_
  output_data.block(0, 0, 1, 3).setZero();
  // x_dot
  output_data(1, 0) = input_data_d(1, 0);
  output_data(1, 1) = input_data_d(1, 1);
  output_data(1, 2) = input_data_d(1, 2);
  // x_ddot
  output_data(2, 0) = para_(6) * (input_data_d(1, 0) - input_data_a(1, 0));
  output_data(2, 1) = para_(7) * (input_data_d(1, 1) - input_data_a(1, 1));
  output_data(2, 2) = para_(8) * (input_data_d(1, 2) - input_data_a(1, 2));

  if (robotdata->momentumTurnOn) {
    if (inRecoveryPhase_) {
      /**************************sim ver*****************************/
      // com momentum x
      // if (fabs(robotdata->comMomentum(0)) > 2.5) {
      //   inRecoveryPhase_ = false;
      //   inWalkRecoveryPhase_ = true;
      //   if (robotdata->comMomentum(0) > 0) {
      //     robotdata->stance_index = 0;
      //     std::cerr << "Suffering from Left side impact, in walk recovery phase!" << std::endl;
      //   } else {
      //     robotdata->stance_index = 1;
      //     std::cerr << "Suffering from Right side impact, in walk recovery phase!" << std::endl;
      //   }
      //   robotdata->impactFactor(1) = 5;
      // } else if (fabs(robotdata->comMomentum(0)) >= 2.0 && fabs(robotdata->comMomentum(0)) <= 2.5) {
      //   robotdata->impactFactor(1) = 3;
      // } else if (fabs(robotdata->comMomentum(0)) > 1.5 && fabs(robotdata->comMomentum(0)) <= 2.0) {
      //   robotdata->impactFactor(1) = 1;
      // } else {
      //   robotdata->impactFactor(1) = 0;
      // }
      // com momentum y
      // if use com q_dot_a, vel threshold is 0.2
      // if (fabs(robotdata->comMomentum(1)) > 0.9) {
      //   inRecoveryPhase_ = false;
      //   inWalkRecoveryPhase_ = true;
      //   if (robotdata->comMomentum(1) > 0) {
      //     std::cerr << "Suffering from rear side impact, in walk recovery phase!" << std::endl;
      //   } else {
      //     std::cerr << "Suffering from front side impact, in walk recovery phase!" << std::endl;
      //   }
      //   if (robotdata->comMomentum(0) > 0) {
      //     robotdata->stance_index = 0;
      //   } else {
      //     robotdata->stance_index = 1;
      //   }
      //   robotdata->impactFactor(0) = 2;
      // } else if (fabs(robotdata->comMomentum(1)) >= 0.6 && fabs(robotdata->comMomentum(1)) <= 0.9) {
      //   robotdata->impactFactor(0) = 1;
      // } else {
      //   robotdata->impactFactor(0) = 0;
      // }

      /**************************real ver*****************************/
      // com momentum x
      if (fabs(robotdata->comMomentum(0)) > 1.0) {
        inRecoveryPhase_ = false;
        inWalkRecoveryPhase_ = true;
        if (robotdata->comMomentum(0) > 0) {
          robotdata->stance_index = 0;
          std::cerr << "Left side impact, in walk recovery phase!" << std::endl;
        } else {
          robotdata->stance_index = 1;
          std::cerr << "Right side impact, in walk recovery phase!" << std::endl;
        }
        robotdata->impactFactor(0) = 0.1;
      } else if (fabs(robotdata->comMomentum(0)) >= 0.6 && fabs(robotdata->comMomentum(0)) <= 1.0) {
        robotdata->impactFactor(0) = 0.05;
      } else {
        robotdata->impactFactor(0) = 0.;
      }
      // com momentum y
      // if use com q_dot_a, vel threshold is 0.2
      if (fabs(robotdata->comMomentum(1)) > 0.9) {
        inRecoveryPhase_ = false;
        inWalkRecoveryPhase_ = true;
        if (robotdata->comMomentum(1) > 0) {
          std::cerr << "Rear side impact, in walk recovery phase!" << std::endl;
        } else {
          std::cerr << "Front side impact, in walk recovery phase!" << std::endl;
        }
        if (robotdata->comMomentum(0) > 0) {
          robotdata->stance_index = 0;
        } else {
          robotdata->stance_index = 1;
        }
        robotdata->impactFactor(1) = 0.5;
      } else if (fabs(robotdata->comMomentum(1)) >= 0.6 && fabs(robotdata->comMomentum(1)) <= 0.9) {
        robotdata->impactFactor(1) = 0.3;
      } else {
        robotdata->impactFactor(1) = 0;
      }
    }

    else if (inWalkRecoveryPhase_) {
      robotdata->PushWalkRecoveryFlag = true;
      inRecoveryPhase_ = true;
    }
  }

  robotdata->temp.segment(0, 2) = compositeZMP;
  robotdata->temp(2) = inPrePushPhase;
  robotdata->temp(3) = inReflexPhase;
  // robotdata->temp.segment(4, 2) = ZMPDes.segment(0, 2);
  robotdata->temp(4) = chestPoseAct.norm();
  robotdata->temp(5) = chestPoseActDot.norm();
}

void MomentumController::getOutputData(Eigen::MatrixXd& output) { output = output_data; }