#include <utility>

#include "../include/MomentumController.h"

MomentumController::MomentumController() {
  input_data_a.setZero(4, 6);
  input_data_d.setZero(4, 6);
  output_data.setZero(4, 6);
  para_.setZero(12);
  IG_.setZero(6, 6);
}

void MomentumController::setInputData(Eigen::MatrixXd input_a,
                                      Eigen::MatrixXd input_d,
                                      const Eigen::MatrixXd& para,
                                      Eigen::MatrixXd IG) {
  input_data_a = std::move(input_a);
  input_data_d = std::move(input_d);
  para_ = para;
  IG_ = std::move(IG);
}

void MomentumController::controllerRun(Robot_Data* robotdata) {
  Eigen::Vector2d rightFootPos, leftFootPos, rightFootZMP, leftFootZMP,
      compositeZMP, footCenter;
  leftFootPos = robotdata->task_card_set[robotdata->left_foot_id]
                    ->X_a.block(0, 3, 1, 2)
                    .transpose();
  rightFootPos = robotdata->task_card_set[robotdata->right_foot_id]
                     ->X_a.block(0, 3, 1, 2)
                     .transpose();
  double d = -robotdata->task_card_set[robotdata->left_foot_id]->T_offset(2, 3);
  leftFootZMP << (-robotdata->grf[1] - robotdata->grf[3] * d) /
                     robotdata->grf[5],
      (robotdata->grf[0] - robotdata->grf[4] * d) / robotdata->grf[5];
  rightFootZMP << (-robotdata->grf[7] - robotdata->grf[9] * d) /
                      robotdata->grf[11],
      (robotdata->grf[6] - robotdata->grf[10] * d) / robotdata->grf[11];
  if (robotdata->inSingleStand) {
    footCenter = rightFootPos;
    footCenter(0) += robotdata->com_x_offset;
    compositeZMP = rightFootZMP;
    ZMPThreshold << 0.01, 0.01;
  } else {
    footCenter = 0.5 * (robotdata->task_card_set[robotdata->left_foot_id]
                            ->X_a.block(0, 3, 1, 2)
                            .transpose() +
                        robotdata->task_card_set[robotdata->right_foot_id]
                            ->X_a.block(0, 3, 1, 2)
                            .transpose());
    footCenter(0) += robotdata->com_x_offset;
    compositeZMP = ((leftFootZMP + leftFootPos) * robotdata->grf[5] +
                    (rightFootZMP + rightFootPos) * robotdata->grf[11]) /
                   (robotdata->grf[5] + robotdata->grf[11]);
    ZMPThreshold << 0.02, 0.02;
  }

  // std::cout << "rightFootZMP: "  << rightFootZMP << std::endl;
  // std::cout << "leftFootZMP: "  << leftFootZMP << std::endl;
  // std::cout << "compositeZMP: " << compositeZMP << std::endl;
  // std::cout << "footCenter: " << footCenter << std::endl;

  Eigen::Vector2d chestPoseAct =
      robotdata->task_card_set[robotdata->chest_task_id]
          ->X_a.block(0, 0, 1, 2)
          .transpose();
  Eigen::Vector2d chestPoseActDot =
      robotdata->task_card_set[robotdata->chest_task_id]
          ->X_a.block(1, 0, 1, 2)
          .transpose();

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
  output_data(2, 3) =
      input_data_d(2, 3) +
      para_(9) * (input_data_d(1, 3) - input_data_a(1, 3)) +
      IG_(5, 5) * para_(3) * (input_data_d(0, 3) - input_data_a(0, 3));
  output_data(2, 4) =
      input_data_d(2, 4) +
      para_(10) * (input_data_d(1, 4) - input_data_a(1, 4)) +
      IG_(5, 5) * para_(4) * (input_data_d(0, 4) - input_data_a(0, 4));
  output_data(2, 5) =
      input_data_d(2, 5) +
      para_(11) * (input_data_d(1, 5) - input_data_a(1, 5)) +
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
  /*
  1.测试Z2S 到 Stand 的过程出发原地恢复   -OK
  2.测试3s内拉动不超过行走恢复            -
  3.测试3s内拉动不超过行走恢复并保持5s     -
  5.测试3s内拉动超过行走恢复              -
  --------------------------------------------
  6.测试3s后拉动不超过行走恢复            -
  7.测试3s后拉动不超过行走恢复并保持5s     -
  8.测试3s后拉动超过行走恢复              -
  */
  if (robotdata->momentumTurnOn) {
    if (inPrePushPhase) {
      prePushTimer += robotdata->dt;
      // 跳过inReflexPhase，直接进入inRecoveryPhase
      if ((fabs(compositeZMP[0] - footCenter[0]) > ZMPThreshold[0] ||
           fabs(compositeZMP[1] - footCenter[1]) > ZMPThreshold[1]) &&
          chestPoseAct.norm() > 0.06 && chestPoseActDot.norm() > 0.06 &&
          prePushTimer > robotdata->PushWalkRecoveryDelayTime) {
        std::cout << "In RecoveryPhase:" << "\t"
                  << fabs(compositeZMP[0] - footCenter[0]) << "\t>"
                  << ZMPThreshold[0] << "\t"
                  << fabs(compositeZMP[1] - footCenter[1]) << "\t>"
                  << ZMPThreshold[1] << "\t" << chestPoseAct.norm() << "\t>"
                  << 0.06 << "\t" << chestPoseActDot.norm() << "\t>" << 0.06
                  << "\t" << "time:" << prePushTimer << std::endl;
        prePushTimer = 0.0;
        inPrePushPhase = false;
        inRecoveryPhase = true;
        ZMPDes.setZero();
        reflexTimer = 0.0;
        q_factor_reflex = robotdata->q_factor;
        q_dot_factor_reflex = robotdata->q_dot_factor;
      }
    } else if (inReflexPhase) {
      reflexTimer += robotdata->dt;
      robotdata->task_card_set[robotdata->body_task_id]->weight << 50., 50.,
          50., 1., 1., 1.;
      robotdata->task_card_set[robotdata->com_task_id]->weight << 20., 20., 20.,
          200., 200., 100.;
      robotdata->task_card_set[robotdata->upper_joints_id]->weight
          << 10. * Eigen::VectorXd::Ones(3, 1),
          50. * Eigen::VectorXd::Ones(8, 1);

      Eigen::Vector3d ZMPdotDes, CoMOffset, rDes, fDes;
      ZMPdotDes << 0.1 * (footCenter[0] - compositeZMP[0]),
          0.05 * (footCenter[1] - compositeZMP[1]), 0.0;
      ZMPDes += ZMPdotDes * robotdata->dt;
      CoMOffset << footCenter, 0;
      rDes = ZMPDes + CoMOffset - input_data_a.block(0, 3, 1, 3).transpose();
      fDes = output_data.row(2).segment(3, 3).transpose() -
             robotdata->G.segment(3, 3);
      // std::cout << "ZMPDes: " << ZMPDes << std::endl;
      // std::cout << "rDes: " << rDes << std::endl;
      // std::cout << "fDes: " << fDes << std::endl;
      double ratio = 0.8;
      // output_data.row(2).segment(0, 3) = ratio *
      // output_data.row(2).segment(0, 3) + (1 - ratio) *
      // rDes.cross(fDes).transpose();

      double deltaT = 0.2;
      q_factor_reflex << 0.01 * Eigen::VectorXd::Ones(12),
          0.01 * Eigen::VectorXd::Ones(3), 0.5 * Eigen::VectorXd::Ones(8);
      q_dot_factor_reflex << 0.01 * Eigen::VectorXd::Ones(12),
          0.01 * Eigen::VectorXd::Ones(3), 0.5 * Eigen::VectorXd::Ones(8);

      if (reflexTimer < deltaT) {
        robotdata->q_factor =
            q_factor_reflex * reflexTimer / deltaT +
            q_factor_prePush * (deltaT - reflexTimer) / deltaT;
        robotdata->q_dot_factor =
            q_dot_factor_reflex * reflexTimer / deltaT +
            q_dot_factor_prePush * (deltaT - reflexTimer) / deltaT;
      } else {
        robotdata->q_factor = q_factor_reflex;
        robotdata->q_dot_factor = q_dot_factor_reflex;
      }

      std::cout << "Reflex  :" << "\t" << reflexTimer << "\t>" << deltaT << "\t"
                << chestPoseActDot.norm() << "\t<" << 0.1 << "\t"
                << chestPoseAct.norm() << "\t>" << 0.4 << std::endl;

      if (reflexTimer > deltaT &&
          (chestPoseActDot.norm() < 0.1 || chestPoseAct.norm() > 0.4)) {
        inReflexPhase = false;
        inRecoveryPhase = true;
        recoveryTimer = 0.0;
        preRecoveryPose = chestPoseAct.norm();
        // std::cout << "inRecoveryPhase" << std::endl;
      }
    } else if (inRecoveryPhase) {
      recoveryTimer += robotdata->dt;
      robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
          100., 1., 1., 5.;
      robotdata->task_card_set[robotdata->com_task_id]->weight << 10., 10., 10.,
          200., 200., 100.;
      robotdata->task_card_set[robotdata->upper_joints_id]->weight
          << 50. * Eigen::VectorXd::Ones(3, 1),
          100. * Eigen::VectorXd::Ones(8, 1);

      double deltaT = 0.2;  // preRecoveryPose * 1;
      Eigen::VectorXd factor_recovery = Eigen::VectorXd::Zero(23);
      factor_recovery << 0.05 * Eigen::VectorXd::Ones(12),
          0.1 * Eigen::VectorXd::Ones(3), 0.5 * Eigen::VectorXd::Ones(8);

      if (recoveryTimer < deltaT) {
        robotdata->q_factor =
            factor_recovery * recoveryTimer / deltaT +
            q_factor_reflex * (deltaT - recoveryTimer) / deltaT;
        robotdata->q_dot_factor =
            factor_recovery * recoveryTimer / deltaT +
            q_dot_factor_reflex * (deltaT - recoveryTimer) / deltaT;
      } else {
        robotdata->q_factor = factor_recovery;
        robotdata->q_dot_factor = factor_recovery;
      }

      std::cout << "Recovery:" << "\t" << fabs(compositeZMP[0] - footCenter[0])
                << "\t<" << ZMPThreshold[0] << "\t"
                << fabs(compositeZMP[1] - footCenter[1]) << "\t<"
                << ZMPThreshold[1] << "\t" << chestPoseAct.norm() << "\t<"
                << 0.04 << "\t" << chestPoseActDot.norm() << "\t<" << 0.04
                << "\t" << "time:>" << recoveryTimer << std::endl;
      // 退出inRecoveryPhase
      if (fabs(compositeZMP[0] - footCenter[0]) < ZMPThreshold[0] &&
          fabs(compositeZMP[1] - footCenter[1]) < ZMPThreshold[1] &&
          chestPoseAct.norm() < 0.04 && chestPoseActDot.norm() < 0.04 &&
          recoveryTimer > deltaT) {
        inRecoveryPhase = false;
        inPrePushPhase = true;
        recoveryTimer = 0.0;
        std::cout << "Out RecoveryPhase" << std::endl;
      }
      // 进入inWalkRecoveryPhase
      else if ((fabs(compositeZMP[0] - footCenter[0]) > ZMPThreshold[0] ||
                fabs(compositeZMP[1] - footCenter[1]) > ZMPThreshold[1]) &&
               chestPoseAct.norm() > 0.1 && recoveryTimer > deltaT) {
        inRecoveryPhase = false;
        inWalkRecoveryPhase = true;
        recoveryTimer = 0.0;

        q_factor_recovery = factor_recovery;
        q_dot_factor_recovery = factor_recovery;
        std::cout << "IN WalkRecoveryPhase" << std::endl;
      }
    } else if (inWalkRecoveryPhase) {
      walkRecoveryTimer += robotdata->dt;

      robotdata->task_card_set[robotdata->body_task_id]->weight << 100., 100.,
          100., 10., 10., 10.;
      robotdata->task_card_set[robotdata->com_task_id]->weight << 20., 20., 20.,
          100., 100., 100.;
      robotdata->task_card_set[robotdata->upper_joints_id]->weight =
          100.0 * Eigen::VectorXd::Ones(11);

      double deltaT = 0.0;
      Eigen::VectorXd factor_walk_recovery = Eigen::VectorXd::Zero(23);
      factor_walk_recovery << 0.1 * Eigen::VectorXd::Ones(12),
          0.5 * Eigen::VectorXd::Ones(11);

      if (walkRecoveryTimer < deltaT) {
        robotdata->q_factor =
            factor_walk_recovery * walkRecoveryTimer / deltaT +
            q_factor_recovery * (deltaT - walkRecoveryTimer) / deltaT;
        robotdata->q_dot_factor =
            factor_walk_recovery * walkRecoveryTimer / deltaT +
            q_dot_factor_recovery * (deltaT - walkRecoveryTimer) / deltaT;
      } else {
        robotdata->PushWalkRecoveryFlag = true;
        inPrePushPhase = true;
        inWalkRecoveryPhase = false;
        walkRecoveryTimer = 0.0;
        robotdata->q_factor = factor_walk_recovery;
        robotdata->q_dot_factor = factor_walk_recovery;
        std::cout << "In S2W" << std::endl;
      }
    }
  }

  robotdata->temp.segment(0, 2) = compositeZMP;
  robotdata->temp(2) = inPrePushPhase;
  robotdata->temp(3) = inReflexPhase;
  // robotdata->temp.segment(4, 2) = ZMPDes.segment(0, 2);
  robotdata->temp(4) = chestPoseAct.norm();
  robotdata->temp(5) = chestPoseActDot.norm();
}

void MomentumController::getOutputData(Eigen::MatrixXd& output) {
  output = output_data;
}