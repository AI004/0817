/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <random>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

void rotationMatrixToXYZEulerAngles(Eigen::Matrix3d R, Eigen::Vector3d& euler) {
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

int main(int argc, char* argv[]) {
  rbdl_check_api_version(RBDL_API_VERSION);

  Model* model = NULL;

  // 从指定文件目录中读取 URDF 模型
  std::string urdf_file = "../../../Sources/urdf/DuckDuck.urdf";  // 替换为你的 URDF 文件路径
  bool floating_base = false;                                     // 是否使用浮动基座

  model = new Model();

  if (!Addons::URDFReadFromFile(urdf_file.c_str(), model, floating_base)) {
    std::cerr << "Error loading URDF model from file: " << urdf_file << std::endl;
    delete model;
    return -1;
  }

  std::cout << "model->dof_count: " << model->dof_count << std::endl;
  std::cout << "model->mBodies.size(): " << model->mBodies.size() << std::endl;

  model->gravity = Vector3d(0., 0., -9.81);
  std::cout << "model->gravity: " << model->gravity.transpose() << std::endl;

  // 打印所有关节名称
  std::cout << "Joints:" << std::endl;
  for (unsigned int i = 0; i < model->mJoints.size(); ++i) {
    std::cout << "  " << model->GetBodyName(i) << std::endl;
  }

  // 打印所有 body 名称和对应的 body ID
  std::cout << "Bodies:" << std::endl;
  for (unsigned int i = 0; i < model->mBodies.size(); ++i) {
    std::cout << "  Body ID: " << i << ", Name: " << model->GetBodyName(i) << std::endl;
  }

  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd Tau = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);

  // 计算前向动力学
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  std::cout << "Q: " << Q.transpose() << std::endl;
  std::cout << "QDot: " << QDot.transpose() << std::endl;
  std::cout << "Tau: " << Tau.transpose() << std::endl;
  std::cout << "QDDot: " << QDDot.transpose() << std::endl;

  // 计算逆动力学
  InverseDynamics(*model, Q, QDot, QDDot, Tau);

  std::cout << "Q: " << Q.transpose() << std::endl;
  std::cout << "QDot: " << QDot.transpose() << std::endl;
  std::cout << "Tau: " << Tau.transpose() << std::endl;
  std::cout << "QDDot: " << QDDot.transpose() << std::endl;

  // 计算正运动学
  UpdateKinematics(*model, Q, QDot, QDDot);

  // 计算某个关节相对于基座的变换矩阵
  MatrixNd T = MatrixNd::Identity(4, 4);
  // Body ID: 8, Name: left_ankle_roll
  // Body ID: 11, Name: neck_yaw
  // Body ID: 17, Name: right_ankle_roll
  unsigned int body_id = 8;
  SpatialTransform body_transform = model->X_base[body_id];
  std::cout << "Body Transform (Base to Body " << body_id << "):" << std::endl;
  T.block(0, 0, 3, 3) = body_transform.E;
  T.block(0, 3, 3, 1) = body_transform.r;
  std::cout << T << std::endl;

  // 计算雅可比矩阵
  MatrixNd J_left_foot = MatrixNd::Zero(6, model->qdot_size);
  body_id = 8;
  CalcPointJacobian6D(*model, Q, body_id, Vector3d(0., 0., 0.), J_left_foot, false);
  std::cout << "Jacobian Matrix (Body " << body_id << "):" << std::endl;
  std::cout << J_left_foot << std::endl;

  MatrixNd J_right_foot = MatrixNd::Zero(6, model->qdot_size);
  body_id = 17;
  CalcPointJacobian6D(*model, Q, body_id, Vector3d(0., 0., 0.), J_right_foot, false);
  std::cout << "Jacobian Matrix (Body " << body_id << "):" << std::endl;
  std::cout << J_right_foot << std::endl;

  MatrixNd J_head = MatrixNd::Zero(3, model->qdot_size);
  body_id = 11;
  CalcPointJacobian6D(*model, Q, body_id, Vector3d(0., 0., 0.), J_head, false);
  std::cout << "Jacobian Matrix (Body " << body_id << "):" << std::endl;
  std::cout << J_head << std::endl;

  // 计算逆运动学（简单示例，实际应用中需要更复杂的优化方法）
  std::cout << "model->q_size: " << model->q_size << std::endl;
  VectorNd Q_target = VectorNd::Zero(model->q_size);
  VectorNd Q_current = VectorNd::Zero(model->q_size);
  Q_current << 0, 0, 0, 0, 0, 0, 0.1, 0.1, -0.4, 0.4, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  VectorNd Q_dot = VectorNd::Zero(model->qdot_size);
  VectorNd Q_ddot = VectorNd::Zero(model->qdot_size);
  VectorNd Tau_inv = VectorNd::Zero(model->qdot_size);

  // 目标末端执行器位置，对应的左腿关节位置[0.15, 0.15, -0.45, 0.5, 0.15, 0.15]
  Vector3d target_position(0.0221, 0.1849, -0.2997);
  Vector3d target_orientation(0.2709, 0.2381, 0.1171);

  auto start = std::chrono::high_resolution_clock::now();
  // 迭代求解逆运动学
  MatrixNd J;
  body_id = 8;
  VectorNd error = VectorNd::Zero(6);

  // 计算关节速度
  double alpha = 1.0;                    // 学习率
  double lambda = 0.01;                  // 阻尼因子
  VectorNd weights = VectorNd::Zero(6);  // 定义权重向量
  weights << 1, 1, 1, 1, 1, 1;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W = weights.asDiagonal();  // 构建权重矩阵
  std::random_device rd;                                                   // 随机数生成器
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 0);                // 均值为0，标准差为0的正态分布
  Eigen::VectorXd noise = Eigen::VectorXd::Zero(6);  // 随机扰动
  MatrixNd I66 = MatrixNd::Identity(6, 6);
  VectorNd q_dot = VectorNd::Zero(6);
  Vector3d current_orientation = Eigen::Vector3d::Zero();
  Vector3d current_position = Eigen::Vector3d::Zero();
  Matrix3d current_orientation_matrix = MatrixNd::Zero(3, 3);
  int meth_select = 1;  // 三种方法计算速度依次增加

  for (int i = 0; i < 50; ++i) {
    UpdateKinematics(*model, Q_current, Q_dot, Q_ddot);
    current_position = CalcBodyToBaseCoordinates(*model, Q_current, body_id, Vector3d(0., 0., 0.));
    current_orientation_matrix =
        CalcBodyWorldOrientation(*model, Q_current, body_id).transpose();  // 需要转置后与MatLab中计算结果一致
    // std::cout << "current_orientation_matrix: " << current_orientation_matrix << std::endl;

    rotationMatrixToXYZEulerAngles(current_orientation_matrix.block(0, 0, 3, 3),
                                   current_orientation);  // 根据雅可比矩阵顺序，需要将旋转矩阵转换为 'XYZ' 欧拉角格式

    // std::cout << current_orientation.transpose() << std::endl;

    error.tail(3) = target_position - current_position;
    error.head(3) = target_orientation - current_orientation;

    std::cout << "error: " << error.norm() << "\t" << error.transpose() << std::endl;

    if (error.norm() < 1e-6) {
      std::cout << "Inverse Kinematics Converged in " << i << " iterations." << std::endl;
      break;
    }

    // 计算雅可比矩阵
    J.setZero(6, model->qdot_size);
    CalcPointJacobian6D(*model, Q_current, body_id, Vector3d(0., 0., 0.), J, true);  // ROLL PITCH YAW X Y Z

    J_left_foot = J.block(0, 6, 6, 6);

    switch (meth_select) {
      case 1:
        q_dot = J_left_foot.transpose() * (J_left_foot * J_left_foot.transpose()).inverse() * error;  // 雅可比转置法
        // std::cout << "雅可比转置法 q_dot: " << q_dot.transpose() << std::endl;
        break;
      case 2:
        q_dot = J_left_foot.completeOrthogonalDecomposition().pseudoInverse() *
                error;  // completeOrthogonalDecomposition()伪逆法
        // std::cout << "completeOrthogonalDecomposition()伪逆法 q_dot: " << q_dot.transpose() << std::endl;
        break;
      case 3:
        for (int i = 0; i < 3; ++i) {
          noise(i) = d(gen);
        }
        error += noise;
        q_dot = (J_left_foot.transpose() * W * J_left_foot + lambda * lambda * I66)
                    .colPivHouseholderQr()
                    .solve(J_left_foot.transpose() * W * error);  // 基于随机扰动的权重阻尼最小二乘法
        // std::cout << "阻尼最小二乘法 q_dot: " << q_dot.transpose() << std::endl;
        break;
      default:
        std::cout << "无效的选择" << std::endl;
        break;
    }

    // 更新关节角度
    Q_current.block(6, 0, 6, 1) += q_dot * alpha;
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "Function took " << elapsed.count() << " milliseconds\n";

  std::cout << "Inverse Kinematics Result: " << Q_current.block(6, 0, 6, 1).transpose() << std::endl;

  delete model;

  return 0;
}