#include "../include/wbc_solver.h"
// #include "wbc_sovler_hqp.h"
#include <iostream>
#include "broccoli/core/Time.hpp"
#include <iostream>
#include "public_parament.h"
#define DEBUG
WbcSolver::WbcSolver() {}

WbcSolver::~WbcSolver() {}

// init
void WbcSolver::Init(Robot_Data* robotdata) {
  priority_num = robotdata->npriority;
  task_num = robotdata->ntask;
  dt = robotdata->dt;
  priorityorder = new PriorityOrder[priority_num];
  count = 5;
  InitDynamics(robotdata);
  wbc = new QPproblem();

  weight_factor = 1.e2;

  deltX.resize(18, 1);
  deltX.setZero();
  tau_lb = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  tau_ub = Eigen::VectorXd::Zero(robotdata->ndof - 6);
  GRF_ub = Eigen::VectorXd::Zero(12);
  Fref = Eigen::VectorXd::Zero(12);

  int nv = actor_num + 6 + 12;       // 71
  int nc = 6 + 28 + 12 + actor_num;  // 99
  wbc->init(nv, nc, 2 * dt, 2000);
  wbc->qp_option.printLevel = qpOASES::PL_LOW;
  wbc->qp_option.terminationTolerance = 0.0001;  // 0.000001;
  wbc->qp_option.enableRegularisation = qpOASES::BT_FALSE;
  // wbc->qp_option.epsRegularisation = 0.00001;// 这个参数需要自行调节一下
  wbc->qp_problem->setOptions(wbc->qp_option);

#ifdef PROXQP
  wqp = new ProxQP();
  dense::isize dim = actor_num + 6 + 12;
  dense::isize n_eq = 0;
  dense::isize n_in = 6 + 28 + 12 + actor_num;
  wqp->init(dim, n_eq, n_in);
#endif
}
// solve and update the joint command
void WbcSolver::SolveWbc(Robot_Data* robotdata) {
  // update dynamics -------------------------------------------------------
  // update dynamics and base constraint
  // std::cout<<"hehe4.5"<<std::endl;
  UpdateDynamics(robotdata);

  // H g
  // std::cout<<"hehe5"<<std::endl;
  wbc->m_hessian.setZero();
  wbc->m_gradient.setZero();
  for (int i = 0; i < robotdata->task_card_set.size(); i++) {
    Eigen::MatrixXd Wi = Eigen::MatrixXd::Zero(robotdata->task_card_set[i]->dim, robotdata->task_card_set[i]->dim);
    for (int j = 0; j < robotdata->task_card_set[i]->dim; j++) {
      Wi(j, j) = robotdata->task_card_set[i]->weight(j) * weight_factor;
    }
    wbc->m_hessian.block(0, 0, ndof, ndof) +=
        robotdata->task_card_set[i]->jacobi.transpose() * Wi * robotdata->task_card_set[i]->jacobi;
    wbc->m_gradient.block(0, 0, ndof, 1) +=
        robotdata->task_card_set[i]->jacobi.transpose() * Wi *
        (robotdata->task_card_set[i]->jacobi_dot_q_dot - robotdata->task_card_set[i]->X_c.row(2).transpose());
  }

  // ||F|| Wf1; ||F - Fref|| Wf2
  wbc->m_hessian.block(ndof, ndof, 12, 12) = robotdata->WF1 * weight_factor + robotdata->WF2 * weight_factor;
  wbc->m_gradient.block(ndof, 0, 12, 1) = -robotdata->WF2 * weight_factor * Fref;
  // constraints
  // std::cout<<"hehe5.5"<<std::endl;
  wbc->m_constraint.setZero();
  wbc->m_lbconstraint.setConstant(-INFINITY_QP);
  wbc->m_ubconstraint.setConstant(INFINITY_QP);
  // std::cout<<"m_constraint"<<wbc->m_constraint.size()<<std::endl;
  // floating base constraints nc += 6
  wbc->m_constraint.block(0, 0, 6, ndof + 12) = dyna.MbJb_T;
  wbc->m_lbconstraint.block(0, 0, 6, 1) = -dyna.Nb;
  wbc->m_ubconstraint.block(0, 0, 6, 1) = -dyna.Nb;
  // std::cout<<"m_constraint"<<wbc->m_constraint<<std::endl;
  // std::cout<<"m_lbconstraint"<<wbc->m_lbconstraint.transpose()<<std::endl;
  // std::cout<<"m_ubconstraint"<<wbc->m_ubconstraint.transpose()<<std::endl;
  // GRF constraints
  // friction cone nc += 28
  wbc->m_constraint.block(6, ndof, 28, 12) = priorityorder[0].A_c;
  // wbc->m_constraint.block(6,18,28,12).setZero();
  wbc->m_ubconstraint.block(6, 0, 28, 1).setZero();
  // GRF nc += 12
  wbc->m_constraint.block(34, ndof, 12, 12).setIdentity();
  wbc->m_ubconstraint.block(34, 0, 12, 1) = GRF_ub;
  wbc->m_lbconstraint.block(34, 0, 12, 1) = GRF_lb;
  // tau constraints nc +=12
  wbc->m_constraint.block(46, 0, actor_num, ndof + 12) = dyna.MaJa_T;
  wbc->m_lbconstraint.block(46, 0, actor_num, 1) = tau_lb - dyna.Na;
  wbc->m_ubconstraint.block(46, 0, actor_num, 1) = tau_ub - dyna.Na;
  wbc->m_lbound.setConstant(-INFINITY_QP);
  wbc->m_ubound.setConstant(INFINITY_QP);
  // broccoli::core::Time timer, start_time, total_time;
  // start_time = timer.currentTime();
  bool solve_flag = wbc->solve();
  // total_time = timer.currentTime() - start_time;
  // std::cout << "qpoasis solve time :" << total_time.m_nanoSeconds/1.e6 <<
  // std::endl;

  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(actor_num + 6 + 12, 1);
  if (solve_flag) {
    X = wbc->getOptimal();
    // std::cout<<"-B_Q_inv*b_Q:
    // "<<std::endl<<(-B_Q_inv*b_Q).transpose()<<std::endl;
  } else {
    std::cout << "wbc qp solve failed!" << std::endl;
  }
  robotdata->q_ddot_c = X.block(0, 0, ndof, 1);
  robotdata->contactforce = X.block(ndof, 0, 12, 1);
  // std::cout<<"X: "<<std::endl<<X.transpose()<<std::endl;
  Eigen::MatrixXd N = robotdata->tau_c;
  N.block(0, 0, 6, 1) = dyna.Nb;
  N.block(6, 0, ndof - 6, 1) = dyna.Na;
  Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(12, ndof);
  Jc.block(0, 0, 6, ndof) = robotdata->task_card_set[robotdata->left_foot_id]->jacobi;
  Jc.block(6, 0, 6, ndof) = robotdata->task_card_set[robotdata->right_foot_id]->jacobi;
  robotdata->tau_c = dyna.M * robotdata->q_ddot_c + N - Jc.transpose() * robotdata->contactforce;

#ifdef PROXQP
  // H g
  for (int i = 0; i < robotdata->task_card_set.size(); i++) {
    Eigen::MatrixXd Wi = Eigen::MatrixXd::Zero(robotdata->task_card_set[i]->dim, robotdata->task_card_set[i]->dim);
    for (int j = 0; j < robotdata->task_card_set[i]->dim; j++) {
      Wi(j, j) = robotdata->task_card_set[i]->weight(j) * weight_factor;
    }
    wqp->H.block(0, 0, ndof, ndof) +=
        robotdata->task_card_set[i]->jacobi.transpose() * Wi * robotdata->task_card_set[i]->jacobi;
    wqp->g.block(0, 0, ndof, 1) +=
        robotdata->task_card_set[i]->jacobi.transpose() * Wi *
        (robotdata->task_card_set[i]->jacobi_dot_q_dot - robotdata->task_card_set[i]->X_c.row(2).transpose());
  }

  // ||F|| Wf1; ||F - Fref|| Wf2
  wqp->H.block(ndof, ndof, 12, 12) = robotdata->WF1 * weight_factor + robotdata->WF2 * weight_factor;
  wqp->g.block(ndof, 0, 12, 1) = -robotdata->WF2 * weight_factor * Fref;
  // // equality constraints
  // // floating base constraints nc += 6
  // wqp->A = dyna.MbJb_T;
  // wqp->b = -dyna.Nb;

  // inequality constraints
  wqp->C.block(0, 0, 6, 41) = dyna.MbJb_T;
  wqp->l.block(0, 0, 6, 1) = -dyna.Nb;
  wqp->u.block(0, 0, 6, 1) = -dyna.Nb;
  // GRF constraints
  // friction cone nc += 28
  wqp->C.block(6, ndof, 28, 12) = priorityorder[0].A_c;
  // wbc->m_constraint.block(6,18,28,12).setZero();
  wqp->u.block(6, 0, 28, 1).setZero();
  wqp->l.block(6, 0, 28, 1) = -1.e20 * Eigen::VectorXd::Ones(28);
  // GRF nc += 12
  wqp->C.block(34, ndof, 12, 12).setIdentity();
  wqp->u.block(34, 0, 12, 1) = GRF_ub;
  wqp->l.block(34, 0, 12, 1) = GRF_lb;
  // tau constraints nc +=12
  wqp->C.block(46, 0, 23, 41) = dyna.MaJa_T;
  wqp->l.block(46, 0, 23, 1) = tau_lb - dyna.Na;
  wqp->u.block(46, 0, 23, 1) = tau_ub - dyna.Na;

  // start_time = timer.currentTime();
  // wqp->solve();
  // total_time = timer.currentTime() - start_time;
  // std::cout << "proxqp solve time :" << total_time.m_nanoSeconds/1.e6 <<
  // std::endl;

  // Eigen::MatrixXd X = Eigen::MatrixXd::Zero(41, 1);
  // if (solve_flag) {
  //   X = wqp->x;
  // } else {
  //   std::cout << "wbc qp solve failed!" << std::endl;
  // }
  // robotdata->q_ddot_c = X.block(0, 0, ndof, 1);
  // robotdata->contactforce = X.block(ndof, 0, 12, 1);
  // // std::cout<<"X: "<<std::endl<<X.transpose()<<std::endl;
  // Eigen::MatrixXd N = robotdata->tau_c;
  // N.block(0, 0, 6, 1) = dyna.Nb;
  // N.block(6, 0, ndof - 6, 1) = dyna.Na;
  // Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(12, ndof);
  // Jc.block(0, 0, 6, ndof) =
  // robotdata->task_card_set[robotdata->left_foot_id]->jacobi; Jc.block(6, 0,
  // 6, ndof) = robotdata->task_card_set[robotdata->right_foot_id]->jacobi;
  // robotdata->tau_c = dyna.M * robotdata->q_ddot_c + N - Jc.transpose() *
  // robotdata->contactforce;
#endif
}
void WbcSolver::SetConstraints(Eigen::VectorXd _tau_lb, Eigen::VectorXd _tau_ub, Eigen::VectorXd _GRF_lb,
                               Eigen::VectorXd _GRF_ub, Eigen::VectorXd _Fref) {
  tau_lb = _tau_lb;
  tau_ub = _tau_ub;
  if ((_GRF_lb.size() == 12) && (_GRF_ub.size() == 12) && (_Fref.size() == 12)) {
    GRF_lb = _GRF_lb;
    GRF_ub = _GRF_ub;
    Fref = _Fref;
  } else {
    std::cout << "GRF constraints or reference not match!" << std::endl;
  }
}
// Float_Base_Open_Chain + hqp_dynamics
void WbcSolver::InitDynamics(Robot_Data* robotdata) {
  Task* pTask_cur = nullptr;
  // priority = 1,2,....
  for (int j = 1; j < priority_num + 1; j++) {
    // save priority information
    PriorityOrder priorityorder_cur;
    priorityorder_cur.priority = j;  // from 1
    for (int i = 0; i < task_num; i++) {
      pTask_cur = robotdata->task_card_set[i];

      if (pTask_cur->priority == j) {
        priorityorder_cur.taskID.push_back(i);

        priorityorder_cur.taskdim.push_back(pTask_cur->dim);
        priorityorder_cur.taskdim_total += pTask_cur->dim;

        if (pTask_cur->type == task_type::contact_task) {
          priorityorder_cur.contact_flag = true;

          priorityorder_cur.contact_id.push_back(i);
          priorityorder_cur.contactdim.push_back(pTask_cur->dim);
          task_num_f += 1;
          task_f_id.push_back(i);
          task_f_dim.push_back(pTask_cur->dim);
          dim_f += pTask_cur->dim;
        }
        if (pTask_cur->type == task_type::com_task) {
          com_task_id = i;
        }
      }
    }
    priorityorder[j - 1] = priorityorder_cur;
  }
  // init friction cone constraints dims  "octagon"
  // controlled dof
  ndof = robotdata->ndof;
  std::cout << "ndof: " << ndof << std::endl;
  // init the base constraint
  LB_base.resize(ndof, 1);
  UB_base.resize(ndof, 1);
  D_base.resize(2 * (ndof), ndof);
  f_base.resize(2 * (ndof), 1);
  LB_base.setZero();
  UB_base.setZero();
  D_base.setZero();
  f_base.setZero();
  // init dim of the prioirty = 0, 1, 2, ..;
  // init qp problems
  for (int i = 0; i < priority_num; i++) {
    // init equality constraints
    priorityorder[i].B_hat.resize(priorityorder[i].taskdim_total, ndof);
    priorityorder[i].b_hat.resize(priorityorder[i].taskdim_total, 1);
    // init inequality constraints
    // priorityorder[i].A_hat.resize(priorityorder[i].constdim_total, ndof);
    // priorityorder[i].a_hat.resize(priorityorder[i].constdim_total, 1);
    // init nullspace
    priorityorder[i].Z.resize(ndof, ndof);
    // init task weight
    priorityorder[i].taskweight.resize(priorityorder[i].taskdim_total, priorityorder[i].taskdim_total);
    // init B_hat_hat_inv

    priorityorder[i].B_hat.setZero();
    priorityorder[i].b_hat.setZero();
    priorityorder[i].A_hat.setZero();
    priorityorder[i].a_hat.setZero();
    priorityorder[i].Z.setZero();
    priorityorder[i].taskweight.setIdentity();
    // init qp oases results
    priorityorder[i].optimal_qpresult.resize(ndof, 1);
    priorityorder[i].y_star.resize(ndof, 1);

    priorityorder[i].optimal_qpresult.setZero();
    priorityorder[i].y_star.setZero();

    // init all QPproblem
    // Variable dim
    int nV = ndof;
    int nC = 0;

    nC = 2 * (ndof) + ndof;

    QPproblem* qp_task_cur = new QPproblem();
    qp_task_cur->init(nV, nC, dt, 2000);
    qp_task.push_back(qp_task_cur);
  }
  // update friction cone constraints
  double miu = 0.4;
  double lamada = 0.1;
  double deltxP = 0.11;
  double deltxN = 0.05;
  double delty = 0.02;

  Eigen::MatrixXd A_c_ = Eigen::MatrixXd::Zero(14, 6);

  GenerateFrictioncone(miu, lamada, deltxP, deltxN, delty, A_c_);

  //    int startrow = 0;
  //    int startcol = 0;
  priorityorder[0].A_c.setZero(28, 12);
  priorityorder[0].A_c.block(0, 0, 14, 6) = A_c_;
  priorityorder[0].A_c.block(14, 6, 14, 6) = A_c_;

  // optimize contact force
  // contact_force.setZero(dim_f,1);
}

// Float_Base_Open_Chain + hqp_dynamics
void WbcSolver::UpdateDynamics(Robot_Data* robotdata) {
  // update dynamics -------------------------------------------------------
  // robotdata->id_body[0],true);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(robotdata->ndof, robotdata->ndof);
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(robotdata->robot_model), robotdata->q_a, H, false);
  dyna.M = H;
  dyna.M_inv = H.completeOrthogonalDecomposition().pseudoInverse();
  dyna.Mb = H.block(0, 0, 6, robotdata->ndof);
  dyna.Ma = H.block(6, 0, robotdata->ndof - 6, robotdata->ndof);
  // std::cout<<"hehe"<<std::endl;
  // dyna.Jb_T
  // dyna.Ja_T
  int total_dim_f_cols = 0;
  for (int i = 0; i < task_num_f; i++) {
    total_dim_f_cols += task_f_dim[i];
  }
  dyna.Jb_T = Eigen::MatrixXd::Zero(6, total_dim_f_cols);
  dyna.Ja_T = Eigen::MatrixXd::Zero(robotdata->ndof - 6, total_dim_f_cols);
  int start_col = 0;
  for (int i = 0; i < task_num_f; i++) {
    int cols = task_f_dim[i];
    dyna.Jb_T.block(0, start_col, 6, cols) =
        -robotdata->task_card_set[task_f_id[i]]->jacobi.transpose().block(0, 0, 6, cols);
    dyna.Ja_T.block(0, start_col, robotdata->ndof - 6, cols) =
        -robotdata->task_card_set[task_f_id[i]]->jacobi.transpose().block(6, 0, robotdata->ndof - 6, cols);
    start_col += cols;
  }
  int totalcols = dyna.Mb.cols() + dyna.Jb_T.cols();
  // std::cout<<"hehe"<<std::endl;
  dyna.MbJb_T = Eigen::MatrixXd::Zero(6, totalcols);
  dyna.MbJb_T.block(0, 0, 6, robotdata->ndof) = dyna.Mb;
  dyna.MbJb_T.block(0, robotdata->ndof, 6, dyna.Jb_T.cols()) = dyna.Jb_T;

  dyna.MaJa_T = Eigen::MatrixXd::Zero(robotdata->ndof - 6, totalcols);
  dyna.MaJa_T.block(0, 0, robotdata->ndof - 6, robotdata->ndof) = dyna.Ma;
  dyna.MaJa_T.block(0, robotdata->ndof, robotdata->ndof - 6, dyna.Ja_T.cols()) = dyna.Ja_T;

  RigidBodyDynamics::Math::VectorNd _tau = RigidBodyDynamics::Math::VectorNd::Zero(robotdata->ndof);
  RigidBodyDynamics::NonlinearEffects(*(robotdata->robot_model), robotdata->q_a, robotdata->q_dot_a, _tau, nullptr);
  dyna.Nb = _tau.block(0, 0, 6, 1);
  dyna.Na = _tau.block(6, 0, robotdata->ndof - 6, 1);
  // std::cout<<"hehe"<<std::endl;
  // update end ------------------------------------------------------------

  // normal joint position and speed limit
  double pos = 0;
  double vel = 0;
  // // floating base
  for (int i = 0; i < 6; i++) {
    LB_base(i, 0) = -INFINITY_QP;
    UB_base(i, 0) = INFINITY_QP;
  }
  // actuator
  for (int i = 6; i < (robotdata->ndof); i++) {
    pos = (robotdata->q_lbound(i, 0) - robotdata->q_a(i, 0) - robotdata->q_dot_a(i, 0) * count * dt) /
          (count * dt * count * dt / 2);
    vel = (-robotdata->qd_bound(i, 0) - robotdata->q_dot_a(i, 0)) / (count * dt);
    LB_base(i, 0) = std::max(pos, vel);

    pos = (robotdata->q_ubound(i, 0) - robotdata->q_a(i, 0) - robotdata->q_dot_a(i, 0) * count * dt) /
          (count * dt * count * dt / 2);
    vel = (robotdata->qd_bound(i, 0) - robotdata->q_dot_a(i, 0)) / (count * dt);
    UB_base(i, 0) = std::min(pos, vel);
  }
  // std::cout<<"hehe"<<std::endl;
  int dims = ndof;

  D_base.block(0, 0, ndof - 6, dims) = dyna.Ma;
  D_base.block(ndof - 6, 0, ndof - 6, dims) = -dyna.Ma;

  f_base.block(0, 0, ndof - 6, 1) =
      robotdata->tau_bound.block(6, 0, ndof - 6, 1) - dyna.Na - dyna.Ja_T * robotdata->contactforce;
  f_base.block(ndof - 6, 0, ndof - 6, 1) =
      robotdata->tau_bound.block(6, 0, ndof - 6, 1) + dyna.Na + dyna.Ja_T * robotdata->contactforce;

  // update friction cone constraints
  double miu = 0.4;
  double lamada = 0.1;
  double deltxP = 0.11;
  double deltxN = 0.05;
  double delty = 0.02;
  if (robotdata->inSingleStand) {
    delty = 0.015;
  }

  // double deltXPLeft = fmax(fabs(deltxP * cos(robotdata->q_a(7))), fabs(delty
  // * sin(robotdata->q_a(7)))); double deltXNLeft = fmax(fabs(deltxN *
  // cos(robotdata->q_a(7))), fabs(delty * sin(robotdata->q_a(7)))); double
  // deltYLeft = fmax(fabs(deltxN * sin(robotdata->q_a(7))), fabs(delty *
  // cos(robotdata->q_a(7)))); double deltXPRight = fmax(fabs(deltxP *
  // cos(robotdata->q_a(13))), fabs(delty * sin(robotdata->q_a(13)))); double
  // deltXNRight = fmax(fabs(deltxN * cos(robotdata->q_a(13))), fabs(delty *
  // sin(robotdata->q_a(13)))); double deltYRight = fmax(fabs(deltxN *
  // sin(robotdata->q_a(13))), fabs(delty * cos(robotdata->q_a(13))));

  double deltXPLeft = deltxP;
  double deltXNLeft = deltxN;
  double deltYLeft = delty;
  double deltXPRight = deltxP;
  double deltXNRight = deltxN;
  double deltYRight = delty;

  Eigen::MatrixXd A_c_left = Eigen::MatrixXd::Zero(14, 6);
  Eigen::MatrixXd A_c_right = Eigen::MatrixXd::Zero(14, 6);

  GenerateFrictioncone(miu, lamada, deltXPLeft, deltXNLeft, deltYLeft, A_c_left);
  GenerateFrictioncone(miu, lamada, deltXPRight, deltXNRight, deltYRight, A_c_right);

  priorityorder[0].A_c.setZero(28, 12);
  priorityorder[0].A_c.block(0, 0, 14, 6) = A_c_left;
  priorityorder[0].A_c.block(14, 6, 14, 6) = A_c_right;
}

void WbcSolver::GenerateFrictioncone(double miu, double lamda, double deltxP, double deltxN, double delty,
                                     Eigen::MatrixXd& A_c) {
  int n = 1;
  int ln = 2 * 4 + 6;
  A_c = Eigen::MatrixXd::Zero(ln, 6);
  // plane contact
  A_c.row(0) << 0., 0., 1., 0., 0., -lamda;
  A_c.row(1) << 0., 0., -1., 0., 0., -lamda;
  A_c.row(2) << 1., 0., 0., 0., 0., -delty;
  A_c.row(3) << -1., 0., 0., 0., 0., -delty;
  A_c.row(4) << 0., 1., 0., 0., 0., -deltxN;
  A_c.row(5) << 0., -1., 0., 0., 0., -deltxP;
  // point contact
  int i = 0;
  double k0 = (cos((i + 1) * M_PI / 4) - cos((i)*M_PI / 4)) / (sin((i + 1) * M_PI / 4) - sin((i)*M_PI / 4));
  double b0 = miu * (k0 * sin((i)*M_PI / 4) - cos((i)*M_PI / 4));
  i = 1;
  double k1 = (cos((i + 1) * M_PI / 4) - cos((i)*M_PI / 4)) / (sin((i + 1) * M_PI / 4) - sin((i)*M_PI / 4));
  double b1 = miu * (k1 * sin((i)*M_PI / 4) - cos((i)*M_PI / 4));
  A_c.row(6) << 0., 0., 0., -k0, 1, b0;
  A_c.row(7) << 0., 0., 0., -k1, 1, b1;

  A_c.row(8) << 0., 0., 0., -k0, -1, b0;
  A_c.row(9) << 0., 0., 0., -k1, -1, b1;

  A_c.row(10) << 0., 0., 0., k0, 1, b0;
  A_c.row(11) << 0., 0., 0., k1, 1, b1;

  A_c.row(12) << 0., 0., 0., k0, -1, b0;
  A_c.row(13) << 0., 0., 0., k1, -1, b1;
}
