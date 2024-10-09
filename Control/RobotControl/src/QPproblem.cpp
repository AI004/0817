#include "../include/QPproblem.h"
// #include "QPproblem.h"
QPproblem::QPproblem() {}

QPproblem::~QPproblem() {
  delete qp_problem;
  delete qp_hessian;
  delete qp_constraint;
  delete qp_gradient;
  delete qp_lbound;
  delete qp_ubound;
  delete qp_lbconstraint;
  delete qp_ubconstraint;
  delete qp_optimal;
}

// init qp problem
void QPproblem::init(int taskdim, int constdim, double cputime, int nWSR) {
  qp_taskdim = taskdim;
  qp_constdim = constdim;
  qp_cputime = cputime;
  qp_nWSR = nWSR;

  // init all variables
  qp_problem = new qpOASES::SQProblem(qp_taskdim, qp_constdim);
  qp_hessian = new qpOASES::real_t[qp_taskdim * qp_taskdim];
  qp_constraint = new qpOASES::real_t[qp_taskdim * qp_constdim];
  qp_gradient = new qpOASES::real_t[qp_taskdim];
  qp_lbound = new qpOASES::real_t[qp_taskdim];
  ;
  qp_ubound = new qpOASES::real_t[qp_taskdim];
  qp_lbconstraint = new qpOASES::real_t[qp_constdim];
  qp_ubconstraint = new qpOASES::real_t[qp_constdim];
  // QP optimal result
  qp_optimal = new qpOASES::real_t[qp_taskdim];
  // QP option
  // qp_option.setToReliable();
  /** information level:
   * PL_NONE: no output at all
   * qpOASES::PL_LOW: print error messages only
   *  qpOASES::PL_MEDIUM : print error messages, warnings, some info messages as
   * well as a concise  iteration summary (default value)
   *  */
  // qp_option.setToFast();
  qp_option.printLevel = qpOASES::PL_LOW;
  qp_option.terminationTolerance = 0.000001;
  qp_option.enableRegularisation = qpOASES::BT_TRUE;
  qp_option.epsRegularisation = 0.00001; // 这个参数需要自行调节一下
  // qp_option.numRegularisationSteps = 5000;
  // Enables or disables the use of far bounds, an idea to reliably detect
  // unboundedness
  qp_problem->setOptions(qp_option);

  m_hessian.resize(qp_taskdim, qp_taskdim);
  m_gradient.resize(qp_taskdim, 1);
  m_lbound.resize(qp_taskdim, 1);
  m_ubound.resize(qp_taskdim, 1);
  m_optimal.resize(qp_taskdim, 1);
  //    if (qp_constdim != 0)
  //    {
  m_constraint.resize(qp_constdim, qp_taskdim);
  m_lbconstraint.resize(qp_constdim, 1);
  m_ubconstraint.resize(qp_constdim, 1);
  //    }

  m_hessian.setZero();
  m_gradient.setZero();
  m_lbound.setZero();
  m_ubound.setZero();
  m_constraint.setZero();
  m_lbconstraint.setZero();
  m_ubconstraint.setZero();
  m_optimal.setZero();

  // qp_init_flag
  qp_init_flag = false;
  solve_flag = false;
}

// init qp problem
bool QPproblem::solve() {

  // update qp array
  updateQPArray();

  qpOASES::real_t cputime[1];
  cputime[0] = qp_cputime;
  qpOASES::int_t nWSR = qp_nWSR;
  // std::cout<<"nWSR: "<<nWSR<<std::endl;
  // qp_init_flag = false;
  if (qp_init_flag == false) {
    cputime[0] = qp_cputime * 1;
    QP_returnValue = qp_problem->init(qp_hessian, qp_gradient, qp_constraint,
                                      qp_lbound, qp_ubound, qp_lbconstraint,
                                      qp_ubconstraint, nWSR, cputime);
    // qp_init_flag = true;
    // std::cout<<"qp init function!"<<std::endl;
  } else {
    cputime[0] = qp_cputime;
    QP_returnValue = qp_problem->hotstart(
        qp_hessian, qp_gradient, qp_constraint, qp_lbound, qp_ubound,
        qp_lbconstraint, qp_ubconstraint, nWSR, cputime);
    // qp_init_flag = true;
    // std::cout<<"qp hotstart  function!"<<std::endl;
  }
  // whether solve successfully
  if (QP_returnValue == qpOASES::SUCCESSFUL_RETURN) {
    qp_init_flag = true;
    qp_problem->getPrimalSolution(qp_optimal);
    for (int loopi = 0; loopi < qp_taskdim; loopi++) {
      m_optimal(loopi) = qp_optimal[loopi];
    }
    solve_flag = true;
    return true;
  } else {
    qp_init_flag = false;
    // std::cout  << std::endl;
    std::cout << "QP SOLVED FAILED ! Error ID" << QP_returnValue << std::endl;
    solve_flag = false;
    return false;
  }
}

// get optimal result
Eigen::Matrix<double, Eigen::Dynamic, 1> QPproblem::getOptimal() {
  return m_optimal;
}

void QPproblem::MatrixToArray(qpOASES::real_t *arr,
                              Eigen::MatrixXd input_matrix) {
  int col = input_matrix.cols();
  int row = input_matrix.rows();

  int size_arr = 0;

  for (int size_row = 0; size_row < row; size_row++) {
    for (int size_col = 0; size_col < col; size_col++) {
      arr[size_arr] = input_matrix(size_row, size_col);
      size_arr++;
    }
  }
}

void QPproblem::updateQPArray() {
  MatrixToArray(qp_hessian, m_hessian);
  MatrixToArray(qp_gradient, m_gradient);

  if (m_lbound.rows() != 0) {
    MatrixToArray(qp_lbound, m_lbound);
    MatrixToArray(qp_ubound, m_ubound);
  } else {
    *qp_lbound = 0.0;
    *qp_ubound = 0.0;
  }

  if (qp_constdim != 0) {
    MatrixToArray(qp_constraint, m_constraint);
    MatrixToArray(qp_ubconstraint, m_ubconstraint);
    MatrixToArray(qp_lbconstraint, m_lbconstraint);
  } else {
    *qp_constraint = 0.0;
    *qp_ubconstraint = 0.0;
    *qp_lbconstraint = 0.0;
  }
}
