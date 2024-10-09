#ifndef QPPROBLEM_H
#define QPPROBLEM_H
#include <Eigen/Dense>
#include <iostream>
#include <qpOASES.hpp>

const double INFINITY_QP = qpOASES::INFTY;

class QPproblem {
public:
  QPproblem();
  ~QPproblem();

  // init qp problem
  void init(int taskdim, int constdim, double cputime = 0.002, int nWSR = 1000);
  // init qp problem
  bool solve();
  // get optimal result
  Eigen::Matrix<double, Eigen::Dynamic, 1> getOptimal();

  // qp matrix for calculation
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_hessian;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_constraint;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_gradient;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_lbound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_ubound;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_lbconstraint;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_ubconstraint;
  Eigen::Matrix<double, Eigen::Dynamic, 1> m_optimal;

public:
  int qp_taskdim;
  int qp_constdim;

  qpOASES::SQProblem *qp_problem;
  qpOASES::real_t qp_cputime; // s
  qpOASES::int_t qp_nWSR;
  qpOASES::real_t *qp_hessian;
  qpOASES::real_t *qp_constraint;
  qpOASES::real_t *qp_gradient;
  qpOASES::real_t *qp_lbound;
  qpOASES::real_t *qp_ubound;
  qpOASES::real_t *qp_lbconstraint;
  qpOASES::real_t *qp_ubconstraint;
  // QP optimal result
  qpOASES::real_t *qp_optimal;
  // QP returnValue
  qpOASES::returnValue QP_returnValue;
  // QP option
  qpOASES::Options qp_option;
  // qp init flag
  bool qp_init_flag;

  // qp solve flag
  bool solve_flag;
  // MatrixToArray
  void MatrixToArray(qpOASES::real_t *arr, Eigen::MatrixXd input_matrix);
  // update qp array according to the qp matrix
  void updateQPArray();
};

#endif // QPPROBLEM_H
