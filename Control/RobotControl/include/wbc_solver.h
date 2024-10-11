#ifndef WBC_SOLVER
#define WBC_SOLVER
#include <thread>
#include <vector>
#include "QPproblem.h"
#include "Robot_Data.h"
#include "basicfunction.h"
// #define PROXQP

#ifdef PROXQP
#  include "ProxQP.h"
using namespace proxsuite::proxqp;
#endif

struct PriorityOrder {
  // priority
  int priority = 0;
  // task number
  int ntask = 0;

  // tasks and constraints included in this priority
  std::vector<int> taskID;
  std::vector<int> taskdim;
  std::vector<int> constdim_x;
  std::vector<int> constdim_xd;

  // task and constraint dimensions
  int taskdim_total = 0;
  int constdim_total = 0;

  // contact task flag
  bool contact_flag = false;
  std::vector<int> contact_id;
  std::vector<int> contactdim;
  std::vector<int> constdim_f;

  int f_taskdim_total = 0;
  int f_constdim_total = 0;

  // task weight
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> taskweight;

  // qpOases
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> optimal_qpresult;  //(un;vn)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> y_star;            // y_star

  // null space
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Z;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_hat_hat_inv;
  // equality constraints
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_hat;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b_hat;
  // inequality constraints
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_hat;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a_hat;
  // can not violate constraints . only in priority = 0;
  // equality constraints is the object of the priority = 0
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_c;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a_c;
  // damping
  double ypsilon = 1.0e-4;
};
struct RobotDynamics {
  Eigen::MatrixXd M;
  Eigen::MatrixXd M_inv;

  Eigen::MatrixXd Mb;    // inertial
  Eigen::MatrixXd Ma;    // inertial
  Eigen::MatrixXd Jb_T;  // internal
  Eigen::MatrixXd Ja_T;  // internal

  //
  Eigen::MatrixXd MbJb_T;
  Eigen::MatrixXd MaJa_T;

  Eigen::MatrixXd Nb;  // nonlinear
  Eigen::MatrixXd Na;  // nonlinear
};
/**
 * @brief WbcSolver
 *  *
 */
class WbcSolver {
 public:
  // construct function
  WbcSolver();
  ~WbcSolver();

  // init
  void Init(Robot_Data* robotdata);
  // solve and update the joint command
  void SolveWbc(Robot_Data* robotdata);
  // set constraints
  void SetConstraints(Eigen::VectorXd _tau_lb, Eigen::VectorXd _tau_ub, Eigen::VectorXd _GRF_lb,
                      Eigen::VectorXd _GRF_ub, Eigen::VectorXd _Fref);

 public:
  // task num
  int task_num = 0;
  // priority num
  int priority_num;
  // optimal variables dimesions
  int ndof;
  // contact force dimesions
  int dim_f = 0;
  int task_num_f = 0;
  std::vector<int> task_f_id;
  std::vector<int> task_f_dim;
  // comtask
  int com_task_id;
  // dynamics
  RobotDynamics dyna;
  // priority order
  PriorityOrder* priorityorder;
  // control cycle, unit:s
  double dt;
  double weight_factor;
  // construct qp problem
  std::vector<QPproblem*> qp_task;

#ifdef PROXQP
  ProxQP* wqp;
#endif

  // WBIC QP
  QPproblem* wbc;
  Eigen::MatrixXd deltX;
  // WQP
  Eigen::VectorXd tau_lb;
  Eigen::VectorXd tau_ub;
  // mx my mz fx fy fz
  Eigen::VectorXd GRF_ub;
  Eigen::VectorXd GRF_lb;
  // GRF ref
  Eigen::VectorXd Fref;
  // prediction count
  int count = 5;
  /**
   * @brief
   * update robot dynamics and calculate the base constraint matrix
   * input: Robot_Data
   */
  // lb ub
  Eigen::VectorXd LB_base;
  Eigen::VectorXd UB_base;
  // lu_tau ub_tau
  Eigen::MatrixXd D_base;
  Eigen::VectorXd f_base;

  /**
   * @brief
   * init function for different methods
   * input: Robot_Data
   * update: priority number & tasks and constraints in this priority ->
   */
  // Float_Base_Open_Chain + hqp_dynamics
  void InitDynamics(Robot_Data* robotdata);

  /**
   * @brief
   * update robot dynamics and calculate the base constraint matrix
   * input: Robot_Data
   */
  // Float_Base_Open_Chain + hqp_dynamics
  void UpdateDynamics(Robot_Data* robotdata);

  /**
   * @brief friction cone constraints matrix
   *
   */
  void GenerateFrictioncone(double miu, double lamda, double deltxP, double deltaxN, double delty,
                            Eigen::MatrixXd& A_c);
};
#endif