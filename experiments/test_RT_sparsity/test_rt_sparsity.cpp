#include <iostream>
#include <thread>
#include <mutex>
#include <ctime>
#include <array>
#include <exception>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>

#include <gmp_lib/GMP/GMP.h>
#include <gmp_lib/GMP/GMP_MPC.h>
#include <gmp_lib/io/file_io.h>
#include <yaml-cpp/yaml.h>
#include <thread_lib/thread_lib.h>

using namespace as64_;


double y0_;
double yg_;
double Tf;

arma::rowvec pos_lim;
arma::rowvec vel_lim;
arma::rowvec accel_lim;

unsigned n_dof;

unsigned N_horizon;
double pred_time_step;
unsigned N_kernels;
double kernels_std_scaling;

double trunc_kern_thres;

double opt_pos;
double opt_vel;

std::array<double,3> slack_gains;
std::array<double,3> slack_limits;

arma::vec final_state_err_tol;

double time_limit;
unsigned max_iter;
double abs_tol;
double rel_tol;

std::string main_path;

void loadParams(const std::string &filename);
void printElapsTime(const arma::rowvec &elaps_t_data, const std::string &title);
void PRINT_INFO_MSG(const std::string &msg, std::ostream &out=std::cerr);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out=std::cerr);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out=std::cerr);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out=std::cerr);

void test_rt_sparsity();

// ======================================
// ============== MAIN ==================
// ======================================

int main(int argc, char **argv)
{
  main_path = ros::package::getPath("gmp_mpc_controller") + "/config/";

  loadParams(main_path + "/test_rt_sparsity_params.yaml");

  std::thread thr = std::thread(test_rt_sparsity);
    
  int err_code = thr_::setThreadPriority(thr, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG("[MainController::makeThreadRT]: Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[MainController::makeThreadRT]: Set thread priority successfully!\n", std::cerr);

  thr.join();

  return 0;
}

// ===================================
// ============= Utils ===============
// ===================================

void test_rt_sparsity()
{
  // ============= Load training data ============
    arma::rowvec Timed;
    arma::rowvec yd_data;
    try
    {
      gmp_::FileIO fid(main_path + "/test_rt_sparsity_data.bin", gmp_::FileIO::in);
      fid.read("Time", Timed);
      fid.read("y_data", yd_data);
    }
    catch(const std::exception& e)
    {
      PRINT_ERROR_MSG(e.what());
      exit(-1);
    }
    
    arma::mat Yd_data = arma::repmat(yd_data, n_dof, 1);

    // ============  Create and train a DMP  ==============
    gmp_::GMP::Ptr gmp( new gmp_::GMP(n_dof, 30, 1.5) );
    arma::vec train_err;
    gmp->train("LS", Timed/Timed.back(), Yd_data, &train_err);
    std::cerr << "train_err = \n" << train_err << "\n";
    
    // ==========  Initialize GMP_MPC ============

    gmp_::GMP_MPC::Ptr gmp_mpc;
    gmp_mpc.reset( new gmp_::GMP_MPC(gmp.get(), N_horizon, pred_time_step, N_kernels, kernels_std_scaling, slack_gains, trunc_kern_thres) );

    gmp_mpc->setObjCostGains(opt_pos, opt_vel);

    gmp_mpc->settings.time_limit = time_limit;
    gmp_mpc->settings.max_iter = max_iter;
    gmp_mpc->settings.abs_tol = abs_tol;
    gmp_mpc->settings.rel_tol = rel_tol;

    arma::mat y_lim = arma::repmat(pos_lim, n_dof, 1);
    arma::mat ydot_lim = arma::repmat(vel_lim, n_dof, 1);
    arma::mat yddot_lim = arma::repmat(accel_lim, n_dof, 1);
    
    gmp_mpc->setPosLimits(y_lim.col(0), y_lim.col(1));
    gmp_mpc->setVelLimits(ydot_lim.col(0), ydot_lim.col(1));
    gmp_mpc->setAccelLimits(yddot_lim.col(0), yddot_lim.col(1));

    gmp_mpc->setPosSlackLimit(slack_limits[0]);
    gmp_mpc->setVelSlackLimit(slack_limits[1]);
    gmp_mpc->setAccelSlackLimit(slack_limits[2]);

    // ========== Run simulation ===========

    double t = 0;
    double dt = 0.002;

    double s = 0;
    double s_dot = 1/Tf;
    double s_ddot = 0;

    arma::vec O_ndof = arma::vec().zeros(n_dof);

    arma::vec y0 = y0_*arma::vec().ones(n_dof);
    arma::vec yg = yg_*arma::vec().ones(n_dof);
    
    arma::vec y = y0;
    arma::vec y_dot = O_ndof;
    arma::vec y_ddot = O_ndof;

    gmp_mpc->setInitialState(y, y_dot, y_ddot, s, s_dot, s_ddot);
    gmp_mpc->setFinalState(yg, O_ndof, O_ndof, 1, s_dot, 0, final_state_err_tol);

    auto can_sys_fun = [](double s, double s_dot){ return std::array<double,2>({s_dot, 0.0}); };
    gmp_mpc->setCanonicalSystemFunction(can_sys_fun);

    // gmp->setScaleMethod( gmp_::TrajScale::Ptr( new gmp_::TrajScale_Prop(n_dof) ) );
    gmp->setY0(y0);
    gmp->setGoal(yg);

    arma::rowvec elaps_t_data;
    arma::wall_clock timer;

    bool success = true;

    // =========  Simulation loop  =========
    while (true)
    {
      // --------  Stopping criteria  --------
      if (s > 1.0) break;
    
      timer.tic();
      gmp_::GMP_MPC::Solution sol = gmp_mpc->solve(s, s_dot);

      if (sol.exit_flag > 0) PRINT_WARNING_MSG(sol.exit_msg + "\n");
      else if (sol.exit_flag < 0)
      {
        success = false;
        PRINT_ERROR_MSG(sol.exit_msg + "\n");
        break;
      }

      y = sol.y;
      y_dot = sol.y_dot;
      y_ddot = sol.y_ddot;
      arma::vec pos_slack_var = sol.pos_slack;
      arma::vec vel_slack_var = sol.vel_slack;
      arma::vec accel_slack_var = sol.accel_slack;
      elaps_t_data = arma::join_horiz( elaps_t_data, arma::vec({timer.toc()*1000}) );
      
      // --------  Numerical integration  --------
      t = t + dt;

      std::array<double,2> phase_dot = can_sys_fun(s, s_dot);
      s = s + phase_dot[0]*dt;
      s_dot = s_dot + phase_dot[1]*dt;
    }

    if (success)
    {
      PRINT_INFO_MSG("Finished!");
      // emit gui->showMsgSignal(ExecResultMsg(ExecResultMsg::INFO, "Finished!"));
    }

    printElapsTime(gmp_mpc->elaps_t_data, "========= Total elapsed time (ms) =========\n");
    printElapsTime(gmp_mpc->solve_elaps_t_data, "========= Solve elapsed time (ms) =========\n");
    printElapsTime(gmp_mpc->calcH_elaps_t_data, "========= Calc H elapsed time (ms) =========\n");
    std::cerr << "---------------------------\n";

    {
      Eigen::SpMat H = gmp_mpc->H_;
      Eigen::SpMat A = gmp_mpc->A_;

      int nnz = H.nonZeros();
      int n_elem = H.size();
      printf("H sparsity: %.1f %% ( %d / %d )\n", 100*nnz/(double)n_elem, nnz, n_elem);

      nnz = A.nonZeros();
      n_elem = A.size();
      printf("A sparsity: %.1f %% ( %d / %d )\n", 100*nnz/(double)n_elem, nnz, n_elem);
    }
    
    if (!elaps_t_data.empty())
    {
      double mean_elaps_t = arma::mean(elaps_t_data);
      double std_elaps_t = arma::stddev(elaps_t_data);
      double max_elaps_t = arma::max(elaps_t_data);
      double min_elaps_t = arma::min(elaps_t_data);

      // std::cerr << "======= Elapsed time (ms) ======\n";
      // std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
      // std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
      // std::cerr << "min      : " << min_elaps_t << "\n";
      // std::cerr << "max      : " << max_elaps_t << "\n";
      // std::cerr << "==============================\n";

      std::cerr << "======= Final state errors (m) ======\n";
      std::cerr << "pos_err = " << arma::norm(yg - y) << "\n";
      std::cerr << "vel_err = " << arma::norm(y_dot) << "\n";
      std::cerr << "accel_err = " << arma::norm(y_ddot) << "\n";
      std::cerr << "==============================\n";
    }
}

void loadParams(const std::string &filename)
{
  // ============== Load params =============
  try
  {
    YAML::Node node = YAML::LoadFile(filename);

    if ( !YAML::getParam(node, "y0", y0_) )
      throw std::runtime_error("Failed to load param 'y0'...");

    if ( !YAML::getParam(node, "yg", yg_) )
      throw std::runtime_error("Failed to load param 'yg'...");

    if ( !YAML::getParam(node, "Tf", Tf) )
      throw std::runtime_error("Failed to load param 'Tf'...");

    if ( !YAML::getParam(node, "pos_lim", pos_lim) )
      throw std::runtime_error("Failed to load param 'y_lim'...");

    if ( !YAML::getParam(node, "vel_lim", vel_lim) )
      throw std::runtime_error("Failed to load param 'vel_lim'...");

    if ( !YAML::getParam(node, "accel_lim", accel_lim) )
      throw std::runtime_error("Failed to load param 'accel_lim'...");

    if ( !YAML::getParam(node, "n_dof", n_dof) )
      throw std::runtime_error("Failed to load param 'n_dof'...");

    if ( !YAML::getParam(node, "N_horizon", N_horizon) )
      throw std::runtime_error("Failed to load param 'N_horizon'...");

    if ( !YAML::getParam(node, "pred_time_step", pred_time_step) )
      throw std::runtime_error("Failed to load param 'pred_time_step'...");

    if ( !YAML::getParam(node, "N_kernels", N_kernels) )
      throw std::runtime_error("Failed to load param 'N_kernels'...");

    if ( !YAML::getParam(node, "kernels_std_scaling", kernels_std_scaling) )
      throw std::runtime_error("Failed to load param 'kernels_std_scaling'...");

    if ( !YAML::getParam(node, "trunc_kern_thres", trunc_kern_thres) )
      throw std::runtime_error("Failed to load param 'trunc_kern_thres'...");

    if ( !YAML::getParam(node, "opt_pos", opt_pos) )
      throw std::runtime_error("Failed to load param 'opt_pos'...");

    if ( !YAML::getParam(node, "opt_vel", opt_vel) )
      throw std::runtime_error("Failed to load param 'opt_vel'...");

    if ( !YAML::getParam(node, "slack_gains", slack_gains) )
      throw std::runtime_error("Failed to load param 'slack_gains'...");

    if ( !YAML::getParam(node, "slack_limits", slack_limits) )
      throw std::runtime_error("Failed to load param 'slack_limits'...");

    if ( !YAML::getParam(node, "final_state_err_tol", final_state_err_tol) )
      final_state_err_tol = arma::vec().zeros(3,1);

    if ( !YAML::getParam(node, "time_limit", time_limit) )
      throw std::runtime_error("Failed to load param 'time_limit'...");

    if ( !YAML::getParam(node, "max_iter", max_iter) )
      throw std::runtime_error("Failed to load param 'max_iter'...");

    if ( !YAML::getParam(node, "abs_tol", abs_tol) )
      throw std::runtime_error("Failed to load param 'abs_tol'...");

    if ( !YAML::getParam(node, "rel_tol", rel_tol) )
      throw std::runtime_error("Failed to load param 'rel_tol'...");

  }
  catch (std::exception &e)
  { 
    PRINT_ERROR_MSG(e.what());
    exit(-1); 
  }
}

void printElapsTime(const arma::rowvec &elaps_t_data, const std::string &title)
{
  double mean_elaps_t = arma::mean(elaps_t_data);
  double std_elaps_t = arma::stddev(elaps_t_data);
  double max_elaps_t = arma::max(elaps_t_data);
  double min_elaps_t = arma::min(elaps_t_data);

  if (!title.empty()) std::cerr << title;
  std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
  std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
  std::cerr << "min      : " << min_elaps_t << "\n";
  std::cerr << "max      : " << max_elaps_t << "\n";
  std::cerr << "==============================\n";
}


void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}