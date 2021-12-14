#ifndef $_PROJECT_384$_GMP_MPC_CONTROLLER_HANDOVER_EXPERIMENT_H
#define $_PROJECT_384$_GMP_MPC_CONTROLLER_HANDOVER_EXPERIMENT_H

#include <sstream>
#include <gmp_mpc_controller/experiment/experiment.h>
#include <gmp_mpc_controller/gmp_mpc_controller.h>

using namespace as64_;

namespace gmp_mpc_ctrl_
{

class HandoverExperiment : public Experiment
{
public:

  HandoverExperiment(GmpMpcController *ctrl_);

  bool init() override;

  void execute() override;


protected:

  bool init_success = false;

  std::string main_path;
  std::string gmp_model_filename;
  std::string exec_data_filename;
  std::string params_filename;

  GmpMpcController *ctrl;

  apriltag_ros::AprilTagListener *tag_listener; 

  rw_::Robot *robot;
  r85_::R85Gripper *gripper;

  //=========== data to log ==========
  arma::rowvec Time_data;
  arma::mat phase_var_data; // {s, s_dot}
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  arma::mat target_data;
  arma::rowvec time_duration_data;
  
  gmp_::GMP::Ptr gmp;
  gmp_::GMP_MPC::Ptr gmp_mpc;

  MpcSettings mpc_cfg;

  int target_tag_id;
  arma::vec target_offset; // w.r.t. the base frame

  double Tf_per_meter;
  double Tf_update_gain;

  arma::vec q_start; // starting joints configuration for the robot

  arma::mat pos_lim;
  arma::mat vel_lim;
  arma::mat accel_lim;

  int max_ntimes_target_not_detected;
  int readTarget(arma::vec &yg, int &ntimes_target_not_detected);

  void openGripper();

};

} // namespace gmp_mpc_ctrl_

#endif // $_PROJECT_384$_GMP_MPC_CONTROLLER_HANDOVER_EXPERIMENT_H
