#ifndef $_PROJECT_384$_GMP_MPC_CONTROLLER_OBSTACLES_EXPERIMENT_H
#define $_PROJECT_384$_GMP_MPC_CONTROLLER_OBSTACLES_EXPERIMENT_H

#include <sstream>
#include <gmp_mpc_controller/experiment/experiment.h>
#include <gmp_mpc_controller/gmp_mpc_controller.h>

#include <math_lib/math.h>

#include <rviz_lib/tf_pose_publisher.h>
#include <rviz_lib/publish_path.h>

#include <gmp_mpc_controller/utils.h>

using namespace as64_;

namespace gmp_mpc_ctrl_
{

class ObstaclesExperiment : public Experiment
{
public:

  ObstaclesExperiment(GmpMpcController *ctrl_);
  ~ObstaclesExperiment();

  bool init() override;

  void execute() override;


protected:

  std::shared_ptr<GmpMpcRvizPlot> rviz_plot;

  bool use_sim;

  bool publish_path_online;

  bool init_success = false;

  std::string main_path;
  std::string gmp_model_filename;
  std::string exec_data_filename;
  std::string params_filename;

  GmpMpcController *ctrl;

  apriltag_ros::AprilTagListener *tag_listener;
  rviz_::RvizMarkerPublisher *rviz_pub;

  rw_::Robot *robot;
  r85_::R85Gripper *gripper;

  gmp_::GMP::Ptr gmp;
  gmp_::GMP_MPC::Ptr gmp_mpc;

  MpcSettings mpc_cfg;

  double Tf;

  arma::vec q_start; // starting joints configuration for the robot

  arma::vec target_pos;

  arma::mat pos_lim;
  arma::mat vel_lim;
  arma::mat accel_lim;

  struct ObstTagConfig
  {
    int id;
    std::string name;
    arma::vec center;
    arma::vec axes_len;
  };

  std::list<ObstTagConfig> obst_tag_cfg;

  std::vector<Obstacle> getObstacles();

  void openGripper();


  std::thread pub_obst_thread;
  bool publish_obstacles;
  std::vector<Obstacle> rviz_obstacles;
  std::mutex rviz_obst_mtx;
  void updateRvizObstacles(const std::vector<Obstacle> &obstacles);
  void publishObstacles();


  std::shared_ptr<rviz_::PublishPath> pub_path;
  std::shared_ptr<rviz_::PublishPath> pub_ref_path;

  int target_tag_id;
  arma::vec target_offset; // w.r.t. the base frame
  int max_ntimes_target_not_detected;
  int readTarget(arma::vec &yg, int &ntimes_target_not_detected);

  //=========== data to log ==========
  arma::rowvec Time_data;
  arma::mat P_data;
  arma::mat dP_data;
  arma::mat ddP_data;
  arma::mat target_data;
  std::vector<std::vector<Obstacle>> obstacles_data;

};

} // namespace gmp_mpc_ctrl_

#endif // $_PROJECT_384$_GMP_MPC_CONTROLLER_OBSTACLES_EXPERIMENT_H
