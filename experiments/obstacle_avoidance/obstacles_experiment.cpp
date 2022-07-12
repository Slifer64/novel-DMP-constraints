#include <gmp_mpc_controller/experiment/obstacles_experiment.h>

#include <gmp_mpc_controller/gmp_mpc_controller.h>

// #define DEBUG_ON


namespace gmp_mpc_ctrl_
{

#define GmpMpcObstacles_fun_ std::string("[GmpMpc_Obstacles_exp_::") + __func__ + "]: "

ObstaclesExperiment::ObstaclesExperiment(GmpMpcController *ctrl_)
{
  this->ctrl = ctrl_;

  this->main_path = ros::package::getPath("gmp_mpc_controller") + "/";
  this->params_filename = main_path + "/config/obstacles_params.bin";
  this->gmp_model_filename = main_path + "/config/obstacles_gmp_model.bin";
  this->exec_data_filename = main_path + "/data/obstacles_exec_data.bin";

  // pub_path.reset(new rviz_::PublishPath(ctrl_->marker_array_topic, ctrl_->base_link, "obstacles_online_path"));
  // pub_ref_path.reset(new rviz_::PublishPath(ctrl_->marker_array_topic, ctrl_->base_link, "obstacles_online_ref_path"));
  // pub_ref_path->color = rviz_::Color::CYAN;

  // ============ For online visualization ===============
  rviz_plot.reset(new GmpMpcRvizPlot(ctrl_->marker_array_topic, ctrl_->base_link));
}

ObstaclesExperiment::~ObstaclesExperiment()
{
  
}

int ObstaclesExperiment::readTarget(arma::vec &yg, int &ntimes_target_not_detected)
{
  // ------- For DEBUGGING --------
  #ifdef DEBUG_ON
  yg = {0.6917,   0.0424,   0.1639};
  return 0;
  #endif

  apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(target_tag_id);
  if (tag.tag_id < 0) ++ntimes_target_not_detected;
  else
  {
    ntimes_target_not_detected = 0;
    Eigen::Vector3d offset(target_offset(0), target_offset(1), target_offset(2));
    Eigen::Vector3d p = tag.pos + tag.quat.toRotationMatrix()*offset;

    // std::cerr << "R = \n" << tag.quat.matrix() << "\n";
    // std::cerr << "p2 = " << p.transpose() << "\n";
    // std::cerr << "p = " << tag.pos.transpose() << "\n";
    // std::cerr << "offset = " << offset.transpose() << "\n";

    yg = { p.x(), p.y(), p.z() };
    // yg += target_offset;
  }

  if (ntimes_target_not_detected == 0) return 0; // found target
  else if (ntimes_target_not_detected < max_ntimes_target_not_detected) return 1; // didn't find target, but iterations not exceeded
  else return -1; // iterations to find target exceeded
}

bool ObstaclesExperiment::init()
{
  init_success = false;

  robot = ctrl->robot;
  gripper = ctrl->main_ctrl->gripper.get();

  this->rviz_pub = ctrl->getRvizPublisher();

  this->tag_listener = ctrl->getTagListener(); // update here, so if it is reinitialized, we point to the new updated location
  if (!tag_listener)
  {
    ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, "tag_listener is not initialized..."));
    return init_success;;
  }

  unsigned n_dof = 3;
  arma::vec ones_ndof = arma::vec().ones(n_dof);
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  // ======== Load model =========
  gmp.reset( new gmp_::GMP() );
  try{  gmp_::read( gmp.get(), gmp_model_filename ); }
  catch (std::exception &e)
  { 
    ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, e.what() ));
    return init_success;;
  }

  // ========== Load params =========

  try
  {
    YAML::Node config = YAML::LoadFile(main_path + "/config/obstacles_params.yaml");

    if ( !YAML::getParam(config, "use_sim", use_sim) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'use_sim'...");

    if ( !YAML::getParam(config, "publish_path_online", publish_path_online) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'publish_path_online'...");
      
    if ( !YAML::getParam(config, "q_start", q_start) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'q_start'...");

    if ( !YAML::getParam(config, "target_pos", target_pos) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'target_pos'...");

    if ( !YAML::getParam(config, "target_tag_id", target_tag_id) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'target_tag_id'...");

    if ( !YAML::getParam(config, "target_offset", target_offset) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'target_offset'...");

    if ( !YAML::getParam(config, "max_ntimes_target_not_detected", max_ntimes_target_not_detected) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'max_ntimes_target_not_detected'...");

    if ( !YAML::getParam(config, "Tf", Tf) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'Tf_per_meter'...");

    if ( !YAML::getParam(config, "publish_obstacles", publish_obstacles) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'publish_via_points'...");

    // load kinematic limits
    if ( !YAML::getParam(config, "pos_lim", pos_lim) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'pos_lim'...");
    if ( !YAML::getParam(config, "vel_lim", vel_lim) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'vel_lim'...");
    if (vel_lim.n_rows == 1) vel_lim = arma::repmat(vel_lim, 3, 1);
    if ( !YAML::getParam(config, "accel_lim", accel_lim) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'accel_lim'...");
    if (accel_lim.n_rows == 1) accel_lim = arma::repmat(accel_lim, 3, 1);

    // update the main controller's limits (so that any plots or visualization in rviz displays the correct limits)
    ctrl->setKinematicLimits(pos_lim, vel_lim, accel_lim);

    ctrl->setInitPose(q_start);

    // ======== load obstacle tags config ===========
    obst_tag_cfg.clear();

    YAML::Node vp_node;
    if ( !YAML::getParam(config, "obstacle_tags", vp_node) )
      throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'obstacle_tags'...");
    if ( ! vp_node.IsSequence() )
      throw std::runtime_error(GmpMpcObstacles_fun_+"'obstacle_tags' must be an array...");
    
    for (const YAML::Node &vp_node_i : vp_node)
    {
      ObstTagConfig vp_cfg;
      if ( !YAML::getParam(vp_node_i, "id", vp_cfg.id) )
        throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'obstacle_tags[i].id'...");
      if ( !YAML::getParam(vp_node_i, "id", vp_cfg.name) )
        throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'obstacle_tags[i].id'...");
      if ( !YAML::getParam(vp_node_i, "center", vp_cfg.center) )
        throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'obstacle_tags[i].center'...");
      if ( !YAML::getParam(vp_node_i, "axes_len", vp_cfg.axes_len) )
        throw std::runtime_error(GmpMpcObstacles_fun_+"Failed to load param 'obstacle_tags[i].axes_len'...");
      obst_tag_cfg.push_back(vp_cfg);
    }

    // for (auto it = obst_tag_cfg.begin(); it!=obst_tag_cfg.end(); it++)
    // {
    //   std::cerr << "====> Tag id: " << it->id << "\n";
    //   for (int j=0; j<it->size(); j++)
    //   {
    //     std::cerr << "s: " << it->s[j] << ", offset: " << it->offset[j].t() << "\n";
    //   }
    // }

    // =========  load MPC settings ===========
    ExecResultMsg mcp_msg = ctrl->loadMpcParams(main_path + "/config/obstacles_params.yaml", mpc_cfg);
    if (mcp_msg.getType() != ExecResultMsg::INFO) throw std::runtime_error(mcp_msg.getMsg());
    
  }
  catch (std::exception &e)
  {
    ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, e.what() ));
    return init_success;;
  }

  // ========= Initialize MPC ========
  gmp_mpc.reset( new gmp_::GMP_MPC(gmp.get(), mpc_cfg.N_horizon, mpc_cfg.pred_time_step, mpc_cfg.N_kernels, mpc_cfg.kernels_std_scaling, 
                                   mpc_cfg.slack_gains, mpc_cfg.kernels_trunc_thres) );
  
  gmp_mpc->setObjCostGains(mpc_cfg.opt_pos_gain, mpc_cfg.opt_vel_gain);

  gmp_mpc->settings.time_limit = mpc_cfg.time_limit;
  gmp_mpc->settings.abs_tol = mpc_cfg.abs_tol;
  gmp_mpc->settings.rel_tol = mpc_cfg.rel_tol;

  gmp_mpc->setPosLimits(pos_lim.col(0), pos_lim.col(1));
  gmp_mpc->setVelLimits(vel_lim.col(0), vel_lim.col(1));
  gmp_mpc->setAccelLimits(accel_lim.col(0), accel_lim.col(1));

  gmp_mpc->setPosSlackLimit(mpc_cfg.slack_limits[0]);
  gmp_mpc->setVelSlackLimit(mpc_cfg.slack_limits[1]);
  gmp_mpc->setAccelSlackLimit(mpc_cfg.slack_limits[2]);

  init_success = true;
  
  return init_success;
}

std::vector<Obstacle> ObstaclesExperiment::getObstacles()
{
  std::vector<Obstacle> obstacles;
  for (auto it=obst_tag_cfg.begin(); it!=obst_tag_cfg.end(); it++)
  {
    apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(it->id);
    if (tag.tag_id < 0) continue;
    
    arma::vec tag_pos = { tag.pos.x(), tag.pos.y(), tag.pos.z() };
    arma::vec tag_quat = {tag.quat.w(), tag.quat.x(), tag.quat.y(), tag.quat.z()};
    obstacles.push_back( Obstacle(it->center, it->axes_len, tag_pos, tag_quat, it->name) );

    // it = obst_tag_cfg.erase(it); // this tag id is proccessed, so remove it from the list

    // std::cerr << "obst_tag_cfg.size() = " <<  obst_tag_cfg.size() << "\n";
    // std::cerr << "processed tag id: " << it->id << "\n";
  }

  // ------- For DEBUGGING --------
  #ifdef DEBUG_ON
  obstacles.clear();
  arma::vec center = {0.4170,  -0.2757,   0.1209};
  arma::mat Sigma = {{0.2492,  0.0134,  -0.0002}, {1.3370e-02, 4.0855e-02, -1.0252e-05}, {-0.0002, -0.0000, 0.0400}};

  obstacles.push_back(Obstacle(center, Sigma, "obstacle_14"));
  #endif

  return obstacles;
}

void ObstaclesExperiment::execute()
{
  init();

  // if (publish_obstacles) pub_obst_thread = std::thread([this](){ publishObstacles(); });

  // if (publish_path_online)
  // {
  //   pub_path->clearPath();
  //   pub_path->start(50);

  //   pub_ref_path->clearPath();
  //   pub_ref_path->start(50);
  // }

  if (!init_success)
  {
    ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, "Initialization was unsuccessful. Reinitialize to procceed..."));
    return;
  }

  Time_data.clear();
  P_data.clear();
  dP_data.clear();
  ddP_data.clear();
  target_data.clear();
  obstacles_data.clear();

  unsigned n_dof = 3;
  arma::vec ones_ndof = arma::vec().ones(n_dof);
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  arma::vec yg = this->target_pos;

  // Detect target for the first time
  int ntimes_target_not_detected = 0;
  while ( readTarget(yg, ntimes_target_not_detected) )
  {
    if (ntimes_target_not_detected > max_ntimes_target_not_detected)
    {
      ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, "Failed to detect target..."));
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  robot->update();
  arma::vec y0 = robot->getTaskPosition();

  double Tf = this->Tf;
  double t = 0;
  double dt = robot->getCtrlCycle();
  double s = 0;
  double s_dot = 1/Tf;
  double s_ddot = 0;
  arma::vec y = y0;
  arma::vec y_dot = O_ndof;
  arma::vec y_ddot = O_ndof;

  gmp_::CanonicalSystem can_sys(Tf);
  auto can_sys_fun = [&can_sys](double s, double s_dot){ return std::array<double,2>({s_dot, can_sys.getPhaseDDot(s,s_dot)}); };
  gmp_mpc->setCanonicalSystemFunction(can_sys_fun);

  gmp_mpc->setInitialState(y, y_dot, y_ddot, can_sys.s, can_sys.s_dot, 0);
  gmp_mpc->setFinalState(yg, O_ndof, O_ndof, 1, can_sys.s_dot, 0, mpc_cfg.final_state_err_tol);

  // gmp->setScaleMethod( gmp_::TrajScale::Ptr( new gmp_::TrajScale_Prop(n_dof) ) );
  gmp->setY0(y0);
  gmp->setGoal(yg);

  rviz_plot->init(mpc_cfg.N_kernels, mpc_cfg.kernels_std_scaling, y0, 50);

  arma::rowvec elaps_t_data;
  arma::wall_clock timer, timer2;
  timer.tic();

  ctrl->setMode(rw_::CART_VEL_CTRL);

  robot->update();

  bool interrupted = false;

  // =========  Simulation loop  =========
  while (true)
  {
    if (can_sys.s >= 1) break;

    if ( !ctrl->runOn() )
    {
      emit ctrl->gui->experimentStoppedSignal( ExecResultMsg(ExecResultMsg::INFO, "Execution was stopped...") );
      interrupted = true;
      break;
    }

    if ( !robot->isOk() )
    {
      emit ctrl->gui->experimentStoppedSignal( ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok:\n" + robot->getErrMsg() + "\n Aborting execution..." ) );
      interrupted = true;
      break;
    }

    int ret = readTarget(yg, ntimes_target_not_detected);
    if ( ret < 0 )
    {
      emit ctrl->gui->experimentStoppedSignal( ExecResultMsg(ExecResultMsg::ERROR, "Failed to detect target...") );
      interrupted = true;
      break;
    }

    gmp->setGoal(yg);
    gmp_mpc->setFinalState(yg, O_ndof, O_ndof, 1, can_sys.s_dot, 0, mpc_cfg.final_state_err_tol);

    std::vector<Obstacle> obstacles = getObstacles();
    for (const Obstacle& obst : obstacles) gmp_mpc->addEllipsoidObstacle(obst.c, obst.Sigma, obst.name);

    // updateRvizObstacles(obstacles);
    // pub_path->addPoint(y);
    // pub_ref_path->addPoint(gmp->getYd(can_sys.s));

    // --------  Stopping criteria  --------
    if (can_sys.s > 1.0) break;

    // std::cerr << "phase = " << can_sys.s << "\n";

    timer2.tic();
    gmp_::GMP_MPC::Solution sol = gmp_mpc->solve(can_sys.s, can_sys.s_dot);

    rviz_plot->operator()(gmp_mpc->log, obstacles);

    if (sol.exit_flag > 0) PRINT_WARNING_MSG(sol.exit_msg + "\n");
    else if (sol.exit_flag < 0)
    {
      PRINT_ERROR_MSG(sol.exit_msg + "\n");
      emit ctrl->gui->experimentStoppedSignal( ExecResultMsg(ExecResultMsg::ERROR, "Failed to find solution: " + sol.exit_msg) );
      interrupted = true;
      break;
    }

    y = sol.y;
    y_dot = sol.y_dot;
    y_ddot = sol.y_ddot;
    // arma::vec pos_slack_var = sol.pos_slack;
    // arma::vec vel_slack_var = sol.vel_slack;
    // arma::vec accel_slack_var = sol.accel_slack;
    elaps_t_data = arma::join_horiz( elaps_t_data, arma::vec({timer2.toc()*1000}) );

    arma::vec V_cmd = arma::vec().zeros(6);
    V_cmd.subvec(0,2) = y_dot;
    if (!use_sim) ctrl->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
    robot->update();
    
    // --------  Log data  --------
    Time_data = arma::join_horiz( Time_data, arma::vec({t}) );
    P_data = arma::join_horiz( P_data, y);
    dP_data = arma::join_horiz( dP_data, y_dot);
    ddP_data = arma::join_horiz( ddP_data, y_ddot);
    target_data = arma::join_horiz( target_data, yg);

    if (!obstacles.empty()) obstacles_data.push_back(obstacles);

    // if (!obstacles.empty())
    // {
    //   std::cerr << "yg = " << yg.t() << "\n";
    //   std::cerr << "obst_c = " << obstacles[0].c.t() << "\n";
    //   std::cerr << "obst_S = \n" << obstacles[0].Sigma << "\n";
    // }
    
    
    // --------  Numerical integration  --------
    can_sys.integrate(t, t+dt);
    t = t + dt;
  }

  ctrl->setMode(rw_::IDLE);

  if (!interrupted)
  {
    openGripper();
    emit ctrl->gui->experimentFinishSignal( ExecResultMsg(ExecResultMsg::INFO, "Handover completed!") );
  }

  this->publish_obstacles = false;
  if (pub_obst_thread.joinable()) pub_obst_thread.join();

  rviz_plot->stop();

  // pub_path->stop();
  // pub_ref_path->stop();

  double elaps_t_ms = timer.toc()*1000;
  PRINT_INFO_MSG("===> Finished! Elaps time: " + std::to_string(elaps_t_ms) + " ms\n");

  if (!elaps_t_data.empty())
  {
    double mean_elaps_t = arma::mean(elaps_t_data);
    double std_elaps_t = arma::stddev(elaps_t_data);
    double max_elaps_t = arma::max(elaps_t_data);
    double min_elaps_t = arma::min(elaps_t_data);

    std::cerr << "======= Elapsed time (ms) ======\n";
    std::cerr << "std_range: [" << std::max(0.,mean_elaps_t-std_elaps_t) << " -  " << mean_elaps_t + std_elaps_t <<"]\n";
    std::cerr << "mean     : " << mean_elaps_t << " +/- " << std_elaps_t <<"\n";
    std::cerr << "min      : " << min_elaps_t << "\n";
    std::cerr << "max      : " << max_elaps_t << "\n";
    std::cerr << "==============================\n";

    std::cerr << "======= Final state errors (m) ======\n";
    std::cerr << "pos_err = " << arma::norm(yg - y) << "\n";
    std::cerr << "vel_err = " << arma::norm(y_dot) << "\n";
    std::cerr << "accel_err = " << arma::norm(y_ddot) << "\n";
    std::cerr << "==============================\n";
  }

  // ========  save logged data  ===========
  if (!Time_data.empty())
  {
    int max_obst = obst_tag_cfg.size();
    int max_rows = 1 + (3 + 9) * max_obst; // 1: current num of obst, 3: center, 9: Sigma

    arma::mat obst_mat_data(max_rows, obstacles_data.size());
    for (int j=0; j<obstacles_data.size();j++)
    {
      std::vector<Obstacle> &obst_vec = obstacles_data[j];
      arma::mat obst_dat(3+9, max_obst);
      for (int k=0; k<obst_vec.size(); k++)
      {
        obst_dat.col(k) = arma::join_vert(obst_vec[k].c, arma::vectorise(obst_vec[k].Sigma));
      }
      obst_mat_data.col(j) = arma::join_vert(arma::vec({obst_vec.size()}), arma::vectorise(obst_dat));
    }


    try
    {
      gmp_::FileIO fid(exec_data_filename, gmp_::FileIO::out|gmp_::FileIO::trunc);
      fid.write("Time", Time_data);
      fid.write("P_data", P_data);
      fid.write("dP_data", dP_data);
      fid.write("ddP_data", ddP_data);
      fid.write("target_data", target_data);
      fid.write("obstacles_data", obst_mat_data);
      fid.write("pos_lim", pos_lim);
      fid.write("vel_lim", vel_lim);
      fid.write("accel_lim", accel_lim);
      fid.write("slack_limits", arma::vec({mpc_cfg.slack_limits[0], mpc_cfg.slack_limits[1], mpc_cfg.slack_limits[2]}));
    }
    catch (std::exception &e)
    {
      PRINT_ERROR_MSG(e.what());
      ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, e.what() ));
      return;
    }
  }
  
}


void ObstaclesExperiment::openGripper()
{
  gripper->move(80, 20);
}

// ==================== Obstacles utility functions =====================
void ObstaclesExperiment::updateRvizObstacles(const std::vector<Obstacle> &obstacles)
{
  std::unique_lock<std::mutex> lck(this->rviz_obst_mtx);
  this->rviz_obstacles = obstacles;
}

void ObstaclesExperiment::publishObstacles()
{
  static std::vector<std::string> visible_obst;
  rviz_::Color obst_color = rviz_::Color(1.0, 0.5, 0.0, 0.6);

  while (publish_obstacles)
  {
    for (auto &obst_name : visible_obst) rviz_pub->deleteMarkers(obst_name);
    visible_obst.clear();

    for (auto &obst : this->rviz_obstacles)
    {
      Eigen::Vector3d center(obst.c(0), obst.c(1), obst.c(2));
      Eigen::Matrix3d Sigma = Eigen::Map<Eigen::Matrix3d>(obst.Sigma.memptr());
      rviz_pub->publishEllipsoid(center, Sigma, obst_color, obst.name);
      visible_obst.push_back(obst.name);
    }
    rviz_pub->drawnow();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace gmp_mpc_ctrl_

