#include <gmp_mpc_controller/experiment/viapoints_experiment.h>

#include <gmp_mpc_controller/gmp_mpc_controller.h>


namespace gmp_mpc_ctrl_
{

#define GmpMpcViapoints_fun_ std::string("[GmpMpcViapoints_fun_::") + __func__ + "]: "

ViapointsExperiment::ViapointsExperiment(GmpMpcController *ctrl_)
{
  this->ctrl = ctrl_;

  this->main_path = ros::package::getPath("gmp_mpc_controller") + "/";
  this->params_filename = main_path + "/config/viapoints_params.bin";
  this->gmp_model_filename = main_path + "/config/viapoints_gmp_model.bin";
  this->exec_data_filename = main_path + "/data/viapoints_exec_data.bin";
}

bool ViapointsExperiment::init()
{
  init_success = false;

  robot = ctrl->robot;
  gripper = ctrl->main_ctrl->gripper.get();

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
    YAML::Node config = YAML::LoadFile(main_path + "/config/viapoints_params.yaml");

    if ( !YAML::getParam(config, "q_start", q_start) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'q_start'...");

    // load kinematic limits
    if ( !YAML::getParam(config, "pos_lim", pos_lim) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'pos_lim'...");
    if ( !YAML::getParam(config, "vel_lim", vel_lim) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'vel_lim'...");
    if (vel_lim.n_rows == 1) vel_lim = arma::repmat(vel_lim, 3, 1);
    if ( !YAML::getParam(config, "accel_lim", accel_lim) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'accel_lim'...");
    if (accel_lim.n_rows == 1) accel_lim = arma::repmat(accel_lim, 3, 1);


    if ( !YAML::getParam(config, "publish_via_points", publish_via_points) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'publish_via_points'...");

    // update the main controller's limits (so that any plots or visualization in rviz displays the correct limits)
    ctrl->setKinematicLimits(pos_lim, vel_lim, accel_lim);

    ctrl->setInitPose(q_start);

    if ( !YAML::getParam(config, "target_tag_id", target_tag_id) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'target_tag_id'...");

    if ( !YAML::getParam(config, "target_offset", target_offset) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'target_offset'...");

    if ( !YAML::getParam(config, "max_ntimes_target_not_detected", max_ntimes_target_not_detected) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'max_ntimes_target_not_detected'...");

    if ( !YAML::getParam(config, "Tf_per_meter", Tf_per_meter) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'Tf_per_meter'...");

    // ======== load vp_tags config ===========
    vp_tag_cfg.clear();

    YAML::Node vp_node;
    if ( !YAML::getParam(config, "viapoint_tags", vp_node) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'viapoint_tags'...");
    if ( ! vp_node.IsSequence() )
      throw std::runtime_error(GmpMpcViapoints_fun_+"'viapoint_tags' must be an array...");
    
    for (int i=0; i<vp_node.size(); i++)
    {
      VpTagConfig vp_cfg;

      const YAML::Node &vp_node_i = vp_node[i];
      
      int id;
      if ( !YAML::getParam(vp_node_i, "id", id) )
        throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'viapoint_tags[i].id'...");

      vp_cfg.id = id;

      YAML::Node vp_config_node;
      if ( !YAML::getParam(vp_node_i, "config", vp_config_node) )
      throw std::runtime_error(GmpMpcViapoints_fun_+"Failed to load param 'viapoint_tags[i].config'...");
      if ( ! vp_config_node.IsSequence() )
        throw std::runtime_error(GmpMpcViapoints_fun_+"'viapoint_tags[i].config' must be an array...");

      for (int j=0; j<vp_config_node.size(); j++)
      {
        const YAML::Node &vp_cfg_node_j = vp_config_node[j];
        if ( ! vp_cfg_node_j.IsMap() )
          throw std::runtime_error(GmpMpcViapoints_fun_+"'viapoint_tags[i].config[j]' must be a struct...");

        double s;
        arma::vec offset;
        if ( !YAML::getParam(vp_cfg_node_j, "s", s) ) throw std::runtime_error(GmpMpcViapoints_fun_+"viapoint_tags[i].config[j].s...");
        if ( !YAML::getParam(vp_cfg_node_j, "offset", offset) ) throw std::runtime_error(GmpMpcViapoints_fun_+"viapoint_tags[i].config[j].offset...");

        vp_cfg.offset.push_back(offset);
        vp_cfg.s.push_back(s);
      }

      vp_tag_cfg.push_back(vp_cfg);
      
    }

    // for (auto it = vp_tag_cfg.begin(); it!=vp_tag_cfg.end(); it++)
    // {
    //   std::cerr << "====> Tag id: " << it->id << "\n";
    //   for (int j=0; j<it->size(); j++)
    //   {
    //     std::cerr << "s: " << it->s[j] << ", offset: " << it->offset[j].t() << "\n";
    //   }
    // }

    // =========  load MPC settings ===========
    ExecResultMsg mcp_msg = ctrl->loadMpcParams(main_path + "/config/viapoints_params.yaml", mpc_cfg);
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

int ViapointsExperiment::readTarget(arma::vec &yg, int &ntimes_target_not_detected)
{
  apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(target_tag_id);
  if (tag.tag_id < 0) ++ntimes_target_not_detected;
  else
  {
    ntimes_target_not_detected = 0;
    yg = { tag.pos.x(), tag.pos.y(), tag.pos.z() };
    yg += target_offset;
  }

  if (ntimes_target_not_detected == 0) return 0; // found target
  else if (ntimes_target_not_detected < max_ntimes_target_not_detected) return 1; // didn't find target, but iterations not exceeded
  else return -1; // iterations to find target exceeded
}

std::vector<ViapointsExperiment::Viapoint> ViapointsExperiment::getViaPoints()
{
  std::vector<Viapoint> via_points;
  auto it = vp_tag_cfg.begin();
  while ( it!=vp_tag_cfg.end() )
  {
    apriltag_ros::AprilTagListener::TagDetection tag = tag_listener->getTagDetection(it->id);
    if (tag.tag_id < 0)
    {
      it++;
      continue;
    }
    
    arma::vec pos = { tag.pos.x(), tag.pos.y(), tag.pos.z() };
    for (int i=0; i<it->size(); i++) via_points.push_back( Viapoint(it->s[i], pos+it->offset[i]) );

    it = vp_tag_cfg.erase(it); // this tag id is proccessed, so remove it from the list

    // std::cerr << "vp_tag_cfg.size() = " <<  vp_tag_cfg.size() << "\n";
    // std::cerr << "processed tag id: " << it->id << "\n";
  }

  return via_points;
}

void ViapointsExperiment::execute()
{
  init();

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
  via_points_data.clear();

  unsigned n_dof = 3;
  arma::vec ones_ndof = arma::vec().ones(n_dof);
  arma::vec O_ndof = arma::vec().zeros(n_dof);

  arma::vec yg = robot->getTaskPosition();
  int ntimes_target_not_detected = 0;

  // Detect target for the first time
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

  double Tf = arma::norm(yg - y0)*Tf_per_meter; // 4 sec per meter
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

    std::vector<Viapoint> via_points = getViaPoints();
    for (const Viapoint& vp : via_points) gmp_mpc->addViaPoint(vp.s, vp.pos, vp.err_tol);

    // --------  Stopping criteria  --------
    if (can_sys.s > 1.0) break;

    // std::cerr << "phase = " << can_sys.s << "\n";

    timer2.tic();
    gmp_::GMP_MPC::Solution sol = gmp_mpc->solve(can_sys.s, can_sys.s_dot);

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
    ctrl->setTaskVelocity(V_cmd, y, robot->getTaskOrientation());
    robot->update();
    
    // --------  Log data  --------
    Time_data = arma::join_horiz( Time_data, arma::vec({t}) );
    P_data = arma::join_horiz( P_data, y);
    dP_data = arma::join_horiz( dP_data, y_dot);
    ddP_data = arma::join_horiz( ddP_data, y_ddot);
    target_data = arma::join_horiz( target_data, yg);

    if (!via_points.empty())
    {
      arma::mat temp_vp(5, via_points.size() );
      for (int j=0; j<via_points.size(); j++)
      {
        const Viapoint &vp = via_points[j];
        temp_vp.col(j) = {t, vp.s, vp.pos(0), vp.pos(1), vp.pos(2)};
      }
      via_points_data = arma::join_horiz( via_points_data, temp_vp );
    }
    
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


  double elaps_t_ms = timer.toc()*1000;
  PRINT_INFO_MSG("===> Handover finished! Elaps time: " + std::to_string(elaps_t_ms) + " ms\n");

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
    try
    {
      gmp_::FileIO fid(exec_data_filename, gmp_::FileIO::out|gmp_::FileIO::trunc);
      fid.write("Time", Time_data);
      fid.write("P_data", P_data);
      fid.write("dP_data", dP_data);
      fid.write("ddP_data", ddP_data);
      fid.write("target_data", target_data);
      fid.write("via_points_data", via_points_data);
      fid.write("pos_lim", pos_lim);
      fid.write("vel_lim", vel_lim);
      fid.write("accel_lim", accel_lim);
    }
    catch (std::exception &e)
    {
      PRINT_ERROR_MSG(e.what());
      ctrl->sendGuiMsg(ExecResultMsg(ExecResultMsg::ERROR, e.what() ));
      return;
    }
  }
  
}


void ViapointsExperiment::openGripper()
{
  gripper->move(80, 20);
}

} // namespace gmp_mpc_ctrl_

