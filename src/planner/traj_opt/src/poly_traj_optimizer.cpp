
#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, const bool use_formation)
  {
    // ztr_log3<<"in optimizeTrajectory_lbfgs"<<endl;
    init_start_zy[0] = iniState(0, 0);
    init_start_zy[1] = iniState(1, 0);
    init_start_zy[2] = iniState(2, 0);
    init_end_zy[0] = finState(0, 0);
    init_end_zy[1] = finState(1, 0);
    init_end_zy[2] = finState(2, 0);

    // if(drone_id_==2)
    // {
    //   std::cout<<"start_part::"<<init_start_zy<<std::endl;
    // std::cout<<"end_part::"<<init_end_zy<<std::endl;
    // std::cout<<"start::"<<iniState<<std::endl;
    // std::cout<<"end::"<<finState<<std::endl;
    // }

    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);
    // jerkOpt_.generate(iniState, initT);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    ros::Time t0 = ros::Time::now(), t1, t2;
    bool use_formation_temp = use_formation_;

    double q[variable_num_];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    Eigen::Map<Eigen::VectorXd> Vt(q + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.0;      // 0.1
    lbfgs_params.min_step = 1e-28;     // 1e-32
    lbfgs_params.max_linesearch = 120; // default:60
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-4;
    lbfgs_params.line_search_type = 0;

    /* trick : for real-time optimization */
    if (use_formation)
    {
      // consider formation
      // so we use less iterations for real time optimization
      lbfgs_params.max_iterations = 100; // 20
    }
    else
    {
      // do not consider formation
      // so we use more iterations for more precise optimization results
      lbfgs_params.max_iterations = 100;
      use_formation_ = false;
    }

    // calculate in advance a part of swarm graph in other agents' local traj for each replanning
    opt_local_min_loop_sum_num_ = 0;
    if (enable_fix_step_)
      setSwarmGraphInAdavanced(initT);
    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */
    t1 = ros::Time::now();
    int result = lbfgs::lbfgs_optimize(variable_num_,
                                       q,
                                       &final_cost,
                                       PolyTrajOptimizer::costFunctionCallback,
                                       NULL,
                                       PolyTrajOptimizer::earlyExitCallback,
                                       this,
                                       &lbfgs_params);

    use_formation_ = use_formation_temp;
    // ztr_log3<<"after lbfgs_optimize"<<endl;
    t2 = ros::Time::now();
    double optimize_time_ms = (t2 - t1).toSec() * 1000;
    double total_time_ms = (t2 - t0).toSec() * 1000;

    // debug
    // cout << "[debug] final total: " << jerkOpt_.get_T1().sum() << ", T: " << jerkOpt_.get_T1().transpose() << endl;

    // printf("\033[32m[Optimization]: iter=%d, use_formation=%d, optimize time(ms)=%5.3f, total time(ms)=%5.3f, graph num=%i \n\033[0m", iter_num_, use_formation, optimize_time_ms, total_time_ms, opt_local_min_loop_sum_num_);

    // print the optimization result
    // if (result != lbfgs::LBFGS_STOP && result != lbfgs::LBFGSERR_MAXIMUMITERATION)
    //   printf("\033[33m[Optimization]: %s\n\033[0m", lbfgs::lbfgs_strerror(result));

    // optimal_points for visulization
    if (!enable_fix_step_)
      optimal_points = cps_.points;
    else
      optimal_points = time_cps_.points;

    if (enable_fix_step_)
      time_cps_.resetBuffer();

    // check collision
    bool occ = false;
    double collision_relative_time;
    occ = checkCollision(collision_relative_time);
    return true; // shit
    if (occ)
    {
      printf("\033[31m[Optimization]: Failed! Trajectory is collied in the relative time=%5.3f \n\033[0m", collision_relative_time);
      return false;
    }
    else
      return true;
  }
  // void PolyTrajOptimizer::formation_error_callback()
  //   {
  //     Eigen::Vector3d formation_error_pos;
  //     double t_cur_global = ros::Time::now().toSec();
  //     if(swarm_trajs_->size() == formation_size_all)
  //     {
  //     log_zy2<<"[[";

  //       for(int i=0;i<formation_size_all;i++)
  //       {
  //         ROS_ERROR("current traj in collision, replan.@@@@@@@@@@@@@@@@@@@@@@@@@@");
  //         formation_error_pos = swarm_trajs_->at(i).traj.getPos(0.0);
  //         ROS_ERROR("current traj in collision, replan.##########################");

  //         log_zy2<<formation_error_pos[0]<<","<<formation_error_pos[1]<<","<<formation_error_pos[2]<<","<<"]"<<",";
  //       }
  //     log_zy2<<"]"<<std::endl;

  //     }
  //   }
  void PolyTrajOptimizer::setSwarmGraphInAdavanced(const Eigen::VectorXd initT)
  {
    is_set_swarm_advanced_ = false;
    if (use_formation_)
    {

      // int size = swarm_trajs_->size();
      // if (drone_id_ == formation_size_all - 1)
      //   size = formation_size_all;
      // if (size < formation_size_all)
      //   return;
      // return false;
      int size = swarm_trajs_->size();
      if (drone_id_ == sparse_id.back() && size < drone_id_)
        size++;
      if (size < sparse_id.back() - 1)
        return;
      int fake_num = graph_size;
      if (drone_id_ == sparse_id.back())
      {
        fake_num--;
      }
      if (fake_num > swarm_trajs_->size())
      {
        return;
      }
      for (int i = 0, fake_id = 0; i < fake_num; i++)
      {

        if (i == sparse_id[fake_id] && swarm_trajs_->at(i).drone_id < 0 && sparse_id[fake_id] != drone_id_)
        {
          return;
        }

        if (i == sparse_id[fake_id])
        {
          fake_id++;
        }
      }
      time_cps_.start_time = t_now_;

      // calculate the effective time duration
      time_cps_.duration = initT.sum();
      time_cps_.sampling_num = floor(time_cps_.duration / time_cps_.sampling_time_step) + 1; // should be >= 1

      // calculate the effective time duration of swarm
      // time_cps_.relative_time_ahead.resize(size);
      time_cps_.relative_time_ahead.resize(sparse_id.size());
      for (int id = 0; id < sparse_id.size(); id++)
      {
        if (drone_id_ == sparse_id[id])
          time_cps_.relative_time_ahead(id) = 0.0;
        else
          time_cps_.relative_time_ahead(id) = time_cps_.start_time - swarm_trajs_->at(sparse_id[id]).start_time;

        // if (id != id_in_group)
        //   cout << "!time_relative::::::::::::::::::::::::::::::" << time_cps_.start_time - swarm_trajs_->at(id).start_time << endl;
      }

      // calculate in advance for swarm graph
      time_cps_.swarm_graph_advance_sets.resize(time_cps_.sampling_num);
      time_cps_.swarm_pos_advance_sets.resize(time_cps_.sampling_num);
      time_cps_.swarm_vel_advance_sets.resize(time_cps_.sampling_num);

      double accumulated_step_swarm(0.0);
      static const double step = time_cps_.sampling_time_step;
      max_cline_id_.clear();
      for (int i = 0; i < time_cps_.sampling_num;
           i++, accumulated_step_swarm += time_cps_.sampling_time_step)
      {

        SwarmGraph swarm_graph;
        /**************************************************************************************************************/
        std::vector<Eigen::Vector3d> graph_des_opt(sparse_id.size());

        for (int i = 0, fake_id = 0; i < swarm_des_opt.size(); i++)
        {
          if (i == sparse_id[fake_id])
          {
            graph_des_opt[fake_id] = swarm_des_opt[i];
            fake_id++;
          }
        }
        int debug_num = sparse_id.size() - 1;
        std::vector<int> adj_in_cline, adj_out_cline;
        for (int i = 0; i < debug_num + 1; i++)
        {
          for (int j = 0; j < i; j++)
          {
            adj_in_cline.push_back(i);
            adj_out_cline.push_back(j);
          }
        }

        Eigen::VectorXi assignment_cline = Eigen::VectorXi::LinSpaced(debug_num + 1, 0, debug_num + 1 - 1);
        swarm_graph.setnewform();
        swarm_graph.setDesiredForm(graph_des_opt, adj_in_cline, adj_out_cline);
        swarm_graph.setAssignment(assignment_cline);
        /**************************************************************************************************************************************** */

        std::vector<Eigen::Vector3d> swarm_pos(sparse_id.size()), swarm_vel(sparse_id.size());
        // get swarm pos and vel
        for (int id = 0; id < sparse_id.size(); id++)
        {
          Eigen::Vector3d pos, vel;
          if (sparse_id[id] == drone_id_)
          {
            // own pos and vel, set as zero now. And they will be updated in the optimized loop
            pos = Eigen::Vector3d::Zero();
            vel = Eigen::Vector3d::Zero();
          }
          else
          {
            // others pos and vel
            double t_in_st = time_cps_.relative_time_ahead(id) + accumulated_step_swarm;
            if (t_in_st < swarm_trajs_->at(sparse_id[id]).duration)
            {
              pos = swarm_trajs_->at(sparse_id[id]).traj.getPos(t_in_st);
              vel = swarm_trajs_->at(sparse_id[id]).traj.getVel(t_in_st);
            }
            else
            {
              double exceed_time = t_in_st - swarm_trajs_->at(sparse_id[id]).duration;
              vel = swarm_trajs_->at(sparse_id[id]).traj.getVel(swarm_trajs_->at(sparse_id[id]).duration);
              pos = swarm_trajs_->at(sparse_id[id]).traj.getPos(swarm_trajs_->at(sparse_id[id]).duration);
            }
          }
          swarm_pos[id] = pos;
          swarm_vel[id] = vel;
        }
        time_cps_.swarm_pos_advance_sets[i] = swarm_pos;
        time_cps_.swarm_vel_advance_sets[i] = swarm_vel;
        time_cps_.swarm_graph_advance_sets[i] = swarm_graph;
      }

      if (time_cps_.enable_decouple_swarm_graph)
      {
        // test : calculate the local minimum of swarm graph
        time_cps_.local_min_advance_sets.resize(time_cps_.sampling_num);

        // record

        for (int i = 0; i < time_cps_.sampling_num; i++)
        {

          // init optimizer
          lbfgs::lbfgs_parameter_t lbfgs_params;
          lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
          lbfgs_params.mem_size = 8;
          lbfgs_params.g_epsilon = 1e-5; // 0.0
          lbfgs_params.min_step = 1e-10; // 1e-32
          lbfgs_params.delta = 1.0e-4;
          lbfgs_params.line_search_type = 1; // 1: continues
          lbfgs_params.max_iterations = 20;

          /* get init value */
          double q[3];

          // Average of other swarm pos, if there is any other more efficient trick?
          Eigen::Vector3d average_pos = Eigen::Vector3d::Zero();
          for (int id = 0; id < sparse_id.size(); id++)
          {
            if (sparse_id[id] != drone_id_)
            {
              average_pos += time_cps_.swarm_pos_advance_sets[i][id];
            }
          }
          average_pos /= (sparse_id.size() - 1);
          q[0] = average_pos(0);
          q[1] = average_pos(1);
          q[2] = average_pos(2);

          /* optimize */
          double final_cost;
          time_cps_.idx_of_sets = i;
          opt_local_min_loop_num_ = 0;
          int result = lbfgs::lbfgs_optimize(3,
                                             q,
                                             &final_cost,
                                             PolyTrajOptimizer::swarmGraphCostCallback,
                                             NULL,
                                             NULL,
                                             this,
                                             &lbfgs_params);
          // save the local minimum
          Eigen::Vector3d local_min;
          if (is_opt_formation_z_)
            local_min << q[0], q[1], q[2];
          else
            local_min << q[0], q[1], fixed_formation_z_;
          time_cps_.local_min_advance_sets[i] = local_min;
          // if (drone_id_ == 7)
          //   cout << "local_min::" << local_min << endl;

          // debug: print the optimization result
          // if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_ALREADY_MINIMIZED)
          //   printf("\033[33m[advanced] idx: %i, result: %s\n\033[0m", time_cps_.idx_of_sets, lbfgs::lbfgs_strerror(result));
        }

        if (record_once_)
        {
          record_once_ = false;
          log_.close();
        }
      }

      is_set_swarm_advanced_ = true;
    }
    // ztr_log3<<"is_set_swarm_advanced_ true"<<endl;
    return;
  }

  double PolyTrajOptimizer::swarmGraphCostCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Vector3d local_min_pos;
    local_min_pos << x[0], x[1], x[2];

    /* calculate cost and grad of swarm graph */
    double cost;
    Eigen::Vector3d gradP;
    int idx = opt->time_cps_.idx_of_sets;
    int id_m = opt->time_cps_.sampling_num;
    int id_in_cline = opt->id_in_group;
    vector<Eigen::Vector3d> swarm_pos;
    // std::vector<Eigen::Vector3d> swarm_des_cline;
    Eigen::VectorXi max_cline_id_m = opt->max_cline_id_m_z; ////debug
    // std::cout<<"max_cline_id_m::"<<max_cline_id_m.size()<<std::endl;
    // int debug_num = opt->max_cline_id_[idx].size();
    int debug_num = opt->sparse_id.size();
    std::vector<Eigen::Vector3d> graph_des_opt(opt->sparse_id.size());
    // swarm_des_cline.resize(debug_num);
    if (true)
    {
      swarm_pos.resize(debug_num);
      for (int id = 0; id < opt->sparse_id.size(); id++)
      {
        Eigen::Vector3d pos;
        if (opt->sparse_id[id] == opt->drone_id_)
        {
          // own pos and vel, set as zero now. And they will be updated in the optimized loop
          pos = local_min_pos;
          id_in_cline = id;
        }
        else
        {
          // others pos and vel
          pos = opt->time_cps_.swarm_pos_advance_sets[idx][id];
        }
        swarm_pos[id] = pos;
      }
    }
    else
    {
      // get swarm pos
      swarm_pos.resize(opt->formation_size_);
      for (int id = 0; id < opt->formation_size_; id++)
      {
        if (id == opt->id_in_group)
        {
          swarm_pos[id] = local_min_pos;
          id_in_cline = opt->id_in_group;
        }
        else
        {
          swarm_pos[id] = opt->time_cps_.swarm_pos_advance_sets[idx][id];
        }
      }
    }
    // get cost and gradP
    opt->time_cps_.swarm_graph_advance_sets[idx].updatePartGraphAndGetGrad(id_in_cline, swarm_pos, gradP);
    opt->time_cps_.swarm_graph_advance_sets[idx].calcFNorm2(cost);
    grad[0] = gradP(0);
    grad[1] = gradP(1);
    grad[2] = gradP(2);

    // record
    if (opt->record_once_)
    {
      opt->log_ << opt->opt_local_min_loop_num_ << ", pos: " << local_min_pos.transpose();
      opt->log_ << ", cost: " << cost << ", grad: " << -gradP.transpose() << endl;
    }

    opt->opt_local_min_loop_num_++;
    opt->opt_local_min_loop_sum_num_++;

    return cost;
  }

  bool PolyTrajOptimizer::checkCollision(double &collision_relative_time)
  {
    /* check the safety of trajectory */
    double T_end;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    if (enable_fix_step_)
    {
      // fixed time step sampling
      double T_all = traj.getTotalDuration();
      double step = time_cps_.sampling_time_step;
      int k = floor(T_all / step) + 1;
      T_end = (k * 2.0 / 3.0) * step;
    }
    else
    {
      // fixed point number sampling
      int N = traj.getPieceNum();
      int k = cps_num_prePiece_ * N + 1;
      int idx = k / 3 * 2;
      int piece_of_idx = floor((idx - 1) / cps_num_prePiece_);
      Eigen::VectorXd durations = traj.getDurations();
      T_end = durations.head(piece_of_idx).sum() + durations(piece_of_idx) * (idx - piece_of_idx * cps_num_prePiece_) / cps_num_prePiece_;
    }

    bool occ = false;
    double dt = 0.01;
    int i_end = floor(T_end / dt);
    double t = 0.0;
    double T_all = traj.getTotalDuration();
    collision_relative_time = 0.0;
    collision_check_time_end_ = T_end;

    for (int i = 0; i < i_end; i++)
    {
      Eigen::Vector3d pos = traj.getPos(t);
      // cout<<"--------pose----------------"<<endl;
      // cout<<pos.transpose()<<endl;
      if (grid_map_->getInflateOccupancy(pos) == 1)
      {
        collision_relative_time = t / T_all;
        // prevent collision check being too strict
        if (collision_relative_time < 0.6)
        {
          occ = true;
          break;
        }
      }
      t += dt;
    }
    // cout << "[checkCollision] T_end: " << T_end << ", occ: " << occ << ", t: " << t/T_all << endl;
    return occ;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;

    opt->jerkOpt_.generate(P, T);

    // debug
    // if (opt->iter_num_ == 0)
    // cout << "[debug] init total: " << T.sum() << ", T: " << T.transpose() << endl;

    if (T.sum() > 20.00)
    {
      gradP.setZero();
      gradt.setZero();
      return 9999999.0;
    }

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    Eigen::VectorXd obs_swarm_feas_qvar_costs;

    if (opt->enable_fix_step_)
    {
      obs_swarm_feas_qvar_costs.resize(5);
      opt->addPVAGradCost2CTwithFixedTimeSteps(gradT, obs_swarm_feas_qvar_costs);
    }
    else
    {
      obs_swarm_feas_qvar_costs.resize(6);
      opt->addPVAGradCost2CTwithFixedCtrlPoints(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_);
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    // debug
    // double sum_cost = smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
    // if (opt->drone_id_ == 0){
    //   // double cost_sum = obs_swarm_feas_qvar_costs.sum();
    //   cout << "----iter_num_ : " << opt->iter_num_ << "  ----cost sum : " << sum_cost << " ----"<< endl;
    //   cout << "collision cost           : " << obs_swarm_feas_qvar_costs(0) << " , " << 100*obs_swarm_feas_qvar_costs(0)/sum_cost << "%" << endl;
    //   // cout << "swarm collision cost     : " << obs_swarm_feas_qvar_costs(1) << " , " << 100*obs_swarm_feas_qvar_costs(1)/cost_sum << "%" << endl;
    //   cout << "swarm formation cost     : " << swarm_cost << " , " << 100* swarm_cost / sum_cost << "%" << endl;
    //   // cout << "swarm formation cost : " << obs_swarm_feas_qvar_costs(2) << " , " << 100*obs_swarm_feas_qvar_costs(2)/sum_cost << "%" << endl;
    //   // cout << "dynamic fisibility cost  : " << obs_swarm_feas_qvar_costs(4) << " , " << 100*obs_swarm_feas_qvar_costs(4)/cost_sum << "%" << endl;
    //   // cout << "quratic variance cost    : " << obs_swarm_feas_qvar_costs(5) << " , " << 100*obs_swarm_feas_qvar_costs(5)/cost_sum << "%" << endl;
    // }

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g,
                                           const double fx, const double xnorm, const double gnorm,
                                           const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
    gdT *= wei_smooth_;
    cost *= wei_smooth_;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CTwithFixedCtrlPoints(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0;
    // Eigen::MatrixXd constrain_pts(3, N * K + 1);

    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        /* Formation method chosing */
        if (use_formation_)
        {
          switch (formation_method_type_)
          {
          case FORMATION_METHOD_TYPE::SWARM_GRAPH:
          {
            // deformale formation
            if (swarmGraphGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
            {
              gradViolaPc = beta0 * gradp.transpose();
              gradViolaPt = alpha * gradt;
              jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
              gdT(i) += omg * (costp / K + step * gradViolaPt);
              if (i > 0)
              {
                gdT.head(i).array() += omg * step * grad_prev_t;
              }
              costs(2) += omg * step * costp;
            }
            break;
          }

          case FORMATION_METHOD_TYPE::LEADER_POSITION:
          {
            // benchmark: leader-follower position-based formation
            if (leaderPosFormationCostGradP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
            {
              gradViolaPc = beta0 * gradp.transpose();
              gradViolaPt = alpha * gradt;
              jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
              gdT(i) += omg * (costp / K + step * gradViolaPt);
              if (i > 0)
              {
                gdT.head(i).array() += omg * step * grad_prev_t;
              }
              costs(3) += omg * step * costp;
            }
            break;
          }

          case FORMATION_METHOD_TYPE::RELATIVE_POSITION:
          {
            // benchmark: relative position-based formation
            if (relativePosFormationCostGradP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
            {
              gradViolaPc = beta0 * gradp.transpose();
              gradViolaPt = alpha * gradt;
              jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
              gdT(i) += omg * (costp / K + step * gradViolaPt);
              if (i > 0)
              {
                gdT.head(i).array() += omg * step * grad_prev_t;
              }
              costs(3) += omg * step * costp;
            }
            break;
          }

          case FORMATION_METHOD_TYPE::VRB_METHOD:
          {
            // benchmark: VRB-method
          }

          default:
            break;
          }
        }

        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(4) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(4) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(5) += var;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CTwithFixedTimeSteps(EIGENVEC &gdT, Eigen::VectorXd &costs)
  {
    getFixedTimePos();
    costs.setZero();
    if (id_in_group == 0)
    {
      vector<Eigen::Vector3d> swarm_graph_pos_zy;
      Eigen::Vector3d pos_zy(0.0, 0.0, 0.0);
      getFormationPos(swarm_graph_pos_zy, pos_zy);
    }
    double obs_cost = obstacleGradCostP(gdT);
    costs(0) = obs_cost;

    double swarm_obs_cost = swarmGradCostP(gdT);
    costs(1) = swarm_obs_cost;

    double swarm_formation_cost = 0.0;
    if (opt_ok && (!clc_form_error(init_start_zy) || use_leader_))
    // if (false)
    {
      swarm_formation_cost = leaderGradCostP(gdT);
      // std::cout << "!!#######################:::: " << swarm_formation_cost << std::endl;
      if ((ros::Time::now() - number_change_time).toSec() > 0.5)
      {
        number_change = false;
      }
    }
    else
    {
      // swarm_formation_cost = swarmGraphGradCostP(gdT);
      if (use_graph_)
      {
        swarm_formation_cost = swarmGraphGradCostP(gdT);
      }
      cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@!!!!!!!!"<< endl;
      // std::cout<<"!!#######################:::: "<<swarm_formation_cost <<std::endl;
    }
    costs(2) = swarm_formation_cost;
    if (formation_change_get)
    {
      // std::cout<<"formation_change_get!!!!!!!!!!"<<std::endl;
      formation_change_get = false;
    }

    double vel_cost = 0.0, acc_cost = 0.0;
    feasibilityGradCostVandA(gdT, vel_cost, acc_cost);
    costs(3) = vel_cost;
    costs(4) = acc_cost;
  }

  void PolyTrajOptimizer::getFixedTimePos()
  {
    // for visualization
    static const double step = time_cps_.sampling_time_step;
    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;
    // time_cps_.points.resize(3, k+2);    // k sampling points and start/end points
    time_cps_.points.resize(3, k + 1); // k sampling points and start points

    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0;
    Eigen::Vector3d pos;
    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      pos = c.transpose() * beta0;
      time_cps_.points.col(idx) = pos;
    }

    // final pos: maybe has bugs
    // s1 = jerkOpt_.get_T1().sum();
    // piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
    // const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);
    // s2 = s1 * s1;
    // s3 = s2 * s1;
    // s4 = s2 * s2;
    // s5 = s4 * s1;
    // beta0 << 1.0, s1, s2, s3, s4, s5;
    // pos = c.transpose() * beta0;
    // time_cps_.points.col(k+1) = pos;
  }

  double PolyTrajOptimizer::obstacleGradCostP(Eigen::VectorXd &gdT)
  {
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    /* Calculate the gdC, gdT and obs_cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double obs_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {
      // if (idx >= k * 2 / 3)
      //   break;

      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;

      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // calculate the cost and grad of obstacle avoidance
      double dJ_df, df_dt, grad_prev_t;
      Eigen::Matrix<double, 6, 3> gradViolaPc;
      Eigen::Vector3d df_dp;

      // use esdf
      double dist;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGrad(pos, dist, dist_grad);
      double dist_err = obs_clearance_ - dist;

      if (dist_err > 0)
      {
        dJ_df = 3 * wei_obs_ * step * omg * pow(dist_err, 2);
        df_dp = -dist_grad;
        gradViolaPc = dJ_df * beta0 * df_dp.transpose();
        df_dt = df_dp.dot(vel);
        grad_prev_t = -1 * dJ_df * df_dt;

        // cost
        obs_cost += wei_obs_ * step * omg * pow(dist_err, 3);

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }
    }
    return obs_cost;
  }

  double PolyTrajOptimizer::swarmGradCostP(Eigen::VectorXd &gdT)
  {
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    /* Calculate the gdC, gdT and swarm_obs_cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double swarm_obs_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {
      if (idx >= k * 2 / 3)
        break;

      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;

      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      int size = swarm_trajs_->size();
      for (int id = 0; id < size; id++)
      {
        if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
          continue;

        // get swarm pos
        double t_in_st = accumulated_dur + (t_now_ - swarm_trajs_->at(id).start_time);
        Eigen::Vector3d swarm_p, swarm_v;
        if (t_in_st < swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(t_in_st);
          swarm_v = swarm_trajs_->at(id).traj.getVel(t_in_st);
        }
        else
        {
          double exceed_time = t_in_st - swarm_trajs_->at(id).duration;
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) + exceed_time * vel;
        }

        // calculate the cost and grad of swarm reciprocal avoidance
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        Eigen::Vector3d dist_vec = pos - swarm_p;
        double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
        double dist2_err = CLEARANCE2 - ellip_dist2;
        double dist2_err2 = dist2_err * dist2_err;
        double dist2_err3 = dist2_err2 * dist2_err;

        if (dist2_err > 0)
        {
          dJ_df = 3 * wei_swarm_ * step * omg * dist2_err2;
          df_dp = -2 * dist_vec;
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          df_dt = df_dp.dot(vel);
          grad_prev_t = -1 * dJ_df * df_dt;

          // cost
          swarm_obs_cost += wei_swarm_ * step * omg * dist2_err3;

          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
      }
    }
    return swarm_obs_cost;
  }

  double PolyTrajOptimizer::swarmGraphGradCostP(Eigen::VectorXd &gdT)
  {
    if (!is_set_swarm_advanced_)
      return 0.0;

    int size = swarm_trajs_->size();
    if (drone_id_ == sparse_id.back() && size < drone_id_)
      size++;
    if (size < sparse_id.back() - 1)
      return false;
    int fake_num = graph_size;
    if (drone_id_ == sparse_id.back())
    {
      fake_num--;
    }
    if (fake_num > swarm_trajs_->size())
    {
      return false;
    }
    for (int i = 0, fake_id = 0; i < fake_num; i++)
    {

      if (i == sparse_id[fake_id] && swarm_trajs_->at(i).drone_id < 0 && sparse_id[fake_id] != drone_id_)
      {
        return false;
      }

      if (i == sparse_id[fake_id])
      {
        fake_id++;
      }
    }

    static const double step = time_cps_.sampling_time_step;
    cline_begin = false;
    double T_sum = jerkOpt_.get_T1().sum();
    int k;

    /* Expand swarm_graph_advance_sets */
    if (T_sum > time_cps_.duration)
    {
      k = floor(T_sum / step) + 1;
      for (int i = time_cps_.sampling_num; i < k; i++)
      {
        SwarmGraph swarm_graph;

        double time_start = 0;
        std::vector<Eigen::Vector3d> swarm_des_zy(formation_size_);
        /**************************************************************************************************************/
        std::vector<Eigen::Vector3d> graph_des_opt(sparse_id.size());

        for (int i = 0, fake_id = 0; i < swarm_des_opt.size(); i++)
        {
          if (i == sparse_id[fake_id])
          {
            graph_des_opt[fake_id] = swarm_des_opt[i];
            fake_id++;
          }
        }
        int debug_num = sparse_id.size() - 1;
        std::vector<int> adj_in_cline, adj_out_cline;
        for (int i = 0; i < debug_num + 1; i++)
        {
          for (int j = 0; j < i; j++)
          {
            adj_in_cline.push_back(i);
            adj_out_cline.push_back(j);
          }
        }

        Eigen::VectorXi assignment_cline = Eigen::VectorXi::LinSpaced(debug_num + 1, 0, debug_num + 1 - 1);
        swarm_graph.setnewform();
        swarm_graph.setDesiredForm(graph_des_opt, adj_in_cline, adj_out_cline);
        swarm_graph.setAssignment(assignment_cline);
        /**************************************************************************************************************************************** */
        std::vector<Eigen::Vector3d> swarm_pos(sparse_id.size()), swarm_vel(sparse_id.size());
        // get swarm pos and vel
        for (int id = 0; id < sparse_id.size(); id++)
        {
          Eigen::Vector3d pos, vel;
          if (sparse_id[id] == drone_id_)
          {
            // own pos and vel, set as zero now. And they will be updated in the optimized loop
            pos = Eigen::Vector3d::Zero();
            vel = Eigen::Vector3d::Zero();
          }
          else
          {
            // others pos and vel
            double t_in_st = time_cps_.relative_time_ahead(id) + i * step;
            if (t_in_st < swarm_trajs_->at(sparse_id[id]).duration)
            {
              pos = swarm_trajs_->at(sparse_id[id]).traj.getPos(t_in_st);
              vel = swarm_trajs_->at(sparse_id[id]).traj.getVel(t_in_st);
            }
            else
            {
              double exceed_time = t_in_st - swarm_trajs_->at(sparse_id[id]).duration;
              vel = swarm_trajs_->at(sparse_id[id]).traj.getVel(swarm_trajs_->at(sparse_id[id]).duration);
              pos = swarm_trajs_->at(sparse_id[id]).traj.getPos(swarm_trajs_->at(sparse_id[id]).duration);
            }
          }
          swarm_pos[id] = pos;
          swarm_vel[id] = vel;
        }

        time_cps_.swarm_pos_advance_sets.emplace_back(swarm_pos);
        time_cps_.swarm_vel_advance_sets.emplace_back(swarm_vel);
        // set swarm graph, need to change: calculate the local optimal pos of swarm graph
        // swarm_graph.updateGraph(swarm_pos);
        time_cps_.swarm_graph_advance_sets.emplace_back(swarm_graph);
      }

      std::vector<Eigen::Vector3d> swarm_des_zy_;
      if (time_cps_.enable_decouple_swarm_graph)
      {
        for (int i = time_cps_.sampling_num; i < k; i++)
        {
          // get_changeformation(swarm_des_zy_, kkk-1, k-time_cps_.sampling_num);
          // setDesiredFormation2(swarm_des_zy_);
          // init optimizer
          lbfgs::lbfgs_parameter_t lbfgs_params;
          lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
          lbfgs_params.mem_size = 8;
          lbfgs_params.g_epsilon = 1e-5; // 0.0
          lbfgs_params.min_step = 1e-10; // 1e-32
          lbfgs_params.delta = 1.0e-4;
          lbfgs_params.line_search_type = 1; // 1: continues
          lbfgs_params.max_iterations = 20;

          /* get init value */
          double q[3];
          // Average of other swarm pos, if there is any other more efficient trick?
          Eigen::Vector3d average_pos = Eigen::Vector3d::Zero();
          for (int id = 0; id < sparse_id.size(); id++)
          {
            if (sparse_id[id] != drone_id_)
            {
              average_pos += time_cps_.swarm_pos_advance_sets[i][id];
            }
          }
          average_pos /= (sparse_id.size() - 1);
          q[0] = average_pos(0);
          q[1] = average_pos(1);
          q[2] = average_pos(2);

          /* optimize */
          double final_cost;
          time_cps_.idx_of_sets = i;
          opt_local_min_loop_num_ = 0;
          lbfgs::lbfgs_optimize(3,
                                q,
                                &final_cost,
                                PolyTrajOptimizer::swarmGraphCostCallback,
                                NULL,
                                NULL,
                                this,
                                &lbfgs_params);
          // save the local minimum
          Eigen::Vector3d local_min;
          if (is_opt_formation_z_)
            local_min << q[0], q[1], q[2];
          else
            local_min << q[0], q[1], fixed_formation_z_;

          time_cps_.local_min_advance_sets.emplace_back(local_min);
        }
      }
      time_cps_.sampling_num = k;
      time_cps_.duration = T_sum;
    }
    else
    {
      k = time_cps_.sampling_num;
    }

    /* Calculate the gdC, gdT and swarm cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double swarm_formation_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {

      if (idx >= k * 2 / 3)
        break;

      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;

      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // calculate the cost and grad of swarm graph
      // decoupled swarm graph: calculate the local min of swarm graph in advanced
      // coupled swarm graph: calculate the grad and cost of swarm graph in each loop of optimization
      if (time_cps_.enable_decouple_swarm_graph)
      {
        // update swarm graph and calculate cost and grad
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        Eigen::Vector3d dist_vec = pos - time_cps_.local_min_advance_sets[idx];
        double dist = dist_vec.norm();
        double dist2 = dist_vec.squaredNorm();

        if (dist2 > 0)
        {
          // double dist2_2 = dist2 * dist2;
          // double dist2_3 = dist2_2 * dist2;

          // dJ_df = 3 * wei_formation_ * step * omg * dist2_2;
          // df_dp = 2 * dist_vec;
          // gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          // df_dt = df_dp.dot(vel);
          // grad_prev_t = -1 * dJ_df * df_dt;

          dJ_df = 2 * wei_graph_ * step * omg * dist;
          df_dp << dist_vec(0) / dist,
              dist_vec(1) / dist,
              dist_vec(2) / dist;
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          df_dt = df_dp.dot(vel);
          grad_prev_t = -1 * dJ_df * df_dt;

          // cost
          swarm_formation_cost += wei_graph_ * step * omg * dist2;

          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
      }
      else
      {
        // get the swarm pos
        vector<Eigen::Vector3d> swarm_graph_pos(formation_size_);
        for (int id = 0; id < formation_size_; id++)
        {
          if (id == id_in_group)
            swarm_graph_pos[id] = pos;
          else
            swarm_graph_pos[id] = time_cps_.swarm_pos_advance_sets[idx][id];
        }
        // update swarm graph and calculate cost and grad
        double dJ_df, df_dt, grad_prev_t;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        Eigen::Vector3d df_dp;

        time_cps_.swarm_graph_advance_sets[idx].updatePartGraphAndGetGrad(id_in_group, swarm_graph_pos, df_dp);
        double similarity_error;
        time_cps_.swarm_graph_advance_sets[idx].calcFNorm2(similarity_error);

        if (similarity_error > 0.0)
        {
          dJ_df = wei_formation_ * step * omg;
          df_dt = df_dp.dot(vel);
          gradViolaPc = dJ_df * beta0 * df_dp.transpose();
          grad_prev_t = -1 * dJ_df * df_dt;

          // cost
          swarm_formation_cost += wei_formation_ * step * omg * similarity_error;

          // gdC
          jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

          // gdT
          if (piece_of_idx > 0)
            gdT.head(piece_of_idx).array() += grad_prev_t;
        }
      }
    }
    return swarm_formation_cost;
  }

  void PolyTrajOptimizer::feasibilityGradCostVandA(Eigen::VectorXd &gdT, double &vel_cost, double acc_cost)
  {
    static const double step = time_cps_.sampling_time_step;

    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    /* Calculate the gdC, gdT and vel_cost */
    // consider the whole trajectory
    vel_cost = 0.0;
    acc_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4;
    Eigen::Matrix<double, 6, 1> beta1, beta2, beta3;
    Eigen::Vector3d vel, acc, jer;
    double omg;

    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {
      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;

      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;

      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // calculate the cost and grad of velocity feasibility
      double dJ_df = 0.0, df_dt = 0.0, grad_prev_t = 0.0;
      Eigen::Matrix<double, 6, 3> gradViolaPc;

      /* velocity feasibility*/
      double vpen = vel.squaredNorm() - max_vel_ * max_vel_;
      if (vpen > 0)
      {
        dJ_df = 3 * wei_feas_ * step * omg * vpen * vpen;
        gradViolaPc = dJ_df * 2 * beta1 * vel.transpose();
        df_dt = 2 * vel.dot(acc);
        grad_prev_t = -1 * dJ_df * df_dt;

        // cost
        vel_cost += wei_feas_ * step * omg * vpen * vpen * vpen;

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }

      /* acc feasibility*/
      double apen = acc.squaredNorm() - max_acc_ * max_acc_;
      if (apen > 0)
      {
        dJ_df = 3 * wei_feas_ * step * omg * apen * apen;
        gradViolaPc = dJ_df * 2 * beta2 * acc.transpose();
        df_dt = 2 * acc.dot(jer);

        // cost
        acc_cost += wei_feas_ * step * omg * apen * apen * apen;

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }
    }

    // calculate the rest part
    // ...... it is not need

    return;
  }

  bool PolyTrajOptimizer::swarmGraphGradCostP(const int i_dp,
                                              const double t,
                                              const Eigen::Vector3d &p,
                                              const Eigen::Vector3d &v,
                                              Eigen::Vector3d &gradp,
                                              double &gradt,
                                              double &grad_prev_t,
                                              double &costp)
  {
    // wait all the drones have trajectories
    // if (swarm_trajs_->size() < formation_size_ && drone_id_ != formation_size_-1)
    //   return false;
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
      return false;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    // update the swarm graph
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (int id = 0; id < size; id++)
    {
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
        return false;
      // {
      //   double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
      //   swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
      //   swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
      //             exceed_time * swarm_v;
      // }

      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    swarm_graph_->updateGraph(swarm_graph_pos);

    // calculate the swarm graph cost and gradp
    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);

    if (similarity_error > 0)
    {
      ret = true;

      // double similarity_error2 = similarity_error * similarity_error;
      // double similarity_error3 = similarity_error2 * similarity_error;

      costp = wei_formation_ * similarity_error;
      vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);
      // double dJ_df = wei_formation_ * 3 * similarity_error2;
      gradp = wei_formation_ * swarm_grad[drone_id_];

      for (int id = 0; id < size; id++)
      {
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGatherCostGradP(const int i_dp,
                                               const double t,
                                               const Eigen::Vector3d &p,
                                               const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradp,
                                               double &gradt,
                                               double &grad_prev_t,
                                               double &costp)
  {
    // there is a bug still need to fix: when drone_id = formation_size_ - 1,
    // swarm_trajs_->size() = formation_size_ -1 but not swarm_trajs_->size() = formation_size_
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;

    if (size < formation_size_)
      return false;

    // for the formation problem, we may optimize the whole trajectory ?
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    // get the swarm pos and vel
    Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (int id = 0; id < size; id++)
    {
      if (id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    double alpha1 = double(1.0 - formation_size_) / double(formation_size_);
    double alpha2 = double(1.0 / formation_size_);

    for (int id = 0; id < size; id++)
      center_pos += swarm_graph_pos[id];

    center_pos = center_pos * alpha2;

    // calculate the swarm gathering cost and grad
    const double CLEARANCE2 = swarm_gather_threshold_ * swarm_gather_threshold_;
    Eigen::Vector3d dist_vec = center_pos - p;
    double dist2 = dist_vec.norm() * dist_vec.norm();
    double dist_error = dist2 - CLEARANCE2;
    double dist_error2 = dist_error * dist_error;
    double dist_error3 = dist_error2 * dist_error;
    ret = true;

    if (dist_error3 > 0)
    {
      costp = wei_gather_ * dist_error3;
      double dJ_df = wei_gather_ * 3 * dist_error2;
      gradp = dJ_df * 2 * alpha1 * dist_vec;

      for (int id = 0; id < size; id++)
      {
        if (id == drone_id_)
          gradt += dJ_df * 2 * alpha1 * dist_vec.dot(swarm_graph_vel[id]);
        else
          gradt += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);

        if (id != drone_id_)
          grad_prev_t += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);
      }
    }
    return ret;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // use esdf
    double dist;
    Eigen::Vector3d dist_grad;
    grid_map_->evaluateEDTWithGrad(p, dist, dist_grad);
    double dist_err = obs_clearance_ - dist;

    if (dist_err > 0)
    {
      ret = true;
      costp = wei_obs_ * pow(dist_err, 3);
      gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    // if (i_dp <= 0)
    //   return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    int size = swarm_trajs_->size();
    for (int id = 0; id < size; id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
        continue;

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

  bool PolyTrajOptimizer::hybridastarWithMinTraj_QL(const Eigen::MatrixXd &iniState,
                                                    const Eigen::MatrixXd &finState,
                                                    vector<Eigen::Vector3d> &simple_path,
                                                    Eigen::MatrixXd &ctl_points,
                                                    poly_traj::MinJerkOpt &frontendMJ)
  {
    /* step 1: init*/
    bool is_debug = true;
    ros::Time t1 = ros::Time::now();
    // set start and end state
    Eigen::Vector3d start_pt = iniState.col(0);
    Eigen::Vector3d start_vel = iniState.col(1);
    Eigen::Vector3d start_acc = iniState.col(2);
    Eigen::Vector3d end_pt = finState.col(0);
    Eigen::Vector3d end_vel = finState.col(1);
    // Eigen::Vector3d end_vel   = Eigen::Vector3d::Zero();

    // if (is_debug){
    // cout << "[kino] state:---------------------- " << endl;
    // cout << "[kino] start_pt: " << start_pt.transpose();
    // cout <<     ", start_vel: " << start_vel.transpose();
    // cout <<     ", start_acc: " << start_acc.transpose() << endl;
    // cout << "[kino] end_pt: " << end_pt.transpose();
    // // cout <<     ", end_vel: " << end_vel.transpose() << endl;
    // ztr_log3 << "[kino] state:---------------------- " << endl;
    // ztr_log3 << "[kino] start_pt: " << start_pt.transpose();
    // ztr_log3 <<     ", start_vel: " << start_vel.transpose();
    // ztr_log3 <<     ", start_acc: " << start_acc.transpose() << endl;
    // ztr_log3 << "[kino] end_pt: " << end_pt.transpose();
    // ztr_log3 <<     ", end_vel: " << end_vel.transpose() << endl;
    // }

    // ztr debug----------------------------

    // ztr debug----------------------------

    // /* step 2: kino-a* traj search */
    // kino_a_star_->reset();
    // int status = kino_a_star_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
    // // if failed, try again
    // if (status == KinodynamicAstar::NO_PATH){
    //   // if (is_debug)
    //     // ztr_log3 << "[kino] the first try is failed " << endl;
    //     // cout << "[kino] the first try is failed " << endl;

    //   // retry searching with discontinuous initial state
    //   kino_a_star_->reset();
    //   status = kino_a_star_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    //   if (status == KinodynamicAstar::NO_PATH){
    //     // if (is_debug)
    //       // ztr_log3 << "[kino] the replan is also failed " << endl;
    //       cout << "[kino] the replan is also failed " << endl;
    //     kino_a_star_->reset();
    //     return false;
    //   } }
    //   // else{
    //   //   // if (is_debug)
    //   //     // ztr_log3 << "[kino] the replan is success " << endl;
    //   //     // cout << "[kino] the replan is success " << endl;
    //   // }

    // // ztr_log3<<"kino use time-----------"<<(ros::Time::now() - t1).toSec()*1000<<endl;
    // /* step 3: get sample kino-a* waypoints */
    // double  ts = time_cps_.sampling_time_step;    // shoulde be (ctrl_pt_dist / max_vel_) 0.5
    // ts=0.1;//ztr
    // vector<Eigen::Vector3d>  kino_simple_path, start_end_derivatives;
    // kino_a_star_->getSamples(ts, kino_simple_path, start_end_derivatives);
    // int samples_size = kino_simple_path.size();
    // simple_path = kino_simple_path;
    // // if (is_debug)
    // //   ztr_log3 << "[kino] kino_simple_paths size: " << samples_size << endl;
    //   // cout << "[kino] kino_simple_paths size: " << samples_size << endl;

    // /* step 4: generate init minco traj */
    // int piece_num = 4;        // only experience gained
    // Eigen::MatrixXd innerPts(3, piece_num - 1);
    // Eigen::VectorXd time_vec(piece_num);
    // // get innerPts and time_vec from kino_simple_path
    // int per_idx = floor( double(samples_size - 1) / double(piece_num));
    // // if (is_debug){
    // //   ztr_log3<< "[kino] per_idx: " << per_idx << ", ts: " << ts << endl;
    //   // cout << "[kino] per_idx: " << per_idx << ", ts: " << ts << endl;
    // // }
    // //经验数值，取四段，均匀分配路径点
    // for (int i=0; i<piece_num; i++){
    //   if (i != piece_num - 1){
    //     innerPts.col(i) = kino_simple_path[per_idx * (i+1)];
    //     time_vec(i)     = ts * per_idx;
    //   } else{
    //     // the last time duration should not be too large
    //     int rest_num = samples_size - 1 - per_idx * i;
    //     if (rest_num > per_idx){
    //       piece_num += 1;
    //       time_vec(i)     = ts * per_idx;
    //       innerPts.conservativeResize(innerPts.rows(), innerPts.cols()+1);
    //       innerPts.col(innerPts.cols()-1) = kino_simple_path[per_idx * (i+1)];
    //       time_vec.conservativeResize(time_vec.size()+1);
    //       time_vec(time_vec.size()-1) = ts * (rest_num - per_idx);
    //     } else {
    //       time_vec(i)     = ts * rest_num;
    //     }
    //   }
    // }
    // frontendMJ.reset(iniState, finState, piece_num);
    // frontendMJ.generate(innerPts, time_vec);
    // // if (is_debug){
    // //   ztr_log3 << "[kino] duration: " << time_vec.transpose() << endl;
    // //   ztr_log3 << "[kino] max vel: "  << frontendMJ.getTraj().getMaxVelRate() << endl;
    // //   // cout << "[kino] duration: " << time_vec.transpose() << endl;
    // //   // cout << "[kino] max vel: "  << frontendMJ.getTraj().getMaxVelRate() << endl;
    // // }

    // ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);

    // // printf("\033[34m[kino] total time(ms): %.3f\033[0m\n", (ros::Time::now() - t1).toSec()*1000);
    // // ztr_log3<<"[kino] total time(ms):"<< (ros::Time::now() - t1).toSec()*1000<<endl;

    return true;
  }

  bool PolyTrajOptimizer::hybridastarWithMinTraj(const Eigen::MatrixXd &iniState,
                                                 const Eigen::MatrixXd &finState,
                                                 vector<Eigen::Vector3d> &simple_path,
                                                 Eigen::MatrixXd &ctl_points,
                                                 poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pt = iniState.col(0);
    Eigen::Vector3d start_vel = iniState.col(1);
    Eigen::Vector3d start_acc = iniState.col(2);
    Eigen::Vector3d end_pt = finState.col(0);
    Eigen::Vector3d end_vel = finState.col(1);
    // 感觉结尾的vel有问题????????,开始速度与acc感觉有问题!!!!!!!!!!!!!!!!!!!!!!!!!!
    //  ztr_log3 << "!!!!!!!!!!!!!!kino state!!!!!!!!!!!!!!" << endl<<
    //              "start_vel"<<start_vel.transpose()<<
    //              "start_acc"<<start_acc.transpose()<<
    //              "end_vel"<<start_acc.transpose()<<endl;
    end_vel.setZero();

    // kino_a_star_->reset();
    // // ztr_log3 << "before search---------------------------------" << endl;

    // //考虑失败了就用A_star
    // std::vector<Eigen::Vector3d> a_star_path;
    // std::vector<Eigen::Vector3d> a_star_simple_path;
    // auto t2 = ros::Time::now();
    // bool success = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pt, end_pt, a_star_simple_path, a_star_path);
    // simple_path = a_star_simple_path;
    // auto t_search = (ros::Time::now() - t2).toSec() * 1000;
    // // ztr_log3 << "A*: " << success << endl;
    // // ztr_log3 << "[A* plan]: t_search time(ms): " << t_search << endl;
    // // cout << "[A* plan]: t_search time(ms): " << t_search << endl;

    // auto t1 = ros::Time::now();
    // int status = kino_a_star_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

    // // init标志位控制是否是最开始查找，是由search函数里面的init_search接受，其中判断是否是第一次搜索
    // //最开始就start_acc作为起始点的加速度输入进行查找，如果找不到，则再进行一轮离散化输入加速度查找
    // //若都找不到，则路径寻找失败
    // if (status == KinodynamicAstar::NO_PATH)
    // {
    //   // ztr_log3 << "[kino replan]: kinodynamic search fail!" << endl;
    //   // cout << "[kino replan]: kinodynamic search fail!" << endl;

    //   // retry searching with discontinuous initial state
    //   kino_a_star_->reset();
    //   status = kino_a_star_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    //   if (status == KinodynamicAstar::NO_PATH)
    //   {
    //     // ztr_log3 << "!!!!!!!!!!!!!![kino replan]: Can't find path.!!!!!!!!!!!!!!!!" << endl;
    //     // cout << "[kino replan]: Can't find path." << endl;
    //     // return false;
    //   }
    //   else
    //   {
    //     // cout << "[kino replan]: retry search success." << endl;
    //     // ztr_log3 << "[kino replan]: retry search success." << endl;
    //   }
    // }
    // else
    // {
    //   // ztr_log3 << "[kino replan]: kinodynamic search success." << endl;
    //   // cout << "[kino replan]: kinodynamic search success." << endl;
    // }

    // // get path
    // std::vector<Eigen::Vector3d> kino_simple_path;
    // std::vector<Eigen::Vector3d> kino_simple_final_path;
    // if (status != KinodynamicAstar::NO_PATH)
    // {
    //   double t_search = (ros::Time::now() - t2).toSec()*1000;
    //   // ztr_log3 << "[kino plan]: t_search time(ms): " << t_search << endl;
    //   // cout << "[kino plan]: t_search time(ms): " << t_search << endl;

    //   simple_path = kino_a_star_->getKinoTraj(0.01);

    //   kino_simple_path = kino_a_star_->getKinoTraj(1);

    //   // ztr_log3 << "------------------------kino simple path----------------------------------------" << endl;

    //   for (size_t i = 0; i < kino_simple_path.size() - 1; i++)
    //   {
    //     if ((kino_simple_path[i] - kino_simple_path[i + 1]).norm() < 0.5)
    //       continue;

    //     kino_simple_final_path.push_back(kino_simple_path[i]);
    //   }

    //   // for (size_t i = 0; i < kino_simple_final_path.size(); i++)
    //   // {
    //   //   ztr_log3 << "第" << i << "个路径点是" << kino_simple_final_path[i].transpose() << " " << endl;
    //   // }
    // }

    // // ztr_log3 << "------------------------astar----------------------------------------" << endl;
    // // for (size_t i = 0; i < a_star_path.size(); i++)
    // // {
    // //   ztr_log3 << "第" << i << "个路径点是" << a_star_path[i].transpose() << " " << endl;
    // // }
    // // ztr_log3 << "------------------------astar simple path----------------------------------------" << endl;

    // // for (size_t i = 0; i < a_star_simple_path.size(); i++)
    // // {
    // //   ztr_log3 << "第" << i << "个路径点是" << a_star_simple_path[i].transpose() << " " << endl;
    // // }

    // std::vector<std::vector<Eigen::Vector3d>> path;
    // path.push_back(a_star_path);

    // // ztr_log3 << "--------------------start minco-----------------------------------" << endl;
    // if (status != KinodynamicAstar::NO_PATH)
    // {
    //   simple_path = kino_simple_final_path;
    // }

    // int piece_num = simple_path.size() - 1;
    // Eigen::MatrixXd innerPts;

    // if (piece_num > 1)
    // {
    //   innerPts.resize(3, piece_num - 1);
    //   for (int i = 0; i < piece_num - 1; i++)
    //     innerPts.col(i) = simple_path[i + 1];
    // }
    // else
    // {
    //   // piece_num == 1
    //   piece_num = 2;
    //   innerPts.resize(3, 1);
    //   innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    // }

    // frontendMJ.reset(iniState, finState, piece_num);

    // // debug
    // // cout <<"----------------" << endl;
    // // cout << "iniState : " << iniState.col(0).transpose() << endl;
    // // cout << "finState : " << finState.col(0).transpose() << endl;
    // // cout << "piece_num : " << piece_num << endl;
    // // cout << "simple_path : " << endl;
    // // for (int k=0; k<simple_path.size();k++){
    // //   cout <<  simple_path[k].transpose() << endl;
    // // }
    // // cout << "innerPts : " << innerPts << endl;

    // /* generate init traj*/
    // // the way of generating init traj of a* and hybrid-a* is different
    // double des_vel = max_vel_;
    // Eigen::VectorXd time_vec(piece_num);
    // int debug_num = 0;
    // do
    // {
    //   if (piece_num == 2)
    //   {
    //     time_vec(0) = (innerPts.col(0) - start_pt).norm() / des_vel;
    //     time_vec(1) = (innerPts.col(0) - end_pt).norm() / des_vel;
    //   }
    //   else
    //   {
    //     for (size_t i = 1; i <= piece_num; ++i)
    //     {
    //       time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pt).norm() / des_vel
    //                                  : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
    //     }
    //   }

    //   frontendMJ.generate(innerPts, time_vec);
    //   debug_num++;
    //   des_vel /= 1.2;
    // } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ * 1.2 && debug_num <= 5);

    // // debug
    // // cout << "time_vec : " << time_vec.transpose() << endl;
    // // cout << "max vel : " << frontendMJ.getTraj().getMaxVelRate() << endl;
    // // ztr_log3 << "--------------------before get traj-----------------------------------" << endl;
    // poly_traj::Trajectory traj;
    // traj = frontendMJ.getTraj();
    // // ztr_log3 << "--------------------after get traj-----------------------------------" << endl;
    // ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);

    return true;
  }
  bool PolyTrajOptimizer::topoWithMinTraj(const Eigen::MatrixXd &iniState,
                                          const Eigen::MatrixXd &finState,
                                          vector<Eigen::Vector3d> &topo_path,
                                          Eigen::MatrixXd &ctl_points,
                                          poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos = finState.col(0);
    vector<Eigen::Vector3d> simple_path = topo_path;
    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }

    frontendMJ.reset(iniState, finState, piece_num);
    /* generate init traj*/
    double des_vel = 1.2 * max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      if (piece_num == 2)
      {
        time_vec(0) = (innerPts.col(0) - start_pos).norm() / des_vel;
        time_vec(1) = (innerPts.col(0) - end_pos).norm() / des_vel;
      }
      else
      {
        for (int i = 1; i <= piece_num; ++i)
        {
          time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                     : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
        }
      }

      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.2;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ * 1.2 && debug_num <= 5);

    poly_traj::Trajectory traj;
    traj = frontendMJ.getTraj();

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);

    return true;
  }

  bool PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           vector<Eigen::Vector3d> &simple_path,
                                           Eigen::MatrixXd &ctl_points,
                                           poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0);
    init_start_zy = start_pos;
    Eigen::Vector3d end_pos = finState.col(0);
    std::vector<Eigen::Vector3d> a_star_path;
    /* astar search and get the simple path*/
    // simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos);
    bool success = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos, simple_path, a_star_path, 0.2, 1.0);
    if (!success)
    {
      // ROS_WARN("unable do astarSearchAndGetSimplePath");
      return false;
    }

    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    // debug---------------------------------------------------------------------
    // for (size_t i = 0; i < simple_path.size(); i++)
    // {
    //   ztr_log3 << "第" << i << "个路径点是" << simple_path[i].transpose() << " " << endl;
    // }
    // debug---------------------------------------------------------------------
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }

    frontendMJ.reset(iniState, finState, piece_num);

    // debug
    // if(drone_id_==9)
    // {
    //   cout <<"----------------" << endl;
    // cout << "iniState : " << iniState.col(0).transpose() << endl;
    // cout << "finState : " << finState.col(0).transpose() << endl;
    // cout << "piece_num : " << piece_num << endl;
    // cout << "simple_path : " << endl;
    // for (int k=0; k<simple_path.size();k++){
    //   cout <<  simple_path[k].transpose() << endl;
    // }
    // cout << "innerPts : " << innerPts << endl;
    // }

    /* generate init traj*/
    double des_vel = 1.2 * max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      if (piece_num == 2)
      {
        time_vec(0) = (innerPts.col(0) - start_pos).norm() / des_vel;
        time_vec(1) = (innerPts.col(0) - end_pos).norm() / des_vel;
      }
      else
      {
        for (int i = 1; i <= piece_num; ++i)
        {
          time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                     : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
        }
      }

      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.2;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ * 1.2 && debug_num <= 5);

    // debug
    // cout << "time_vec : " << time_vec.transpose() << endl;
    // cout << "max vel : " << frontendMJ.getTraj().getMaxVelRate() << endl;
    // if(drone_id_ == 9){

    //   std::cout << "init traj pn: " << frontendMJ.getTraj().getPieceNum() << "\n";
    //   std::cout << "cps per piece: "<< cps_num_prePiece_ << "\n";
    //   std::cout << "time_vec: "<< time_vec.transpose() << "\n";
    // }
    poly_traj::Trajectory traj;
    traj = frontendMJ.getTraj();

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);

    return true;
  }

  double PolyTrajOptimizer::gettopo_formation_err(const poly_traj::Trajectory &traj_topo)
  {
    if (!is_set_swarm_advanced_)
      return 0.0;

    int size = swarm_trajs_->size();
    if (size < formation_size_all)
      return 0.0;

    static const double step = time_cps_.sampling_time_step;
    std::vector<Eigen::Vector3d> swarm_pos_all;
    swarm_pos_all.resize(formation_size_all);
    double T_sum = traj_topo.getTotalDuration();
    double formation_error_topo = 0.0;
    for (double i = t_now_; i < T_sum; i = i + step)
    {
      for (int iddd = 0; iddd < formation_size_all; iddd++)
      {
        Eigen::Vector3d swarm_p, swarm_v;
        if (iddd == drone_id_)
        {
          traj_topo.getPos(i - t_now_);
        }
        else
        {
          if (i < swarm_trajs_->at(iddd).duration)
          {
            swarm_p = swarm_trajs_->at(iddd).traj.getPos(i - swarm_trajs_->at(iddd).start_time);
            swarm_v = swarm_trajs_->at(iddd).traj.getVel(i - swarm_trajs_->at(iddd).start_time);
          }
          else
          {
            double exceed_time = i - swarm_trajs_->at(iddd).duration;
            swarm_v = swarm_trajs_->at(iddd).traj.getVel(swarm_trajs_->at(iddd).duration);
            swarm_p = swarm_trajs_->at(iddd).traj.getPos(swarm_trajs_->at(iddd).duration) + exceed_time * swarm_v;
          }
        }
        swarm_pos_all[iddd] = swarm_p;
      }
      formation_error_topo = formation_error_topo + getFormationError(swarm_pos_all);
    }
    return formation_error_topo;
  }
  bool PolyTrajOptimizer::getFormationPos(vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos)
  {
    swarm_graph_pos.resize(formation_size_all);
    int size = swarm_trajs_->size();

    if (size < formation_size_all || !use_formation_)
    {
      return false;
    }
    else
    {
      double pt_time = t_now_;

      swarm_graph_pos[0] = init_start_zy;

      for (int id = 0; id < size; id++)
      {
        if (swarm_trajs_->at(id).drone_id < 0 || swarm_trajs_->at(id).drone_id == 0)
          continue;

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

        Eigen::Vector3d swarm_p, swarm_v;
        if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
          swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                    exceed_time * swarm_v;
        }
        swarm_graph_pos[id] = swarm_p;
      }
    }

    return true;
  }

  // 尚未看
  double PolyTrajOptimizer::getFormationError(vector<Eigen::Vector3d> swarm_graph_pos)
  {
    swarm_graph_->updateGraph(swarm_graph_pos);
    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);
    return similarity_error;
  }

  bool PolyTrajOptimizer::leaderPosFormationCostGradP(const int i_dp,
                                                      const double t,
                                                      const Eigen::Vector3d &p,
                                                      const Eigen::Vector3d &v,
                                                      Eigen::Vector3d &gradp,
                                                      double &gradt,
                                                      double &grad_prev_t,
                                                      double &costp)
  {
    if (drone_id_ == 0)
      return false;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init parameters
    gradp.setZero();
    costp = 0;
    gradt = 0;
    grad_prev_t = 0;

    /* formation cost */
    // get pilot_pos and pilot_vel
    Eigen::Vector3d pilot_pos, pilot_vel;
    double pt_time = t_now_ + t;
    double traj_i_satrt_time = swarm_trajs_->at(0).start_time;
    if (pt_time < traj_i_satrt_time + swarm_trajs_->at(0).duration)
    {
      pilot_pos = swarm_trajs_->at(0).traj.getPos(pt_time - traj_i_satrt_time);
      pilot_vel = swarm_trajs_->at(0).traj.getVel(pt_time - traj_i_satrt_time);
    }
    else
    {
      double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(0).duration);
      pilot_vel = swarm_trajs_->at(0).traj.getVel(swarm_trajs_->at(0).duration);
      pilot_pos = swarm_trajs_->at(0).traj.getPos(swarm_trajs_->at(0).duration) + exceed_time * pilot_vel;
    }

    // get formation goal
    Eigen::Vector3d formation_goal = pilot_pos + formation_relative_dist_[drone_id_];
    double formation_clearance = 0.1;

    // calculate cost
    Eigen::Vector3d dist_vec = p - formation_goal;
    double dist2 = dist_vec.squaredNorm();

    double dist2_err_max = dist2 - formation_clearance * formation_clearance;
    double dist2_err2_max = dist2_err_max * dist2_err_max;
    double dist2_err3_max = dist2_err2_max * dist2_err_max;

    if (dist2_err_max > 0)
    {
      costp = wei_formation_ * dist2_err3_max;
      Eigen::Vector3d dJ_dP = wei_formation_ * 3 * dist2_err2_max * 2 * dist_vec;
      gradp = dJ_dP;
      gradt = dJ_dP.dot(v - pilot_vel);
      grad_prev_t = dJ_dP.dot(-pilot_vel);
    }
    return true;
  }

  bool PolyTrajOptimizer::relativePosFormationCostGradP(const int i_dp,
                                                        const double t,
                                                        const Eigen::Vector3d &p,
                                                        const Eigen::Vector3d &v,
                                                        Eigen::Vector3d &gradp,
                                                        double &gradt,
                                                        double &grad_prev_t,
                                                        double &costp)
  {
    if (drone_id_ == 0)
      return false;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    int formation_size = 7;
    int size = swarm_trajs_->size();
    if (size < formation_size)
      return false;

    // init parameters
    gradp.setZero();
    costp = 0;
    gradt = 0;
    grad_prev_t = 0;

    // get the swarm pos
    vector<Eigen::Vector3d> swarm_pos(formation_size), swarm_vel(formation_size);
    swarm_pos[drone_id_] = p;
    swarm_vel[drone_id_] = v;

    for (int id = 0; id < formation_size; id++)
    {
      if (id == drone_id_)
        continue;

      double pt_time = t_now_ + t;
      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_pos[id] = swarm_p;
      swarm_vel[id] = swarm_v;
    }

    // get relative goal and connect_num
    int connect_num;
    vector<Eigen::Vector3d> connect_pos, connect_vel;
    if (drone_id_ == 0)
    {
      connect_num = 6;
      for (int i = 1; i <= connect_num; i++)
      {
        connect_pos.push_back(swarm_pos[i] - formation_relative_dist_[i]);
        connect_vel.push_back(swarm_vel[i]);
      }
    }
    else
    {
      connect_num = 3;
      if (drone_id_ == 1)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[6] + formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[6]);
        connect_pos.push_back(swarm_pos[2] + formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[2]);
      }
      else if (drone_id_ == 2)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[1] + formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[1]);
        connect_pos.push_back(swarm_pos[3] + formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[3]);
      }
      else if (drone_id_ == 3)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[2] + formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[2]);
        connect_pos.push_back(swarm_pos[4] + formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[4]);
      }
      else if (drone_id_ == 4)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[3] + formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[3]);
        connect_pos.push_back(swarm_pos[5] + formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[5]);
      }
      else if (drone_id_ == 5)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[4] + formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[4]);
        connect_pos.push_back(swarm_pos[6] + formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[6]);
      }
      else if (drone_id_ == 6)
      {
        connect_pos.push_back(swarm_pos[0] + formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[5] + formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[5]);
        connect_pos.push_back(swarm_pos[1] + formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[1]);
      }
    }

    // calculate cost
    double formation_clearance = 0.1;
    for (int i = 0; i < connect_num; i++)
    {
      Eigen::Vector3d dist_vec = p - connect_pos[i];
      double dist2 = dist_vec.squaredNorm();

      double dist2_err_max = dist2 - formation_clearance * formation_clearance;
      double dist2_err2_max = dist2_err_max * dist2_err_max;
      double dist2_err3_max = dist2_err2_max * dist2_err_max;

      if (dist2_err_max > 0)
      {
        costp += wei_formation_ * dist2_err3_max;
        Eigen::Vector3d dJ_dP = wei_formation_ * 3 * dist2_err2_max * 2 * dist_vec;
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - connect_vel[i]);
        grad_prev_t += dJ_dP.dot(-connect_vel[i]);
      }
    }
    return true;
  }

  void PolyTrajOptimizer::updateSwarmGraph(Eigen::VectorXi assignment)
  {
    swarm_graph_->setAssignment(assignment);
    assignment_ = assignment;
  }
  double PolyTrajOptimizer::calculateMean(const std::vector<double> &data)
  {
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
  }

  double PolyTrajOptimizer::calculateVariance(const std::vector<double> &data)
  {
    if (data.empty())
      return 0.0;

    double mean = calculateMean(data);
    double variance = 0.0;

    for (double value : data)
    {
      variance += std::pow(value - mean, 2);
    }

    variance /= data.size(); // 总体方差
    // variance /= data.size() - 1; // 样本方差（如果是样本数据）

    return variance;
  }
  double PolyTrajOptimizer::gettopo_visother_err(const poly_traj::Trajectory &traj_topo_my)
  {
    int size = swarm_trajs_->size();
    if (size < formation_size_all)
      return 0.0;

    std::cout << "gettopo_visother_err" << std::endl;

    std::vector<double> vis_rate_each(formation_size_all - 1);
    double safe_distance = 2 * grid_map_->getResolution();
    poly_traj::Trajectory traj_vis = traj_topo_my;
    double time_of_traj = traj_vis.getTotalDuration();
    int sample_num_traj = 50;
    double flag_vis_num = 0.0;
    bool flag_v = false;
    int other_int = 0;
    double t_now_topo = ros::Time::now().toSec();
    for (int k = 0; k < size - 1; k++)
    {
      if (k != drone_id_)
      {
        other_int++;
        for (int i = 0; i < vis_step; ++i)
        {
          Eigen::Vector3d traj_vis_pos = traj_vis.getPos(i * (time_of_traj / vis_step));
          for (int j = 0; j < sample_num_traj; j++)
          {
            Eigen::Vector3d traj_other_pos = swarm_trajs_->at(k).traj.getPos(i * (time_of_traj / vis_step) + t_now_topo - swarm_trajs_->at(k).start_time);
            double j_ = static_cast<double>(j);
            double sample_num_traj_ = static_cast<double>(sample_num_traj);
            Eigen::Vector3d init_traj_pos = traj_vis_pos + j_ * (traj_other_pos - traj_vis_pos) / sample_num_traj_;
            if ((grid_map_->getDistance(init_traj_pos) > safe_distance) && (traj_other_pos - traj_vis_pos).norm() < vis_hor)
            {
              flag_v = true;
            }
          }
          if (flag_v)
          {
            flag_v = false;
            flag_vis_num = flag_vis_num + 1.0;
          }
        }
        vis_rate_each[other_int] = swarm_trajs_->at(k).vis_aware + (flag_vis_num / sample_num_traj) * 100.0;
      }
    }
    return calculateVariance(vis_rate_each);
  }

  double PolyTrajOptimizer::gettopo_vismy_err(const poly_traj::Trajectory &traj_topo_my)
  {
    int size = swarm_trajs_->size();
    if (size < formation_size_all)
      return 0.0;

    double safe_distance = 2 * grid_map_->getResolution();
    poly_traj::Trajectory traj_vis = traj_topo_my;
    double time_of_traj = traj_vis.getTotalDuration();
    int sample_num_traj = 50;
    double flag_vis_num = 0.0;
    bool flag_v = false;
    for (int i = 0; i < vis_step; ++i)
    {
      Eigen::Vector3d traj_vis_pos = traj_vis.getPos(i * (time_of_traj / vis_step));
      for (int j = 0; j < sample_num_traj; j++)
      {
        for (int k = 0; k < size - 1; k++)
        {
          if (k != drone_id_)
          {
            Eigen::Vector3d traj_other_pos = swarm_trajs_->at(k).traj.getPos(i * (time_of_traj / vis_step) + t_now_ - swarm_trajs_->at(k).start_time);
            double j_ = static_cast<double>(j);
            double sample_num_traj_ = static_cast<double>(sample_num_traj);
            Eigen::Vector3d init_traj_pos = traj_vis_pos + j_ * (traj_other_pos - traj_vis_pos) / sample_num_traj_;
            if ((grid_map_->getDistance(init_traj_pos) > safe_distance) && (traj_other_pos - traj_vis_pos).norm() < vis_hor)
            {
              flag_v = true;
            }
          }
        }
        if (flag_v)
        {
          flag_v = false;
          flag_vis_num = flag_vis_num + 1.0;
        }
      }
    }
    return (flag_vis_num / sample_num_traj) * 100.0;
  }

  double PolyTrajOptimizer::sumvisAware()
  {
    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_all - 1)
      size = formation_size_all;
    if (size < formation_size_all)
      return 0.0;

    double safe_distance = 2 * grid_map_->getResolution();
    poly_traj::Trajectory traj_vis = jerkOpt_.getTraj();
    double time_of_traj = traj_vis.getTotalDuration();
    int sample_num_traj = 50;
    double flag_vis_num = 0.0;
    bool flag_v = false;
    for (int i = 0; i < vis_step; ++i)
    {
      Eigen::Vector3d traj_vis_pos = traj_vis.getPos(i * (time_of_traj / vis_step));
      for (int j = 0; j < sample_num_traj; j++)
      {
        for (int k = 0; k < size; k++)
        {
          if (k != drone_id_)
          {
            Eigen::Vector3d traj_other_pos = swarm_trajs_->at(k).traj.getPos(i * (time_of_traj / vis_step) + t_now_ - swarm_trajs_->at(k).start_time);
            double j_ = static_cast<double>(j);
            double sample_num_traj_ = static_cast<double>(sample_num_traj);
            Eigen::Vector3d init_traj_pos = traj_vis_pos + j_ * (traj_other_pos - traj_vis_pos) / sample_num_traj_;
            if ((grid_map_->getDistance(init_traj_pos) > safe_distance) && (traj_other_pos - traj_vis_pos).norm() < vis_hor)
            {
              flag_v = true;
            }
          }
        }
        if (flag_v)
        {
          flag_v = false;
          flag_vis_num = flag_vis_num + 1.0;
        }
      }
    }
    return (flag_vis_num / sample_num_traj) * 100.0;
  }
  bool PolyTrajOptimizer::sumConstrainAware(Eigen::Vector3d &form_grad_sum, Eigen::Vector3d &obs_grad_sum,
                                            double &obs_dist_sum, double &aware_sum, double &t_duration)
  {
    // Make sure every swarm_traj is received
    int swarm_traj_size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      swarm_traj_size = formation_size_;

    if (swarm_traj_size < formation_size_)
      return false;

    // Params for iteration
    int N = piece_num_, K = cps_num_prePiece_;

    Eigen::Vector3d pos;
    Eigen::Matrix<double, 6, 1> beta;
    double s1, s2, s3, s4, s5; // Time basis

    double step;
    double omg; // Trapezoidal rule weights
    double t_cur = 0;
    double t_sum = t_now_;
    int cp_num = 0; // Counts constrain points

    // Params for aware calculation
    double obs_dist;
    Eigen::Vector3d obs_grad, form_grad;
    double cosbeta = 0, aware = 0;
    double alpha = 5, gamma = -1, lambda = 25;

    Eigen::Vector3d swarm_p, swarm_v;
    std::vector<Eigen::Vector3d> swarm_graph_pos(formation_size_);

    form_grad_sum << 0, 0, 0;
    obs_grad_sum << 0, 0, 0;
    obs_dist_sum = 0;
    aware_sum = 0;

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta << 1.0, s1, s2, s3, s4, s5;

        pos = c.transpose() * beta;
        omg = (j == 0 || j == K) ? 0.5 : 1.0; // quadrature coef
        t_cur = t_sum + step * j;

        if (cp_num == 0)
        { // Skip the first evaluation
          s1 += step;
          if (j != K || (j == K && i == N - 1))
          {
            cp_num++;
          }
          continue;
        }

        if (cp_num < cps_.cp_size / 3 * 2)
        {

          // Retrieve the obstacle grad and dist
          grid_map_->evaluateFirstGrad(pos, obs_grad);
          grid_map_->evaluateEDT(pos, obs_dist);
          obs_grad *= -1;

          if (obs_grad.norm() > 5 || obs_grad.norm() < 0.01 || obs_dist > 20 || obs_dist < 0)
          { // Skip the ESDF outlier
            s1 += step;
            if (j != K || (j == K && i == N - 1))
            {
              cp_num++;
            }
            continue;
          }

          // Sum obs_related
          obs_grad_sum += omg * obs_grad;
          obs_dist_sum += omg * obs_dist;

          // Retrieve the formation grad
          swarm_graph_pos[drone_id_] = pos;
          for (int id = 0; id < formation_size_; id++)
          {
            if (id == drone_id_)
              continue;

            double traj_i_start_time = swarm_trajs_->at(id).start_time;

            if (t_cur < traj_i_start_time + swarm_trajs_->at(id).duration)
            {
              swarm_p = swarm_trajs_->at(id).traj.getPos(t_cur - traj_i_start_time);
            }
            else
            {
              double exceed_time = t_cur - (traj_i_start_time + swarm_trajs_->at(id).duration);
              swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
              swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                        exceed_time * swarm_v;
            }
            swarm_graph_pos[id] = swarm_p;
          }

          swarm_graph_->updateGraph(swarm_graph_pos);
          form_grad = swarm_graph_->getGradOrg(drone_id_);

          // Sum form_grad
          form_grad_sum += omg * form_grad;

          // Constrain aware calculation
          cosbeta = (form_grad.dot(obs_grad)) / (form_grad.norm() * obs_grad.norm());
          aware = lambda * form_grad.norm() / (1 + exp(alpha * cosbeta + gamma)) / obs_dist;
          aware_sum += omg * step * aware;

          s1 += step;
          if (j != K || (j == K && i == N - 1))
          {
            cp_num++;
          }
        }
        else
        {
          t_duration = t_cur - t_now_;
          aware_sum = aware_sum / t_duration;
          return true;
        }
      }
      t_sum += jerkOpt_.get_T1()(i);
    }
  }

  void PolyTrajOptimizer::setEnvironment(ros::NodeHandle &nh, const GridMap::Ptr &map, int id)
  {
    grid_map_ = map;
    double resolution = grid_map_->getResolution();
    Eigen::Vector3d map_size = grid_map_->getMapSize();
    Eigen::Vector3i pool_size;
    pool_size << int(map_size(0) / resolution), int(map_size(1) / resolution), int(map_size(2) / resolution);

    if (use_kino_)
    {
      // kino_a_star_.reset(new KinodynamicAstar);
      // kino_a_star_->setParam(nh);
      // kino_a_star_->setEnvironment(grid_map_);
      // kino_a_star_->init(id);
    }
    else
    {
      a_star_.reset(new AStar);
      a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 60));
    }

    ztr_log3.open("/home/ztr/Pipline-Swarm-Formation/log/drone_" + to_string(id) + ".txt");
  }
  // 设置控制点cps_为points
  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::formation_change(bool get_change)
  {
    formation_change_get = get_change;
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id)
  {
    drone_id_ = drone_id;
  }
  void PolyTrajOptimizer::FormationChange(const traj_utils::FormationType::ConstPtr &msg)
  {
    formation_change_get = true;
    formation_change_flag = true;
    F_type = *msg;
    swarm_graph_.reset(new SwarmGraph);
    swarm_des_old = swarm_des_;
    // setDesiredFormation(F_type.formation_type);
  }

  Eigen::MatrixXd PolyTrajOptimizer::compute_normalized_laplacian(const std::vector<Eigen::Vector3d> &points)
  {
    int num_points = points.size();
    // int num_points = 3;
    // 计算距离矩阵
    // Eigen::MatrixXd distance_matrix(num_points, num_points);
    Eigen::MatrixXd distance_matrix; // 声明一个空矩阵
    // distance_matrix.resize(num_points, num_points); // 调整大小
    distance_matrix.resize(num_points, num_points); // 调整大小
    for (int i = 0; i < num_points; ++i)
    {
      for (int j = 0; j < num_points; ++j)
      {
        distance_matrix(i, j) = (points[i] - points[j]).norm();
      }
    }

    // 计算邻接矩阵
    Eigen::MatrixXd adjacency_matrix = (-distance_matrix).array().exp();
    adjacency_matrix.diagonal().setZero(); // 对角线设为0

    // 计算度矩阵
    Eigen::VectorXd degree_vector = adjacency_matrix.rowwise().sum();
    Eigen::MatrixXd degree_matrix = degree_vector.asDiagonal();

    // 计算拉普拉斯矩阵 L = D - A
    Eigen::MatrixXd laplacian_matrix = degree_matrix - adjacency_matrix;

    // 计算归一化拉普拉斯矩阵 L_normalized = D^(-1/2) * L * D^(-1/2)
    // 计算 D^(-1/2)
    Eigen::MatrixXd d_inv_sqrt = degree_matrix.llt().solve(Eigen::MatrixXd::Identity(num_points, num_points));
    d_inv_sqrt = d_inv_sqrt.cwiseSqrt();

    // 最终计算归一化拉普拉斯矩阵
    Eigen::MatrixXd normalized_laplacian = d_inv_sqrt * laplacian_matrix * d_inv_sqrt;

    return normalized_laplacian;
  }

  Eigen::MatrixXd PolyTrajOptimizer::compute_laplacian_difference(
      const std::vector<Eigen::Vector3d> &des_pos,
      const std::vector<Eigen::Vector3d> &cur_pos)
  {
    // 计算期望位置的归一化拉普拉斯矩阵
    Eigen::MatrixXd des_laplacian = compute_normalized_laplacian(des_pos);

    // 计算当前位置的归一化拉普拉斯矩阵
    Eigen::MatrixXd cur_laplacian = compute_normalized_laplacian(cur_pos);

    // 返回两个矩阵的差
    return des_laplacian - cur_laplacian;
  }

  Eigen::MatrixXd PolyTrajOptimizer::process_difference_matrix(const Eigen::MatrixXd &diff_matrix)
  {
    // 创建一个与差异矩阵相同大小的结果矩阵
    Eigen::MatrixXd processed_matrix(diff_matrix.rows(), diff_matrix.cols());

    // 遍历差异矩阵并处理
    for (int i = 0; i < diff_matrix.rows(); ++i)
    {
      for (int j = 0; j < diff_matrix.cols(); ++j)
      {
        if (std::abs(diff_matrix(i, j)) > max_graph_aware || i == j)
        {
          processed_matrix(i, j) = 0;
        }
        else
        {
          processed_matrix(i, j) = 1;
        }
      }
    }

    return processed_matrix;
  }
  void PolyTrajOptimizer::set_leader_path(const Eigen::Matrix3d &start_leader,
                                          const Eigen::Matrix3d &end_leader,
                                          const vector<double> &rrr_start,
                                          const vector<double> &rrr_end,
                                          const vector<double> &lll_start,
                                          const vector<double> &lll_end,
                                          std::vector<Eigen::Vector4d> &path_R_,
                                          const double &opt_time_now,
                                          double &opt_score)
  {
    leader_opt_time = opt_time_now;
    Eigen::Matrix3d start_state, end_state;
    start_state.setZero();
    end_state.setZero();
    start_state = start_leader;
    end_state = end_leader;
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // start_state.col(0) << -2.0, 0.0, 1;
    // end_state.col(0) << 30.0, 0.0, 1;
    Eigen::Matrix<double, 1, 3> headRadius, tailRadius;
    Eigen::Matrix<double, 1, 3> headlll, taillll;
    headRadius.setZero();
    tailRadius.setZero();
    headlll.setZero();
    taillll.setZero();

    headRadius(0, 0) = rrr_start[0];
    tailRadius(0, 0) = rrr_end[0];
    headlll(0, 0) = lll_start[0];
    taillll(0, 0) = lll_end[0];

    std::cout << "Start Kinodynamic Astar\n";
    double t1 = ros::Time::now().toSec();

    // a_star_.search(start_state.col(0), end_state.col(0), 0.4);
    std::vector<Eigen::Vector3d> final_simple_path;
    std::vector<Eigen::Vector3d> simple_path;
    std::vector<Eigen::Vector3d> a_star_path;
    double primitive_cost = 9999.0;
    int rank_rrr = 0, rank_lll = 0, rank_hhh = 0;
    // rrr: rrr_start, 0.9*rrr_start, lll_start, 0.8*lll_start, 0.6*lll_start, 0.4*lll_start, 1.2*lll_start, 1.4*lll_start, 1,6*lll_start
    std::vector<double> primitive_rrr = {rrr_start[0]};
    // std::vector<double> primitive_hhh = {0.0};
    std::vector<double> primitive_hhh = {0.0, 0.10, -0.10, 0.20, -0.20, 0.30, -0.30, 0.40, -0.40};
    // std::vector<double> primitive_lll = {1.0,0.9,1.11,0.8,1.25,0.7,1.43,0.6,1.67,0.5,2.0,0.4,2.5};
    std::vector<double> primitive_lll = {1.0,1.11,1.25,1.43,1.67,2.0,2.5};
    cout << "rrr_start[0]:::" << rrr_start[0] << endl;
    cout << "lll_start[0]:::" << lll_start[0] << endl;

    /*************************************************************************************************************** */
    for (int i = 0; i < primitive_hhh.size(); i++)
    {
      for (int j = 0; j < primitive_lll.size(); j++)
      {
        double s_occup = 0.0;
        for (int k = 0; k < 8; k++)
        {
          double end_lengh = (end_state.col(0) - start_state.col(0)).norm();
          Eigen::Vector3d end_point_hhh = Eigen::Vector3d(cos(primitive_hhh[i]) * end_lengh, sin(primitive_hhh[i]) * end_lengh, 1.0);
          Eigen::Vector3d waypoint_front = start_state.col(0) + double(k) * (end_point_hhh) / 8.0;
          s_occup += a_star_->checkOccupancy_num(waypoint_front, primitive_rrr[0], primitive_lll[j]);
        }
        double cost_prim = s_occup + 90 * abs(primitive_lll[0] - primitive_lll[j]) + 100 * abs(primitive_hhh[0] - primitive_hhh[i]);
        if (cost_prim < primitive_cost)
        {
          opt_score = s_occup;
          primitive_cost = cost_prim;
          rank_hhh = i;
          rank_lll = j;
        }
      }
    }

    if (abs(primitive_hhh[rank_hhh_old] - primitive_hhh[rank_hhh]) >= 0.7 || abs(primitive_lll[rank_lll_old] - primitive_lll[rank_lll]) >= 0.7)
    {
      rank_hhh = rank_hhh_old;
      rank_lll = rank_lll_old;
    }
    if (rank_lll > 5 || rank_hhh > 3)
    {
      number_change = true;
    }
    rank_hhh_old = rank_hhh;
    rank_lll_old = rank_lll;
    double end_lengh = (end_state.col(0) - start_state.col(0)).norm();
    Eigen::Vector3d end_point_hhh = Eigen::Vector3d(cos(primitive_hhh[rank_hhh]) * end_lengh, sin(primitive_hhh[rank_hhh]) * end_lengh, 1.0);
    Eigen::Vector3d waypoint_h = start_state.col(0) + 3.0 * (end_point_hhh) / 4.0;
    final_simple_path.clear();
    for (int i = 0; i < 29; i++)
    {
      final_simple_path.push_back(start_state.col(0) + (double(i) / 30.0) * (waypoint_h - start_state.col(0)));
    }
    for (int i = 31; i < 40; i++)
    {
      final_simple_path.push_back(waypoint_h + (double(i) / 30.0) * (end_state.col(0) - waypoint_h));
    }
    /*************************************************************************************************************** */

    // bool success = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_state.col(0), end_state.col(0), simple_path, a_star_path, primitive_rrr[0], primitive_lll[0]);
    // final_simple_path = a_star_path;
    double t2 = ros::Time::now().toSec();
    cout << "tttttttttttttttttttttt:" << t2 - t1 << endl;

    // std::vector<Eigen::Vector4d> pathR = kino_search_.getKinoTraj(0.1); // astar_search_
    std::vector<Eigen::Vector4d> pathR(final_simple_path.size());
    for (int i = 0; i < final_simple_path.size() - 1; ++i)
    {
      if (i == 0)
      {
        pathR[i] = Eigen::Vector4d(final_simple_path[i][0], final_simple_path[i][1], lll_start[0], rrr_start[0]);
      }
      else
      {
        pathR[i] = Eigen::Vector4d(final_simple_path[i][0], final_simple_path[i][1], primitive_lll[rank_lll], primitive_rrr[rank_rrr]);
      }
    }
    pathR[final_simple_path.size() - 1] = Eigen::Vector4d(final_simple_path[final_simple_path.size() - 1][0], final_simple_path[final_simple_path.size() - 1][1], lll_end[0], rrr_end[0]);
    front_traj_piecenum = pathR.size();
    path_R_ = pathR;
    return;
  }
  void PolyTrajOptimizer::leader_opt_ok()
  {
    int piece_nums = trajROpt.posTraj_.getPieceNum();
    std::vector<double> dura(piece_nums);
    std::vector<Eigen::Matrix<double, 3, 6>> cMats_pos(piece_nums);
    std::vector<Eigen::Matrix<double, 1, 6>> cMats_r(piece_nums);
    std::vector<Eigen::Matrix<double, 1, 6>> cMats_l(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      cMats_pos[i] = trajROpt.posTraj_.getPiece(i).getCoeffMat();
      cMats_r[i] = trajROpt.radiusTraj_.getPiece(i).getCoeffMat();
      cMats_l[i] = trajROpt.lllTraj_.getPiece(i).getCoeffMat();
      dura[i] = trajROpt.posTraj_.getPiece(i).getDuration();
    }
    poly_traj_leader::Trajectory<3> posTraj_sub_(dura, cMats_pos);
    poly_traj_leader::Trajectory<1> radiusTraj_sub_(dura, cMats_r);
    poly_traj_leader::Trajectory<1> lllTraj_sub_(dura, cMats_l);

    posTraj_sub = posTraj_sub_;
    radiusTraj_sub = radiusTraj_sub_;
    lllTraj_sub = lllTraj_sub_;

    opt_ok = true;
  }
  double PolyTrajOptimizer::leaderGradCostP(Eigen::VectorXd &gdT)
  {
    static const double step = time_cps_.sampling_time_step;
    cline_begin = false;
    double T_sum = jerkOpt_.get_T1().sum();
    int k = floor(T_sum / step) + 1;

    /* Calculate the gdC, gdT and swarm cost */
    // choose zhou's trick, only consider 2/3 trajectory
    double time_leader_traj = ros::Time::now().toSec() - leader_opt_time;
    double swarm_formation_cost = 0.0;
    int piece_of_idx(0);
    double accumulated_dur(0.0);
    double pre_dur(0.0);
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    Eigen::Vector3d pos, vel;
    double omg;

    for (int idx = 0; idx <= k; idx++, accumulated_dur += step)
    {
      if (idx >= k * 2 / 3)
        break;

      s1 = accumulated_dur;

      piece_of_idx = jerkOpt_.getTraj().locatePieceIdx(s1);
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(piece_of_idx * 6, 0);

      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;

      omg = (idx == 0 || idx == k) ? 0.5 : 1.0;

      // update swarm graph and calculate cost and grad
      double dJ_df, df_dt, grad_prev_t;
      Eigen::Matrix<double, 6, 3> gradViolaPc;
      Eigen::Vector3d df_dp;

      // Eigen::Vector4d trajP_ = trajROpt.getPwithR(time_leader_traj);
      // double traj_lll_ = trajROpt.get_lll(time_leader_traj);
      // Eigen::Vector3d pos_bias = trajP_[3] * Eigen::Vector3d(traj_lll_ * swarm_des_[id_in_group][0], (1.0 / traj_lll_) * swarm_des_[id_in_group][1], 0.0);
      // Eigen::Vector3d pos_formation = Eigen::Vector3d(trajP_[0], trajP_[1], 1.0) + pos_bias;

      double t_iii = std::min(std::max(time_leader_traj, 0.0), posTraj_sub.getTotalDuration());
      Eigen::Vector3d pos_traj_ = posTraj_sub.getPos(t_iii);
      Eigen::Matrix<double, 1, 1> r_traj_ = radiusTraj_sub.getPos(t_iii);
      Eigen::Matrix<double, 1, 1> l_traj_ = lllTraj_sub.getPos(t_iii);
      Eigen::Vector3d pos_bias = r_traj_(0, 0) * Eigen::Vector3d(l_traj_(0, 0) * swarm_des_opt[id_in_group][0], (1.0 / l_traj_(0, 0)) * swarm_des_opt[id_in_group][1], 0.0);
      Eigen::Vector3d pos_formation = Eigen::Vector3d(pos_traj_[0], pos_traj_[1], 1.0) + pos_bias;

      Eigen::Vector3d dist_vec = pos - pos_formation;
      double dist = dist_vec.norm();
      double dist2 = dist_vec.squaredNorm();
      if (dist2 > 0)
      {
        dJ_df = 2 * wei_formation_prl * step * omg * dist;
        df_dp << dist_vec(0) / dist,
            dist_vec(1) / dist,
            dist_vec(2) / dist;
        gradViolaPc = dJ_df * beta0 * df_dp.transpose();
        df_dt = df_dp.dot(vel);
        grad_prev_t = -1 * dJ_df * df_dt;

        // cost
        swarm_formation_cost += wei_formation_prl * step * omg * dist2;

        // gdC
        jerkOpt_.get_gdC().block<6, 3>(piece_of_idx * 6, 0) += gradViolaPc;

        // gdT
        if (piece_of_idx > 0)
          gdT.head(piece_of_idx).array() += grad_prev_t;
      }
      time_leader_traj += step;
    }
    return swarm_formation_cost;
  }
  bool PolyTrajOptimizer::clc_form_error(const Eigen::Vector3d &pos_mine) // false == use leader ///// true == use graph
  {
    // return false;
    int size = swarm_trajs_->size();
    if (drone_id_ == sparse_id.back() && size < drone_id_)
      size++;
    if (size < sparse_id.back() - 1)
      return false;
    int fake_num = graph_size;
    if (drone_id_ == sparse_id.back())
    {
      fake_num--;
    }
    if (fake_num > swarm_trajs_->size())
    {
      return false;
    }
    for (int i = 0, fake_id = 0; i < fake_num; i++)
    {

      if (i == sparse_id[fake_id] && swarm_trajs_->at(i).drone_id < 0 && sparse_id[fake_id] != drone_id_)
      {
        return false;
      }

      if (i == sparse_id[fake_id])
      {
        fake_id++;
      }
    }
    /********************************************************************************************************************/
    int debug_num = sparse_id.size();
    std::vector<int> adj_in_cline, adj_out_cline;
    for (int i = 0; i < debug_num; i++)
    {
      for (int j = 0; j < i; j++)
      {
        adj_in_cline.push_back(i);
        adj_out_cline.push_back(j);
      }
    }
    Eigen::VectorXi assignment_cline = Eigen::VectorXi::LinSpaced(debug_num, 0, debug_num - 1);

    std::vector<Eigen::Vector3d> swarm_pos(sparse_id.size());
    std::vector<Eigen::Vector3d> graph_des_opt(sparse_id.size());
    for (int i = 0, fake_id = 0; i < swarm_des_opt.size(); i++)
    {

      if (i == sparse_id[fake_id])
      {
        graph_des_opt[fake_id] = swarm_des_opt[i];
        fake_id++;
      }
    }
    SwarmGraph swarm_graph;

    swarm_graph.setnewform();
    swarm_graph.setDesiredForm(graph_des_opt, adj_in_cline, adj_out_cline);
    swarm_graph.setAssignment(assignment_cline);

    for (int id = 0; id < sparse_id.size(); id++)
    {
      Eigen::Vector3d pos;
      if (drone_id_ == sparse_id[id])
      {
        // own pos and vel, set as zero now. And they will be updated in the optimized loop
        pos = pos_mine;
      }
      else
      {
        // others pos and vel
        double t_in_st = ros::Time::now().toSec() - swarm_trajs_->at(sparse_id[id]).start_time + 0.1;
        if (t_in_st < swarm_trajs_->at(sparse_id[id]).duration)
        {
          pos = swarm_trajs_->at(sparse_id[id]).traj.getPos(t_in_st);
        }
        else
        {
          return false;
        }
      }
      swarm_pos[id] = pos;
    }

    swarm_graph.updateGraph(swarm_pos);
    double similarity_error = 0.0;
    swarm_graph.calcFNorm2(similarity_error);
    
    if (similarity_error > error_to_leader_)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  bool PolyTrajOptimizer::pathR_MinTraj(const Eigen::MatrixXd &iniState,
                                        const Eigen::MatrixXd &finState,
                                        vector<Eigen::Vector3d> &simple_path,
                                        Eigen::MatrixXd &ctl_points,
                                        poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::MatrixXd finState_ = finState;
    Eigen::MatrixXd iniState_ = iniState;
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos = finState.col(0);
    std::vector<Eigen::Vector3d> a_star_path;
    /* astar search and get the simple path*/
    // double time_step_front = posTraj_sub.getTotalDuration() / double(front_traj_piecenum);
    double time_step_front = posTraj_sub.getTotalDuration() / 8.0;
    if (opt_ok)
    {
      simple_path.push_back(start_pos);
      bool intep_need = false;

      for (double t_prl = ros::Time::now().toSec() - leader_opt_time + 0.2; t_prl < posTraj_sub.getTotalDuration(); t_prl += time_step_front)
      {
        Eigen::Vector3d pos_traj_ = posTraj_sub.getPos(t_prl);
        Eigen::Matrix<double, 1, 1> r_traj_ = radiusTraj_sub.getPos(t_prl);
        Eigen::Matrix<double, 1, 1> l_traj_ = lllTraj_sub.getPos(t_prl);
        Eigen::Vector3d pos_bias = r_traj_(0, 0) * Eigen::Vector3d(l_traj_(0, 0) * swarm_des_opt[id_in_group][0], (1.0 / l_traj_(0, 0)) * swarm_des_opt[id_in_group][1], 0.0);
        Eigen::Vector3d pos_formation = Eigen::Vector3d(pos_traj_[0], pos_traj_[1], 1.0) + pos_bias;
        // if (!intep_need && (start_pos - pos_formation).norm() >= 1.0 && (start_pos - pos_formation).norm() <= 4.5)
        // {
        //   Eigen::Vector3d intep_pos = start_pos + 0.9 * ((pos_formation - start_pos).normalized());
        //   while ((intep_pos - start_pos).norm() <= (pos_formation - start_pos).norm())
        //   {
        //     simple_path.push_back(intep_pos);
        //     intep_pos += 0.9 * ((pos_formation - start_pos).normalized());
        //   }
        // }
        // else
        {
          simple_path.push_back(pos_formation);
        }
        intep_need = true;
      }
      double t_prl = posTraj_sub.getTotalDuration();
      Eigen::Vector3d pos_traj_ = posTraj_sub.getPos(t_prl);
      Eigen::Matrix<double, 1, 1> r_traj_ = radiusTraj_sub.getPos(t_prl);
      Eigen::Matrix<double, 1, 1> l_traj_ = lllTraj_sub.getPos(t_prl);
      Eigen::Vector3d pos_bias = r_traj_(0, 0) * Eigen::Vector3d(l_traj_(0, 0) * swarm_des_opt[id_in_group][0], (1.0 / l_traj_(0, 0)) * swarm_des_opt[id_in_group][1], 0.0);
      Eigen::Vector3d pos_formation = Eigen::Vector3d(pos_traj_[0], pos_traj_[1], 1.0) + pos_bias;
      end_pos = pos_formation;
      finState_.col(0) = end_pos;
    }
    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }

    frontendMJ.reset(iniState_, finState_, piece_num);

    /* generate init traj*/
    double des_vel = 1.2 * max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      if (piece_num == 2)
      {
        time_vec(0) = (innerPts.col(0) - start_pos).norm() / des_vel;
        time_vec(1) = (innerPts.col(0) - end_pos).norm() / des_vel;
      }
      else
      {
        for (int i = 1; i <= piece_num; ++i)
        {
          time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                     : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
        }
      }

      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.2;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ * 1.2 && debug_num <= 5);

    poly_traj::Trajectory traj;
    traj = frontendMJ.getTraj();

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);

    return true;
  }
  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh, double id)
  {
    nh_ = nh;
    formationchange = nh.subscribe("/formation_change", 10, &PolyTrajOptimizer::FormationChange, this);
    nh.param("search/use_kino", use_kino_, false);
    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_smooth", wei_smooth_, 1.0);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/weight_formation", wei_formation_, -1.0);
    nh.param("optimization/weight_formation2", wei_formation2_, -1.0);
    nh.param("optimization/weight_graph", wei_graph_, -1.0);
    nh.param("optimization/use_graph", use_graph_, false);
    nh.param("optimization/error_to_leader", error_to_leader_, 0.0);
    nh.param("optimization/weight_gather", wei_gather_, -1.0);

    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/swarm_gather_threshold", swarm_gather_threshold_, -1.0);
    nh.param("optimization/formation_type", formation_type_, -1);
    nh.param("size_swarm", size_swarm, -1);
    nh.param("max_graph_aware", max_graph_aware, 0.0);
    nh.param("vis_step", vis_step, 10);
    nh.param("vis_hor", vis_hor, 7.5);
    nh.param("optimization/formation_method_type", formation_method_type_, 0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_vel2", max_vel2_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/is_opt_formation_z", is_opt_formation_z_, true);
    nh.param("optimization/fixed_formation_z", fixed_formation_z_, 1.0);

    nh.param("optimization/enable_fix_step", enable_fix_step_, false);
    nh.param("optimization/sampling_time_step", time_cps_.sampling_time_step, 0.0);
    nh.param("optimization/enable_decouple_swarm_graph", time_cps_.enable_decouple_swarm_graph, false);
    max_vel_real = max_vel_;
    wei_formation_prl = wei_formation_;
    formation_change_get = false;
    if (id == 0)
    {
      wei_formation_ = 100;
    }
    // set the formation type
    swarm_graph_.reset(new SwarmGraph);
    setDesiredFormation(formation_type_, id);

    // benchmark position-based formation setting
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(2.6, -1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, -3.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(-2.6, -1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(-2.6, 1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(2.6, 1.5, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(5.2, -3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, -6.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(-5.2, -3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(-5.2, 3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 6.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(5.2, 3.0, 0.0));
  }

} // namespace ego_planner