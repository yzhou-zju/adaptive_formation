// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() { last_kino_success = true; }

  EGOPlannerManager::~EGOPlannerManager()
  {
    log_ztr.close();
    // std::cout << "des manager" << std::endl;
  }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    nh.param("search/use_kino", use_kino, false);
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    odom_sub = nh.subscribe("/drone_" + to_string(pp_.drone_id) + "_visual_slam/odom", 10, &EGOPlannerManager::odom_cmb, this, ros::TransportHints().tcpNoDelay());
    // for ztr debug---------------------------------------------------------------------------------------
    log_ztr.open("/home/ubuntu/formation_zy/log/all_40/time_" + to_string(pp_.drone_id) + ".txt");
    // for ztr debug---------------------------------------------------------------------------------------
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh, pp_.drone_id);
    ploy_traj_opt_->setEnvironment(nh, grid_map_, pp_.drone_id);
    Astar_pub = nh.advertise<visualization_msgs::MarkerArray>("planning/atar_pub", 10);
    init_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("planning/init_traj_pub", 10);

    visualization_ = vis;
  }
  std::vector<Eigen::Vector4d> EGOPlannerManager::get_pathR(const Eigen::Matrix3d &start_leader_,
                                                            const Eigen::Matrix3d &end_leader_,
                                                            const vector<double> &rrr_start_,
                                                            const vector<double> &rrr_end_,
                                                            const vector<double> &lll_start_,
                                                            const vector<double> &lll_end_,
                                                            const double &opt_time_now,
                                                            double &opt_score)
  {
    /************************************************************************************* */
    Eigen::Matrix3d start_leader = start_leader_;
    Eigen::Matrix3d end_leader = end_leader_;
    std::vector<double> rrr_start = rrr_start_;
    std::vector<double> rrr_end = rrr_end_;
    std::vector<double> lll_start = lll_start_;
    std::vector<double> lll_end = lll_end_;
    ploy_traj_opt_->set_leader_path(start_leader, end_leader, rrr_start, rrr_end, lll_start, lll_end, path_R_, opt_time_now, opt_score);
    /************************************************************************************* */
    return path_R_;
  }
  void EGOPlannerManager::opt_pathR(const Eigen::Matrix3d &start_leader_,
                                    const Eigen::Matrix3d &end_leader_,
                                    const vector<double> &rrr_start_,
                                    const vector<double> &rrr_end_,
                                    const vector<double> &lll_start_,
                                    const vector<double> &lll_end_,
                                    ros::Time &time_opt_end)
  {
    Eigen::Matrix3d start_state, end_state;
    start_state.setZero();
    end_state.setZero();
    Eigen::Matrix<double, 1, 3> headRadius, tailRadius;
    Eigen::Matrix<double, 1, 3> headlll, taillll;
    headRadius.setZero();
    tailRadius.setZero();
    headlll.setZero();
    taillll.setZero();
    start_state = start_leader_;
    end_state = end_leader_;
    headRadius(0, 0) = rrr_start_[0];
    tailRadius(0, 0) = rrr_end_[0];
    headlll(0, 0) = lll_start_[0];
    taillll(0, 0) = lll_end_[0];

    std::vector<Eigen::Vector4d> pathR_zy;
    std::vector<double> path_lll;
    pathR_zy.resize(path_R_.size());
    path_lll.resize(path_R_.size());
    for (size_t i = 0; i < path_R_.size(); i++)
    {
      pathR_zy[i] = Eigen::Vector4d(path_R_[i][0], path_R_[i][1], 1.0, path_R_[i][3]);
      path_lll[i] = path_R_[i][2];
    }
    path_R_.clear();
    // cout << "start_state::::::"<<start_state.col(0) << endl;
    // cout << "end_state::::::"<<end_state.col(0) << endl;
    // cout << "headRadius::::::"<<headRadius(0,0) << endl;
    // cout << "tailRadius::::::"<<tailRadius(0,0) << endl;
    // cout << "headlll::::::"<<headlll(0,0) << endl;
    // cout << "taillll::::::"<<taillll(0,0) << endl;
    ploy_traj_opt_->trajROpt.OptimizeGlobalTrajectory(start_state, end_state, headRadius, tailRadius, headlll, taillll, pathR_zy, path_lll);
    time_opt_end = ros::Time::now();
    ploy_traj_opt_->leader_opt_ok();
    return;
  }
  // void EGOPlannerManager::get_pathR_lll(const double &t, const Eigen::Vector4d &pos_R, const double &lll)
  // {
  //   Eigen::Vector4d trajP = trajROpt.getPwithR(t);
  //   double traj_lll = trajROpt.get_lll(t);
  //   pos_R = trajP;
  //   lll = traj_lll;
  //   return ;
  // }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88");
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  // 计算初始的Minimal jerk轨迹，等待细看，从start点计算到localtarget得到初始化Minco轨迹存入initMJO
  // 考虑失败了就用A_astar规划出来的轨迹不用hybrid_astar

  int EGOPlannerManager::findRank(std::vector<double> &data, double value)
  {
    // 首先对数据进行排序
    std::sort(data.begin(), data.end());

    // 使用二分查找确定元素的位置
    auto it = std::lower_bound(data.begin(), data.end(), value);
    if (it != data.end() && *it == value)
    {
      return it - data.begin() + 1; // 返回排名（从1开始）
    }
    else
    {
      return -1; // 元素不在向量中
    }
  }
  bool EGOPlannerManager::computeInitReferenceState(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &start_vel,
                                                    const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &local_target_pt,
                                                    const Eigen::Vector3d &local_target_vel,
                                                    const double &ts,
                                                    poly_traj::MinJerkOpt &initMJO,
                                                    const bool flag_polyInit)
  {
    static bool flag_first_call = true;
    bool use_topo_aware = false;
    bool jerkCost = false;
    /***************topo test******************* */
    bool astar_or_leader = ploy_traj_opt_->clc_form_error(start_pt);
    
    // if (ploy_traj_opt_->opt_ok && pp_.drone_id != 0)
    if(ploy_traj_opt_->opt_ok && pp_.drone_id!=0 && (!astar_or_leader || ploy_traj_opt_->use_leader_))
    // if(false)
    {
      // double t_local_replan = pp_.planning_horizen_/pp_.max_vel
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;
      constexpr double init_of_init_totaldur = 2.0;

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
      Eigen::MatrixXd ctl_points;
      bool path_success = false;
      /* step 1: A* or Kino_a_star search and generate init traj */
      path_success = ploy_traj_opt_->pathR_MinTraj(headState, tailState, simple_path, ctl_points, initMJO);

      if (!path_success)
      {
        return false;
      }
      traj = initMJO.getTraj();
      if (initMJO.getTrajJerkCost() >= 100)
      {
        jerkCost = true;
      }
      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      // visualization_->displayAStarList(path_view, 0);//可视化前段hybrid_astar
      visualization_->ztr_debug_vis(simple_path, Eigen::Vector4d(0, 0, 1, 0.5), 0.1, 0);
      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);

      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::MarkerArray marker_array2;
      double ttt = 0.0;
      for (size_t i = 0; ttt < traj.getTotalDuration(); i += 1, ttt += 0.1)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; // 或者你使用的其他坐标系
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_R_vis";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE; // 使用球体来模拟椭圆
        marker.action = visualization_msgs::Marker::ADD;

        // 设置位置
        marker.pose.position.x = traj.getPos(ttt)[0];
        marker.pose.position.y = traj.getPos(ttt)[1];
        marker.pose.position.z = 1.0; // 高度设置为1.0

        // 设置大小
        marker.scale.x = 0.2; // x方向的大小
        marker.scale.y = 0.2; // y方向的大小
        marker.scale.z = 0.1; // z方向的大小（可以设置为较小的值）

        // 设置颜色
        marker.color.r = 1.0f;
        marker.color.g = 0.0f; // 绿色
        marker.color.b = 0.0f;
        marker.color.a = 0.8; // 不透明

        marker_array.markers.push_back(marker);
      }
      for (size_t i = 0; i < simple_path.size(); i += 1)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; // 或者你使用的其他坐标系
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_R_vis";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE; // 使用球体来模拟椭圆
        marker.action = visualization_msgs::Marker::ADD;

        // 设置位置
        marker.pose.position.x = simple_path[i][0];
        marker.pose.position.y = simple_path[i][1];
        marker.pose.position.z = 1.0; // 高度设置为1.0

        // 设置大小
        marker.scale.x = 0.5; // x方向的大小
        marker.scale.y = 0.5; // y方向的大小
        marker.scale.z = 0.1; // z方向的大小（可以设置为较小的值）

        // 设置颜色
        marker.color.r = 0.0f;
        marker.color.g = 1.0f; // 绿色
        marker.color.b = 1.0f;
        marker.color.a = 0.8; // 不透明

        marker_array2.markers.push_back(marker);
      }
      if (pp_.drone_id == 6)
      {
        Astar_pub.publish(marker_array2);
        init_traj_pub.publish(marker_array);
      }
    }
    else
    {
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;
      constexpr double init_of_init_totaldur = 2.0;

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
      // log_ztr<<"local_target_pt"<<local_target_pt.transpose()<<"local_target_vel"<<local_target_vel.transpose()<<endl;
      Eigen::MatrixXd ctl_points;
      bool path_success = false;
      /* step 1: A* or Kino_a_star search and generate init traj */
      path_success = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);

      if (!path_success)
      {
        // ROS_WARN("unable do astarWithMinTraj");
        return false;
      }
      traj = initMJO.getTraj();

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      // visualization_->displayAStarList(path_view, 0);//可视化前段hybrid_astar
      visualization_->ztr_debug_vis(simple_path, Eigen::Vector4d(0, 0, 1, 0.5), 0.1, 0);
      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
      //  visualization_->ztr_debug_vis(path_show2,Eigen::Vector4d(0,0,1,0.5), 0.1, 0);
    }

    if (jerkCost)
    {
      ploy_traj_opt_->wei_formation_prl = ploy_traj_opt_->wei_formation2_;
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;
      constexpr double init_of_init_totaldur = 2.0;

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
      // log_ztr<<"local_target_pt"<<local_target_pt.transpose()<<"local_target_vel"<<local_target_vel.transpose()<<endl;
      Eigen::MatrixXd ctl_points;
      bool path_success = false;
      /* step 1: A* or Kino_a_star search and generate init traj */
      path_success = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);

      if (!path_success)
      {
        // ROS_WARN("unable do astarWithMinTraj");
        return false;
      }
      traj = initMJO.getTraj();

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      // visualization_->displayAStarList(path_view, 0);//可视化前段hybrid_astar
      visualization_->ztr_debug_vis(simple_path, Eigen::Vector4d(0, 0, 1, 0.5), 0.1, 0);
      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
      //  visualization_->ztr_debug_vis(path_show2,Eigen::Vector4d(0,0,1,0.5), 0.1, 0);
    }
    else
    {
      ploy_traj_opt_->wei_formation_prl = ploy_traj_opt_->wei_formation_;
    }

    // log_ztr<<"hybridastarWithMinTraj---before return ture-----"<<endl;
    return true;
  }

  // 根据planning_horizen距离远近在全局轨迹traj_.global_traj
  // 上选取曲线积分距离为planning_horizen的点，作为local_target，根据global_target到local_target距离
  // 分类获取local_target速度是0还是global_traj上面对应local_target的点的速度，
  // 最后将localtarget位置速度数据调用setLocalGoal函数结合飞机id存入swarm_local_goal中
  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel)
  {
    double t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_; // 按照执行时间20等分，累加积分模拟计算轨迹长度
    // double dist_min = 9999, dist_min_t = 0.0;
    double dist = 0;
    Eigen::Vector3d last_pos_t = start_pt;

    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double delta_dist = (pos_t - last_pos_t).norm();
      dist += delta_dist; // 按照执行时间20等分，累加积分模拟计算轨迹曲线长度
      last_pos_t = pos_t;
      // dist = (pos_t - last_pos_t).norm();

      if (dist >= planning_horizen) // 达到planning_horizen，记录下local_target_pos
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }
    // 根据planning_horizen，长度大于global_traj长度，设置global_traj终点为local_goal
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    } // why??????????????????为什么
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }

    traj_.setLocalGoal(local_target_pos, local_target_vel, pp_.drone_id);
  }

  void EGOPlannerManager::getLocalTargetHorizenTime(
      const double planning_horizen_time, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel)
  {
    double t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    t = traj_.global_traj.glb_t_of_lc_tgt + planning_horizen_time;
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration)
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
    }
    else
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      local_target_pos = pos_t;
      traj_.global_traj.glb_t_of_lc_tgt = t;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }

    traj_.setLocalGoal(local_target_pos, local_target_vel, pp_.drone_id);
  }

  // 封装了算法：得到全局/局部轨迹的路径点，前段A*得到不同安全轨迹控制点，使用L-BFGS优化控制点，得到最优轨迹控制点
  ////先初始化轨迹，利用mini-snap的方式完成，或者从之前的轨迹再生成轨迹；接着优化轨迹；最后重新调整时间段分配比例？？？
  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const double trajectory_start_time, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool use_formation, const bool have_local_traj)
  {
    ros::Time t_start_replan = ros::Time::now();
    static int count = 0;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      // cout << "Close to goal" << endl;
      // continous_failures_count_++;
      // return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;
    // 这段初始化轨迹InitMinco轨迹是从当前位置到local_target_pt，local_target_vel得到initMJO
    /* well designed front-end init trajectory for ESDF-based planner */
    poly_traj::MinJerkOpt initMJO;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc,
                                   local_target_pt, local_target_vel,
                                   ts, initMJO, flag_polyInit))
    {
      return false;
    }
    Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());

    ploy_traj_opt_->setControlPoints(cstr_pts); // 设置控制点cps_为points

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set; // 即初始化的minco轨迹点
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0); // 可视化Init轨迹，可视化的点为控制点，等时间采样的3*(N*K+1)，蓝色可视化

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();         // 3*(N+1)点位置
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1); // 中间路径点3*(N-1)维度
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);

    flag_success = ploy_traj_opt_->optimizeTrajectory_lbfgs(headState, tailState,
                                                            innerPts, initTraj.getDurations(),
                                                            cstr_pts, use_formation);

    t_opt = ros::Time::now() - t_start;

    // save and display planned results
    static int count_fail = 0;
    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      count_fail++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec() * 1000;
    count_success++;
    average_plan_time_ = sum_time / count_success;

    // calculate the constriant-aware of formation
    Eigen::Vector3d form_grad_sum, obs_grad_sum;
    double obs_dist_sum, aware, t_duration;
    double vis_rate = 100.0;
    // vis_rate = ploy_traj_opt_->sumvisAware();
    // bool is_use_aware = ploy_traj_opt_->sumConstrainAware(form_grad_sum, obs_grad_sum, obs_dist_sum, aware, t_duration);
    bool is_use_aware;
    is_use_aware = false;
    // set the right start time of local traj
    double local_traj_time;
    // if (have_local_traj && use_formation)
    if (have_local_traj && use_formation)
    {
      double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      if (delta_replan_time > 0)
        ros::Duration(delta_replan_time).sleep();

      local_traj_time = trajectory_start_time;
    }
    else
    {
      local_traj_time = ros::Time::now().toSec();
    }

    if (is_use_aware)
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), local_traj_time, aware, vis_rate);
    else
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), local_traj_time, vis_rate);

    visualization_->displayOptimalList(cstr_pts, 0); // 发布Minco优化后的光滑局部轨迹，此处即为红色
    // success. YoY
    continous_failures_count_ = 0;
    ros::Time t_end_replan = ros::Time::now();
    log_ztr << (t_end_replan - t_start_replan).toSec() << endl;
    return true;
  }

  // minco优化出stopMJO轨迹，设置traj_.setLocalTraj为其，尚未细看????
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));
    double vis_rate = 100.0;
    traj_.setLocalTraj(stopMJO.getTraj(), ros::Time::now().toSec(), vis_rate);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < ploy_traj_opt_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_;
    Eigen::VectorXd time_vec(waypoints.size());
    int try_num = 0;
    do
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;

      des_vel /= 1.2;
      try_num++;
    } while (globalMJO.getTraj().getMaxVelRate() > pp_.max_vel_ * 1.2 && try_num <= 5);

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

  // 获取minco全局轨迹使用traj_.setGlobalTraj,combine two minco trajectory to get the global trajectory
  //   global trajectory is consisted by two minco trajectory:(1) from "start" to "remap_lg"(2) from "remap_lg" to "end"
  // 感觉remap_lg的选取可以优化!!!!!!!!!!!!
  bool EGOPlannerManager::planGlobalTrajReMap(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &remap_lg_pos, const Eigen::Vector3d &remap_lg_vel, const Eigen::Vector3d &remap_lg_acc,
      const std::vector<Eigen::Vector3d> &wps, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    /*
      global trajectory replanning for swarm formation.
      global trajectory is consisted by two minco trajectory:
        (1) from "start" to "remap_lg"
        (2) from "remap_lg" to "end"
      by doing so, we can constrain the state of remap_lg.
    */
    // generate the first minco trajectory
    poly_traj::MinJerkOpt globalMJO_first;
    Eigen::Matrix<double, 3, 3> headState_first, tailState_first;
    headState_first << start_pos, start_vel, start_acc;
    tailState_first << remap_lg_pos, remap_lg_vel, remap_lg_acc;
    std::vector<Eigen::Vector3d> waypoints_first;
    waypoints_first.push_back(remap_lg_pos);

    globalMJO_first.reset(headState_first, tailState_first, waypoints_first.size());
    Eigen::MatrixXd innerPts_first;
    double des_vel = pp_.max_vel_;
    int try_num = 0;
    Eigen::VectorXd time_vec_first(waypoints_first.size());

    do // 降低速度到getMaxVelRate<pp_.max_vel_*1.2
    {
      time_vec_first(0) = (remap_lg_pos - start_pos).norm() / des_vel;
      globalMJO_first.generate(innerPts_first, time_vec_first);
      // innerPts_first为空，直接生成headState_first到local_goal即粉色多变形所在的minco多项式轨迹。

      // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug-------------------------------
      //  for (size_t i = 0; i < wps.size(); i++)
      //  {
      //    log_ztr << "wps" << i << ": " << wps[i].transpose() << std::endl;
      //  }
      //  log_ztr << "innerPts_first:" << innerPts_first.transpose() << std::endl
      //          << "waypoints_first:" << waypoints_first[0].transpose() << std::endl
      //          << "time_vec_first:" << time_vec_first.transpose() << std::endl
      //          << "waypoints_first.size():" << waypoints_first.size() << std::endl
      //          << "try_num:" << try_num << "------------------------first minco--------------------------------" << std::endl;
      // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug-------------------------------
      des_vel /= 1.2;
      try_num++;
    } while (globalMJO_first.getTraj().getMaxVelRate() > pp_.max_vel_ * 1.2 && try_num <= 5);

    // generate the second minco trajectory
    poly_traj::MinJerkOpt globalMJO_second;
    Eigen::Matrix<double, 3, 3> headState_second, tailState_second;
    headState_second << remap_lg_pos, remap_lg_vel, remap_lg_acc;
    tailState_second << wps.back(), end_vel, end_acc;
    std::vector<Eigen::Vector3d> waypoints_second;
    waypoints_second = wps;

    globalMJO_second.reset(headState_second, tailState_second, waypoints_second.size());
    Eigen::MatrixXd innerPts_second;
    // bug----------------------------------------------------------------------此处waypoints_second.size为0
    innerPts_second.resize(3, waypoints_second.size() - 1);
    for (int i = 0; i < waypoints_second.size() - 1; i++)
      innerPts_second.col(i) = waypoints_second[i];

    des_vel = pp_.max_vel_;
    try_num = 0;
    Eigen::VectorXd time_vec_second(waypoints_second.size());
    do
    {
      for (size_t i = 0; i < waypoints_second.size(); ++i)
      {
        time_vec_second(i) = (i == 0) ? (waypoints_second[0] - remap_lg_pos).norm() / des_vel
                                      : (waypoints_second[i] - waypoints_second[i - 1]).norm() / des_vel;
      }
      // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug--------------------------------
      //  log_ztr << "innerPts_second:" << innerPts_second.transpose() << std::endl
      //          << "waypoints_second:" << waypoints_second[0].transpose() << std::endl
      //          << "time_vec_second:" << time_vec_second.transpose() << std::endl
      //          << "waypoints_second.size():" << waypoints_second.size() << std::endl
      //          << "try_num:" << try_num << "------------------------second minco--------------------------------" << std::endl;
      // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug--------------------------------

      globalMJO_second.generate(innerPts_second, time_vec_second);

      des_vel /= 1.2;
      try_num++;
    } while (globalMJO_second.getTraj().getMaxVelRate() > pp_.max_vel_ * 1.2 && try_num <= 5);

    // combine two minco trajectory to get the global trajectory
    poly_traj::Trajectory global_traj, traj_2;
    global_traj = globalMJO_first.getTraj();
    traj_2 = globalMJO_second.getTraj();
    // 此处gloabl_traj 好像有Bug------------------------------------debug------------------------------------------------------------------------
    // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug--------------------------------
    //  log_ztr << "globalMJO_first:" << global_traj.getPieceNum() << std::endl
    //          << "globalMJO_second:" << traj_2.getPieceNum() << std::endl
    //          <<"traj_2 time"<<traj_2.getTotalDuration()<<std::endl
    //            <<"globalMJO_first"<<global_traj.getTotalDuration()<<std::endl
    //          << "------------------------second minco--------------------------------" << std::endl;

    constexpr double step_size_t = 0.1;
    int i_end = floor(global_traj.getDurations().sum() / step_size_t);
    vector<Eigen::Vector3d> path_show(i_end);
    for (int i = 0; i < i_end; i++) // 以0.1时间分辨率显示绿色全局轨迹
    {
      path_show[i] = global_traj.getPos(i * step_size_t);
    }

    int i_end2 = floor(traj_2.getDurations().sum() / step_size_t);
    vector<Eigen::Vector3d> path_show2(i_end2);
    for (int i = 0; i < i_end2; i++) // 以0.1时间分辨率显示绿色全局轨迹
    {
      path_show2[i] = traj_2.getPos(i * step_size_t);
    }
    // visualization_->ztr_debug_vis(path_show2,Eigen::Vector4d(0,0,1,0.5), 0.1, 0);
    // visualization_->ztr_debug_vis(path_show,Eigen::Vector4d(1,0,0,0.5), 0.1, 0);

    // 此处waypoints或者innerPts_second好像有Bug--------------------------------debug------------------------------------------------------------
    global_traj.append(traj_2);

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(global_traj, time_now.toSec());

    // traj_.global_traj

    return true;
  }

  // 其中getOptGoals尚未细看，依据localgoal作为nodes_horizon，现在的nodes_cur，还有aware_weight得到opt_goals与opt_assignment
  void EGOPlannerManager::callAlignAssign(const std::vector<Eigen::Vector3d> &nodes_horizon,
                                          const std::vector<Eigen::Vector3d> &nodes_cur,
                                          const Eigen::VectorXd &aware_weight,
                                          std::vector<Eigen::Vector3d> &opt_goals,
                                          Eigen::VectorXi &opt_assignment)
  {
    std::vector<Eigen::Vector3d> nodes_des;
    // get desire formation
    nodes_des = ploy_traj_opt_->getSwarmGraphInit();

    // align_assign_.getOptGoals(nodes_cur, nodes_des, aware_weight, opt_goals, opt_assignment, true);
    align_assign_.getOptGoals(nodes_horizon, nodes_cur, nodes_des, aware_weight, opt_goals, opt_assignment, true, true);
  }
  void EGOPlannerManager::callAssign(const std::vector<Eigen::Vector3d> &nodes_cur,
                                     const std::vector<Eigen::Vector3d> &nodes_des,
                                     Eigen::VectorXi &opt_assignment)
  {

    // align_assign_.getOptGoals(nodes_cur, nodes_des, aware_weight, opt_goals, opt_assignment, true);
    align_assign_.getOptassig(nodes_cur, nodes_des, opt_assignment);
  }
  // 传入当前odom位置以及得到t1时刻formation-error
  double EGOPlannerManager::getFormationError(double t, Eigen::Vector3d pos)
  {
    vector<Eigen::Vector3d> swarm_graph_pos;

    // get pos of swarm formation
    swarm_graph_pos = getFormationCurrentPos(t, pos); // 代码有问题，已经修改了!!!!!返回所有uav位置信息

    // debug
    int formation_size = ploy_traj_opt_->getFormationSize();

    // calculate formation error
    double formation_error = ploy_traj_opt_->getFormationError(swarm_graph_pos);

    // debug
    if (isnan(formation_error))
    {
      int size = swarm_graph_pos.size();
      ROS_WARN("formation_error is nan");
      for (int i = 0; i < size; i++)
      {
        // cout << "[" << i << "] : " << swarm_graph_pos[i].transpose() << endl;
        if (isnan(swarm_graph_pos[i](0)))
        {
          poly_traj::Trajectory traj = traj_.swarm_traj[i].traj;
          int piece_num = traj.getPieceNum();
          // cout << "piece time : " << traj.getDurations().transpose() << endl;
        }
      }
    }

    return formation_error;
  }

  // 代码有问题，已经修改了!!!!!返回所有uav位置信息
  vector<Eigen::Vector3d> EGOPlannerManager::getFormationCurrentPos(double t, Eigen::Vector3d pos)
  {
    int formation_size = ploy_traj_opt_->getFormationSize();
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size);

    // get pos of swarm formation
    swarm_graph_pos[pp_.drone_id] = pos;
    for (int id = 0; id < formation_size; id++)
    {
      if (traj_.swarm_traj[id].drone_id < 0 || traj_.swarm_traj[id].drone_id == pp_.drone_id)
        continue;

      double traj_i_satrt_time = traj_.swarm_traj[id].start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (t < traj_i_satrt_time + traj_.swarm_traj[id].duration)
      {
        swarm_p = traj_.swarm_traj[id].traj.getPos(t - traj_i_satrt_time);
        swarm_v = traj_.swarm_traj[id].traj.getVel(t - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = t - (traj_i_satrt_time + traj_.swarm_traj[id].duration);
        swarm_v = traj_.swarm_traj[id].traj.getVel(traj_.swarm_traj[id].duration);
        swarm_p = traj_.swarm_traj[id].traj.getPos(traj_.swarm_traj[id].duration) + exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
    }
    return swarm_graph_pos;
  }
  vector<Eigen::Vector3d> EGOPlannerManager::getCurrentPos_all(double t, Eigen::Vector3d pos, vector<Eigen::Vector3d> nodes_des)
  {
    vector<Eigen::Vector3d> swarm_graph_pos;

    // get pos of swarm formation
    for (int id = 0; id < nodes_des.size(); id++)
    {
      if (nodes_des[id][2] == -99.0)
        continue;

      double traj_i_satrt_time = traj_.swarm_traj[id].start_time;
      Eigen::Vector3d swarm_p, swarm_v;
      if (id != pp_.drone_id)
      {
        if (t < traj_i_satrt_time + traj_.swarm_traj[id].duration)
        {
          swarm_p = traj_.swarm_traj[id].traj.getPos(t - traj_i_satrt_time);
          swarm_v = traj_.swarm_traj[id].traj.getVel(t - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = t - (traj_i_satrt_time + traj_.swarm_traj[id].duration);
          swarm_v = traj_.swarm_traj[id].traj.getVel(traj_.swarm_traj[id].duration);
          swarm_p = traj_.swarm_traj[id].traj.getPos(traj_.swarm_traj[id].duration) + exceed_time * swarm_v;
        }
      }
      else
      {
        swarm_p = pos;
      }
      swarm_graph_pos.push_back(swarm_p);
    }
    return swarm_graph_pos;
  }
  // 依靠缓冲区swarm_traj，计算得到当前t再过delta_t时间所有uav的位置速度等等存入swarm_graph_pos与swarm_graph_vel
  void EGOPlannerManager::getFormationPosAndVel(double t, double delta_t,
                                                vector<Eigen::Vector3d> &swarm_graph_pos,
                                                vector<Eigen::Vector3d> &swarm_graph_vel)
  {
    int formation_size = ploy_traj_opt_->getFormationSize();
    swarm_graph_pos.resize(formation_size);
    swarm_graph_vel.resize(formation_size);

    double t0 = t + delta_t;

    // get pos and vel of swarm formation
    for (int id = 0; id < formation_size; id++)
    {
      Eigen::Vector3d swarm_p, swarm_v;
      double traj_i_satrt_time;

      if (id == pp_.drone_id)
      {
        traj_i_satrt_time = traj_.local_traj.start_time;
        if (t0 < traj_i_satrt_time + traj_.local_traj.duration)
        {
          swarm_p = traj_.local_traj.traj.getPos(t0 - traj_i_satrt_time);
          swarm_v = traj_.local_traj.traj.getVel(t0 - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = t0 - (traj_i_satrt_time + traj_.local_traj.duration);
          swarm_v = traj_.local_traj.traj.getVel(traj_.local_traj.duration);
          swarm_p = traj_.local_traj.traj.getPos(traj_.local_traj.duration) + exceed_time * swarm_v;
        }
        swarm_graph_pos[id] = swarm_p;
        swarm_graph_vel[id] = swarm_v;
        continue;
      }

      traj_i_satrt_time = traj_.swarm_traj[id].start_time;
      if (t0 < traj_i_satrt_time + traj_.swarm_traj[id].duration)
      {
        swarm_p = traj_.swarm_traj[id].traj.getPos(t0 - traj_i_satrt_time);
        swarm_v = traj_.swarm_traj[id].traj.getVel(t0 - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = t0 - (traj_i_satrt_time + traj_.swarm_traj[id].duration);
        swarm_v = traj_.swarm_traj[id].traj.getVel(traj_.swarm_traj[id].duration);
        swarm_p = traj_.swarm_traj[id].traj.getPos(traj_.swarm_traj[id].duration) + exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }
  }

  bool EGOPlannerManager::planSwarmGlobalPath(const Eigen::Vector3d &start_pos,
                                              const Eigen::Vector3d &goal_pos,
                                              const double &swarm_scale,
                                              std::vector<Eigen::Vector3d> &swarm_global_path)
  {
    bool success = false;

    // test: add global_rrt here
    swarm_global_path.clear();
    swarm_global_path.push_back(Eigen::Vector3d(-13.0, 4.0, 0.5));
    swarm_global_path.push_back(Eigen::Vector3d(0.0, -4.0, 0.5));
    swarm_global_path.push_back(Eigen::Vector3d(13.0, 4.0, 0.5));
    swarm_global_path.push_back(Eigen::Vector3d(25.0, 0.0, 0.5));

    success = true;
    return success;
  }

  void EGOPlannerManager::form_change_get(bool get_change_formation)
  {
    ploy_traj_opt_->formation_change(get_change_formation);
  }

} // namespace ego_planner
