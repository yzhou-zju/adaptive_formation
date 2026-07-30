
#include <plan_manage/ego_replan_fsm.h>
#include <ros/time.h>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    //-----------------------------------------------ztr debug-------------------------------
    log_ztr2.close();
    log_zy.close();
    //-----------------------------------------------ztr debug-------------------------------}
  }
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    nh_ = nh;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    central_state_ = CENTRAL_FSM_STATE::INIT_2;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    flag_relan_astar_ = false;
    have_local_goal_near_target_ = false;
    isuse_local_vel_ = false;
    have_local_traj_ = false;
    formation_change_fsm = false;
    group_id_.resize(100);
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_max_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/formation_error_clearance", formation_error_clearance_, 0.1);
    nh.param("fsm/aware_clearance", aware_clearance_, 0.1);
    nh.param("fsm/softmax_aware_clearance", softmax_aware_clearance_, 0.1);
    nh.param("fsm/ca_delta_t", ca_delta_t_, 0.1);
    nh.param("fsm/isuse_local_vel", isuse_local_vel_, false);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("size_swarm", size_swarm, -1);
    nh.param("vis_hor", vis_hor, 4.5);

    have_trigger_ = !flag_realworld_experiment_;
    planning_horizen_ = planning_horizen_max_;
    planning_horizen_min_ = 3.0;
    swarm_near.resize(size_swarm);
    for (int i = 0; i < size_swarm; i++)
    {
      swarm_near[i] = {999.0, 999.0, 999.0}; // default
    }
    {
      for(int i=0;i<swarm_des_set_.size();i++)
      {
        if(swarm_des_set_[i][2]!=-99.0)
        {
          desired_points.push_back(swarm_des_set_[i]);
        }
      }
    }
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();
    /* callback */
    formation_change = nh.subscribe<traj_utils::FormationType>("/formation_change", 10, &EGOReplanFSM::FormationChange, this, ros::TransportHints().tcpNoDelay());
    exec_timer_ = nh.createTimer(ros::Duration(0.02), &EGOReplanFSM::execFSMCallback, this);

    odom_sub_z = nh_.subscribe("/others_odom", 100, &EGOReplanFSM::rcvOtherOdomCallback, this, ros::TransportHints().tcpNoDelay());
    // safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    // 集群中心，只有其具有swarm_local_goal数据，其余飞机没有这个消息？？？？？?是否合理
    if (planner_manager_->pp_.drone_id == 0)
    {
      centra_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::centralFSMCallback, this);
      pathR_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::formationCallback, this); // const ros::TimerEvent &e
      error_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::errorCallback, this); // const ros::TimerEvent &e
      // stop_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::stopCallback, this);       // const ros::TimerEvent &e
      pathR_pub = nh.advertise<visualization_msgs::MarkerArray>("planning/leader_pathr", 10);
      opt_pathR_pub = nh.advertise<visualization_msgs::MarkerArray>("planning/opt_pathr", 10);
      posR_traj_pub_ = nh.advertise<traj_utils::pathR_lll>("planning/leader_trajectory_pub", 10);
    }
    posR_traj_sub_ = nh.subscribe<traj_utils::pathR_lll>("planning/leader_trajectory_sub", 100,
                                                         &EGOReplanFSM::pathR_TrajCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
    agent_addout_sub = nh.subscribe<traj_utils::add_out>("planning/begin_add_out", 100,
                                                         &EGOReplanFSM::add_out_Callback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
    agent_addout_pub = nh.advertise<traj_utils::add_out>("planning/agent_add_out", 10);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::PolyTraj>("planning/broadcast_traj_recv", 100,
                                                                 &EGOReplanFSM::RecvBroadcastPolyTrajCallback,
                                                                 this,
                                                                 ros::TransportHints().tcpNoDelay());
    // 接受广播的轨迹，按照id号存入缓存区planner_manager_->traj_.swarm_traj
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    broadcast_localgoal_pub_ = nh.advertise<traj_utils::LocalGoal>("planning/broadcast_localgoal_send", 10);
    broadcast_localgoal_sub_ = nh.subscribe<traj_utils::LocalGoal>("planning/broadcast_localgoal_recv", 100,
                                                                   &EGOReplanFSM::RecvBroadcastLocalGoalCallback,
                                                                   this,
                                                                   ros::TransportHints().tcpNoDelay());

    broadcast_remaplocalgoal_pub_ = nh.advertise<traj_utils::RemapLocalGoalList>("planning/broadcast_remaplocalgoal_send", 10);
    broadcast_remaplocalgoal_sub_ = nh.subscribe<traj_utils::RemapLocalGoalList>("planning/broadcast_remaplocalgoal_recv", 100,
                                                                                 &EGOReplanFSM::RecvBroadcastRemapLocalGoalCallback,
                                                                                 this,
                                                                                 ros::TransportHints().tcpNoDelay());

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);

    // init assignment
    assignment_.resize(size_swarm);
    for (int i = 0; i < size_swarm; i++)
      assignment_(i) = i;
    for (int i = 0; i < size_swarm; i++)
      swarm_des_id.push_back(i);

    num_now_ = 20;
    form_change = 0;
    assig_pos_.resize(size_swarm);
    assig_id.resize(num_now_);
    for (int i = 0; i < num_now_; i++)
      assig_id[i] = i;
    for (int i = num_now_; i < size_swarm; i++)
      swarm_des_id[i] = -99;
    // choose the model of global goal setting
    setGlobalGoal(nh);
    odom_pos_ = Eigen::Vector3d(-99.0, -99.0, -99.0);
    // for ztr debug---------------------------------------------------------------------------------------
    if(planner_manager_->pp_.drone_id==0)
    {
      log_zy.open("/home/mark/iros/code/xin_ok/error_re.txt");
    }
    log_ztr2.open("/home/ztr/Pipline-Swarm-Formation/log/fsm_drone_id_" + std::to_string(planner_manager_->pp_.drone_id) + ".txt");
    // for ztr debug---------------------------------------------------------------------------------------
  }
double EGOReplanFSM::matchAndComputeErrorWithScale(
    const std::vector<Eigen::Vector3d>& expected,
    const std::vector<Eigen::Vector3d>& current)
{
    const int N = static_cast<int>(current.size());

    if (expected.empty() || current.empty()) {
        return -1.0;
    }

    // 该版本假设两组点数量相同，但点之间没有固定对应关系
    if (expected.size() != current.size()) {
        return -1.0;
    }

    constexpr int MAX_OUTER_ITER = 15;
    constexpr int MAX_IRLS_ITER = 3;
    constexpr double EPS = 1e-9;
    constexpr double MIN_VAR = 1e-12;
    constexpr double CONVERGENCE_TOL = 1e-7;

    struct Sim3 {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        double s = 1.0;
    };

    auto applyTransform = [](const Sim3& T, const Eigen::Vector3d& p) {
        return T.s * T.R * p + T.t;
    };

    auto computeCentroid = [](const std::vector<Eigen::Vector3d>& pts) {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();

        for (const auto& p : pts) {
            c += p;
        }

        return c / static_cast<double>(pts.size());
    };

    auto computeRmsRadius = [](
        const std::vector<Eigen::Vector3d>& pts,
        const Eigen::Vector3d& c) 
    {
        double sum = 0.0;

        for (const auto& p : pts) {
            sum += (p - c).squaredNorm();
        }

        return std::sqrt(sum / static_cast<double>(pts.size()));
    };

    auto computeMaxPairDistance = [](
        const std::vector<Eigen::Vector3d>& pts) 
    {
        double max_dist = 0.0;

        for (size_t i = 0; i < pts.size(); ++i) {
            for (size_t j = i + 1; j < pts.size(); ++j) {
                max_dist = std::max(max_dist, (pts[i] - pts[j]).norm());
            }
        }

        return max_dist;
    };

    // 因为函数输入不包含 L_fma_max，这里默认用 expected 点云最大两点距离作为归一化尺度。
    // 如果你有真实的 L_fma_max，请把这一行替换成你的成员变量，例如：
    // const double L_fma_max = this->L_fma_max_;
    const double L_fma_max = computeMaxPairDistance(expected);

    if (L_fma_max <= EPS) {
        return -1.0;
    }

    // Hungarian algorithm，求最小代价一对一匹配。
    // cost[i][j] 表示 current[i] 变换后匹配 expected[j] 的代价。
    auto hungarian = [](
        const std::vector<std::vector<double>>& cost) 
    {
        const int n = static_cast<int>(cost.size());
        const int m = static_cast<int>(cost[0].size());

        const double INF = std::numeric_limits<double>::max();

        std::vector<double> u(n + 1, 0.0);
        std::vector<double> v(m + 1, 0.0);
        std::vector<int> p(m + 1, 0);
        std::vector<int> way(m + 1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;

            std::vector<double> minv(m + 1, INF);
            std::vector<char> used(m + 1, false);

            do {
                used[j0] = true;
                const int i0 = p[j0];

                double delta = INF;
                int j1 = 0;

                for (int j = 1; j <= m; ++j) {
                    if (used[j]) {
                        continue;
                    }

                    const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];

                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }

                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }

                for (int j = 0; j <= m; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;

            } while (p[j0] != 0);

            do {
                const int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        std::vector<int> assignment(n, -1);

        for (int j = 1; j <= m; ++j) {
            if (p[j] != 0) {
                assignment[p[j] - 1] = j - 1;
            }
        }

        return assignment;
    };

    auto computeAssignment = [&](const Sim3& T) {
        std::vector<std::vector<double>> cost(
            N, std::vector<double>(N, 0.0));

        for (int i = 0; i < N; ++i) {
            const Eigen::Vector3d p_cur = applyTransform(T, current[i]);

            for (int j = 0; j < N; ++j) {
                cost[i][j] = (expected[j] - p_cur).norm();
            }
        }

        return hungarian(cost);
    };

    auto computeAssignedSumError = [&](
        const Sim3& T,
        const std::vector<int>& assignment) 
    {
        double sum_error = 0.0;

        for (int i = 0; i < N; ++i) {
            const Eigen::Vector3d p_cur = applyTransform(T, current[i]);
            sum_error += (expected[assignment[i]] - p_cur).norm();
        }

        return sum_error;
    };

    // 给定匹配关系 assignment，估计最优 Sim3。
    // 使用 IRLS 近似最小化 sum ||r_i||，而不是 sum ||r_i||^2。
    auto estimateTransformIRLS = [&](
        const std::vector<int>& assignment) 
    {
        Sim3 T;
        std::vector<double> weights(N, 1.0);

        for (int irls = 0; irls < MAX_IRLS_ITER; ++irls) {
            double weight_sum = 0.0;
            Eigen::Vector3d mu_src = Eigen::Vector3d::Zero();
            Eigen::Vector3d mu_dst = Eigen::Vector3d::Zero();

            for (int i = 0; i < N; ++i) {
                const double w = weights[i];
                weight_sum += w;
                mu_src += w * current[i];
                mu_dst += w * expected[assignment[i]];
            }

            if (weight_sum <= EPS) {
                break;
            }

            mu_src /= weight_sum;
            mu_dst /= weight_sum;

            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            double var_src = 0.0;

            for (int i = 0; i < N; ++i) {
                const Eigen::Vector3d ps = current[i] - mu_src;
                const Eigen::Vector3d pd = expected[assignment[i]] - mu_dst;
                const double w = weights[i];

                // H = dst * src^T
                H += w * pd * ps.transpose();
                var_src += w * ps.squaredNorm();
            }

            if (var_src < MIN_VAR) {
                T.R = Eigen::Matrix3d::Identity();
                T.s = 1.0;
                T.t = mu_dst - mu_src;
            } else {
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                    H, Eigen::ComputeFullU | Eigen::ComputeFullV);

                const Eigen::Matrix3d U = svd.matrixU();
                const Eigen::Matrix3d V = svd.matrixV();
                const Eigen::Vector3d sigma = svd.singularValues();

                Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

                // 保证 det(R) = +1，避免镜像翻转
                if ((U * V.transpose()).determinant() < 0.0) {
                    S(2, 2) = -1.0;
                }

                // 因为 H = dst * src^T，所以 R = U * S * V^T
                T.R = U * S * V.transpose();

                const double numerator =
                    sigma(0) + sigma(1) + S(2, 2) * sigma(2);

                T.s = numerator / var_src;

                if (!std::isfinite(T.s) || T.s < 0.0) {
                    T.s = 0.0;
                }

                T.t = mu_dst - T.s * T.R * mu_src;
            }

            // 更新 IRLS 权重
            for (int i = 0; i < N; ++i) {
                const Eigen::Vector3d p_cur = applyTransform(T, current[i]);
                const double r = (expected[assignment[i]] - p_cur).norm();

                weights[i] = 1.0 / std::max(r, EPS);
            }
        }

        return T;
    };

    auto computeCovarianceBasis = [](
        const std::vector<Eigen::Vector3d>& pts,
        const Eigen::Vector3d& c) 
    {
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();

        for (const auto& p : pts) {
            const Eigen::Vector3d q = p - c;
            C += q * q.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);

        Eigen::Matrix3d E = es.eigenvectors();

        // Eigen 的特征值默认从小到大排序，这里改成主方向从大到小
        Eigen::Matrix3d B;
        B.col(0) = E.col(2);
        B.col(1) = E.col(1);
        B.col(2) = E.col(0);

        if (B.determinant() < 0.0) {
            B.col(2) *= -1.0;
        }

        return B;
    };

    const Eigen::Vector3d c_cur = computeCentroid(current);
    const Eigen::Vector3d c_exp = computeCentroid(expected);

    const double r_cur = computeRmsRadius(current, c_cur);
    const double r_exp = computeRmsRadius(expected, c_exp);

    const double s0 = r_cur > EPS ? r_exp / r_cur : 1.0;

    std::vector<Sim3> initial_guesses;
    initial_guesses.reserve(5);

    // 初始化 1：质心 + 尺度对齐，不旋转
    {
        Sim3 T;
        T.R = Eigen::Matrix3d::Identity();
        T.s = s0;
        T.t = c_exp - T.s * T.R * c_cur;
        initial_guesses.push_back(T);
    }

    // 初始化 2：PCA 主轴对齐，带若干合法翻转
    {
        const Eigen::Matrix3d B_cur = computeCovarianceBasis(current, c_cur);
        const Eigen::Matrix3d B_exp = computeCovarianceBasis(expected, c_exp);

        std::vector<Eigen::Matrix3d> flips;

        Eigen::Matrix3d F;

        F = Eigen::Matrix3d::Identity();
        flips.push_back(F);

        F = Eigen::Matrix3d::Identity();
        F(1, 1) = -1.0;
        F(2, 2) = -1.0;
        flips.push_back(F);

        F = Eigen::Matrix3d::Identity();
        F(0, 0) = -1.0;
        F(2, 2) = -1.0;
        flips.push_back(F);

        F = Eigen::Matrix3d::Identity();
        F(0, 0) = -1.0;
        F(1, 1) = -1.0;
        flips.push_back(F);

        for (const auto& flip : flips) {
            Sim3 T;
            T.R = B_exp * flip * B_cur.transpose();

            if (T.R.determinant() < 0.0) {
                continue;
            }

            T.s = s0;
            T.t = c_exp - T.s * T.R * c_cur;

            initial_guesses.push_back(T);
        }
    }

    double best_error = std::numeric_limits<double>::max();

    for (const auto& init_T : initial_guesses) {
        Sim3 T = init_T;
        double prev_error = std::numeric_limits<double>::max();

        for (int iter = 0; iter < MAX_OUTER_ITER; ++iter) {
            // 1. 固定 R, t, s，求最优一对一匹配
            const std::vector<int> assignment = computeAssignment(T);

            // 2. 固定匹配关系，求最优 R, t, s
            T = estimateTransformIRLS(assignment);

            // 3. 更新匹配并计算当前目标函数
            const std::vector<int> new_assignment = computeAssignment(T);
            const double curr_error =
                computeAssignedSumError(T, new_assignment);

            if (curr_error < best_error) {
                best_error = curr_error;
            }

            if (std::abs(prev_error - curr_error) <
                CONVERGENCE_TOL * std::max(1.0, prev_error)) {
                break;
            }

            prev_error = curr_error;
        }
    }

    if (!std::isfinite(best_error)) {
        return -1.0;
    }

    // 对应公式：
    // e_dist = min sum_i || p_des - (s R p_atu + t) || / L_fma_max
    return best_error / L_fma_max;
}
  // 目前是SWARM_MANUAL_TARGET rviz制定发布goal
  void EGOReplanFSM::setGlobalGoal(ros::NodeHandle &nh)
  {
    // global setting for swarm formation
    if (target_type_ == TARGET_TYPE::SWARM_MANUAL_TARGET ||
        target_type_ == TARGET_TYPE::SWARM_PRESET_TARGET ||
        target_type_ == TARGET_TYPE::GLOBAL_PLANNER_TARGET)
    {
      nh.param("global_goal/swarm_central_pos/x", swarm_central_pos_(0), -1.0);
      nh.param("optimization/formation_type", formation_type, -1);
      nh.param("global_goal/swarm_central_pos/y", swarm_central_pos_(1), -1.0);
      nh.param("global_goal/swarm_central_pos/z", swarm_central_pos_(2), -1.0);
      nh.param("global_goal/swarm_scale", swarm_scale_, -1.0);
      int swarm_size = planner_manager_->ploy_traj_opt_->getFormationSize();
      for (int i = 0; i < swarm_size; i++)
      {
        nh.param("t" + to_string((int)formation_type) + "_relative_pos_" + to_string(i) + "/x", swarm_relative_pts_[i][0], -1.0);
        nh.param("t" + to_string((int)formation_type) + "_relative_pos_" + to_string(i) + "/y", swarm_relative_pts_[i][1], -1.0);
        nh.param("t" + to_string((int)formation_type) + "_relative_pos_" + to_string(i) + "/z", swarm_relative_pts_[i][2], -1.0);
        // nh.param("global_goal/relative_pos_" + to_string(i) + "/x", swarm_relative_pts_[i][0], -1.0);
        // nh.param("global_goal/relative_pos_" + to_string(i) + "/y", swarm_relative_pts_[i][1], -1.0);
        // nh.param("global_goal/relative_pos_" + to_string(i) + "/z", swarm_relative_pts_[i][2], -1.0);
      }
    }

    /* global goal setting

      MANUAL_TARGET: (single drone) get goal in rviz

      PRESET_TARGET: (single/swarm drone) preset goal in launch

      SWARM_MANUAL_TARGET: (swarm formation drone) get formation central goal in rviz,
                           preset relative formation position in launch

      SWARM_PRESET_TARGET: (swarm formation drone) preset formation central goal in launch,
                           preset relative formation position in launch
    */
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);

      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      planGlobalTrajbyGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::SWARM_MANUAL_TARGET)
    {
      swarm_formation_waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::formationWaypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::SWARM_PRESET_TARGET)
    {
      swarm_formation_trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);

      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      planGlobalTrajbyGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::GLOBAL_PLANNER_TARGET)
    {
      broadcast_swarmglobalpath_pub_ = nh.advertise<traj_utils::SwarmGlobalPathList>("planning/broadcast_swarmglobalpath_send", 10);
      broadcast_swarmglobalpath_sub_ = nh.subscribe<traj_utils::SwarmGlobalPathList>("planning/broadcast_swarmglobalpath_recv", 100,
                                                                                     &EGOReplanFSM::RecvBroadcastSwarmGlobalPathCallback,
                                                                                     this,
                                                                                     ros::TransportHints().tcpNoDelay());

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ros::Duration(2.0).sleep();

      // call global path planning for swarm
      double scale = 5.0; // scale of group-level
      if (planner_manager_->pp_.drone_id == 0)
      {
        bool success = planner_manager_->planSwarmGlobalPath(odom_pos_, swarm_central_pos_, scale, swarm_global_path_);
        if (success)
        {
          // publish swarm global path
          traj_utils::SwarmGlobalPathList msg;
          int size = swarm_global_path_.size();
          msg.guard_drone_id = planner_manager_->pp_.drone_id;
          msg.path_num = size;
          msg.swarm_global_path_x.resize(size);
          msg.swarm_global_path_y.resize(size);
          msg.swarm_global_path_z.resize(size);
          for (int i = 0; i < size; i++)
          {
            msg.swarm_global_path_x[i] = swarm_global_path_[i](0);
            msg.swarm_global_path_y[i] = swarm_global_path_[i](1);
            msg.swarm_global_path_z[i] = swarm_global_path_[i](2);
          }
          broadcast_swarmglobalpath_pub_.publish(msg);

          // call global traj planning
          planGlobalTrajbyGivenWps();
        }
      }
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }
  // 已读，planning_horizen_目前trick
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    // show the FSM state
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      fsm_num = 0;
      // printFSMExecState();
    }

    // check the planning_horizen_
    // and slowly increase the planning_horizen_ to the planning_horizen_max_
    static int horizen_check_num = 0;
    horizen_check_num++;
    if (horizen_check_num == 100)
    {
      horizen_check_num = 0;
      double horizen_step = 1.0; // Important: desides the increasing speed.
      if (planning_horizen_ < planning_horizen_max_)
      {
        if (planning_horizen_ + horizen_step > planning_horizen_max_)
        {
          planning_horizen_ = planning_horizen_max_;
        }
        else
        {
          planning_horizen_ += horizen_step;
        }
      }
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
      {
        goto force_return;
      } // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      std::cout << "SEQUENTIAL_STAR!!!!" << std::endl; // sequential start

      // if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      if (planner_manager_->pp_.drone_id != 20 && planner_manager_->pp_.drone_id != 21 && planner_manager_->pp_.drone_id != 22 && planner_manager_->pp_.drone_id != 23 && planner_manager_->pp_.drone_id != 24 && planner_manager_->pp_.drone_id != 25 && planner_manager_->pp_.drone_id != 26 && planner_manager_->pp_.drone_id != 27 && planner_manager_->pp_.drone_id != 28 && planner_manager_->pp_.drone_id != 29 && planner_manager_->pp_.drone_id != 30 && planner_manager_->pp_.drone_id != 31 && planner_manager_->pp_.drone_id != 32 && planner_manager_->pp_.drone_id != 33 && planner_manager_->pp_.drone_id != 34 && planner_manager_->pp_.drone_id != 35 && planner_manager_->pp_.drone_id != 36 && planner_manager_->pp_.drone_id != 37 && planner_manager_->pp_.drone_id != 38 && planner_manager_->pp_.drone_id != 39)
      // if (true)
      {
        if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1))
        {                                       // 起始点位置速度加速度start_pt_，start_vel_，start_acc_ 为当前odom反馈的odom_pos_，odom_vel_，0向量，调用trial_times次callReboundReplan(true, false)返回其返回数值
          bool success = planFromGlobalTraj(1); // zx-todo
          log_ztr2 << "after planFromGlobalTraj" << endl;
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
          }
        }
      }
      else
      {
        if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
        {                                       // 起始点位置速度加速度start_pt_，start_vel_，start_acc_ 为当前odom反馈的odom_pos_，odom_vel_，0向量，调用trial_times次callReboundReplan(true, false)返回其返回数值
          bool success = planFromGlobalTraj(1); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
          }
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      std::cout << "GEN_NEW_TRAJ!!!" << std::endl;
      bool success = planFromGlobalTraj(1); // zx-todo
      log_ztr2 << "in GEN_NEW_TRAJ" << endl;

      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        have_target_ = false;                   // debug
        changeFSMExecState(WAIT_TARGET, "FSM"); // debug
        // changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
        // goto force_return;
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      log_ztr2 << "in REPLAN_TRAJ" << endl;
      std::cout << "REPLAN_TRAJ!!!" << std::endl;
      bool success;
      if (flag_relan_astar_)
        // 设置起始点位置速度加速度start_pt_，start_vel_，start_acc_为当前时刻局部轨迹计算出来的位置速度加速度，调用callReboundReplan返回其数值
        success = planFromLocalTraj(true, false); // use a-star front-end and no formation
      else
        success = planFromLocalTraj(false, true); // use previous front-end with formation

      if (success)
      {
        flag_relan_astar_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_relan_astar_ = true;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);

      if ((local_target_pt_ - end_pt_).norm() < 0.1)
      {
        // local target close to the global target
        have_local_goal_near_target_ = true;
        // uav close to the global target
        if (t_cur > info->duration - 0.2)
        {
          have_target_ = false;
          have_local_traj_ = false;

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
          changeFSMCentralState(INIT_2, "Central FSM");

          // printf("\033[47;30m\n[drone %d reached goal]==============================================\033[0m\n",
          //  planner_manager_->pp_.drone_id);
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      // cout << "[EMERGENCY_STOP] : " << flag_escape_emergency_ << endl;
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }
  // 检查local_traj前2/3段是否与障碍物，与其他飞机距离约束，改变状态机
  // 如果有可能碰撞，调用planFromLocalTraj
  void EGOReplanFSM::rcvOtherOdomCallback(const nav_msgs::Odometry &odom)
  {
    // TODO

    int drone_id_odom = planner_manager_->pp_.drone_id; // default
    std::string numstr = odom.child_frame_id.substr(6);
    std::vector<Eigen::Vector3d> near_odom;
    try
    {
      drone_id_odom = std::stoi(numstr);
      swarm_near[drone_id_odom] = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
    }
    catch (const std::exception &e)
    {
      std::cout << e.what() << '\n';
    }
    for (int j = 0; j < size_swarm; j++)
    {
      if ((odom_pos_ - swarm_near[j]).norm() < vis_hor)
      {
        near_odom.push_back(swarm_near[j]);
      }
    }
    planner_manager_->getneardrone(near_odom);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::Vector3d p_cur = info->traj.getPos(t_cur);
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = planner_manager_->ploy_traj_opt_->getCollisionCheckTimeEnd();
    double t_temp; // when may occur collision
    bool occ = false;
    for (double t = t_cur; t < info->duration; t += time_step)
    {
      // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      if (map->getInflateOccupancy(info->traj.getPos(t)) == 1)
      {
        ROS_WARN("drone %d is too close to the obstacle at relative time %f!",
                 planner_manager_->pp_.drone_id, t / info->duration);
        t_temp = t;
        occ = true;
        break;
      }
      // Enumerate uav, check safe distance from other uav
      for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
      { // find a uav whose ID is exactly id and the uav can't be it self
        if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
            (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->traj_.swarm_traj.at(id).start_time;

        if (t_X > planner_manager_->traj_.swarm_traj.at(id).duration)
          continue;

        Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          // ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
          //          planner_manager_->pp_.drone_id, id, dist);
          t_temp = t;
          occ = true;
          break;
        }
      }
    }

    if (occ)
    {
      /* Handle the collided case immediately */
      // ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(true, false)) // Make a chance
      // if (planFromLocalTraj(false, true))
      {
        // ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }
      else
      {
        if (t_temp - t_cur < emergency_time_) // 1.0s of emergency time
        {
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          ROS_WARN("current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }
  Eigen::VectorXi EGOReplanFSM::assignment_pub()
  {
    vector<Eigen::Vector3d> nodes_des;
    vector<Eigen::Vector3d> nodes_cur;
    Eigen::VectorXi assignment_pub_;
    assignment_pub_.resize(num_now_);
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec();
    Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.6));

    Eigen::Vector4d trajP = planner_manager_->ploy_traj_opt_->trajROpt.getPwithR(min(info->duration, t_cur - info->start_time + 0.6));
    Eigen::Vector3d start_pos_leader = trajP.head(3);

    for (int i = 0; i < swarm_des_set_.size(); i++)
    {
      if (swarm_des_set_[i][2] != -99.0)
      {
        nodes_des.push_back(start_pos_leader + swarm_des_set_[i]);
        nodes_cur.push_back(assig_pos_[i]);
      }
    }

    // assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
    planner_manager_->callAssign(nodes_cur, nodes_des, assignment_pub_);

    return assignment_pub_;
  }
  vector<Eigen::Vector3d> EGOReplanFSM::assignment_pos_get(const Eigen::VectorXi &assignment_pub_get)
  {
    vector<Eigen::Vector3d> assignment_pos(swarm_des_set_.size());
    int num_out = 0;
    int num_in = 0;

    for (int i = 0, fake_id = 0; i < swarm_des_set_.size(); i++)
    {
      int flag_j = 0;

      if (swarm_des_set_[i][2] == -99.0)
      {
        assignment_pos[i] = swarm_des_set_[i];
        fake_id++;
      }
      else
      {
        while (num_in <= assignment_pub_get[i - fake_id])
        {
          if (swarm_des_set_[flag_j][2] != -99.0)
          {
            num_in++;
          }
          else
          {
            num_out++;
          }
          flag_j++;
        }
        assignment_pos[i] = swarm_des_set_[assignment_pub_get[i - fake_id] + num_out];
      }
      num_out = 0;
      num_in = 0;
    }
    return assignment_pos;
  }
  bool EGOReplanFSM::stopCallback()
  {
    if (planner_manager_->traj_.swarm_traj.size() < size_swarm)
    {
      return false;
    }

    /// clc_des_swarm
    double future_time = ros::Time::now().toSec() - planner_manager_->ploy_traj_opt_->leader_opt_time + 0.1;
    std::vector<Eigen::Vector3d> swarm_cur_set(size_swarm);
    for (int i = 0; i < size_swarm; i++)
    {
      if (i == 0)
      {
        swarm_cur_set[i] = odom_pos_;
      }
      else
      {
        // cout << "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii!!!!!!!::::" << i << endl;
        double future_times = ros::Time::now().toSec() - planner_manager_->traj_.swarm_traj[i].start_time + 0.1;
        // cout << "ros::Time::now().toSec()!!!!!!!!!!::::" << ros::Time::now().toSec() << endl;
        // cout << "planner_manager_->traj_.swarm_traj[i].start_time!!!!!!!!::::" << planner_manager_->traj_.swarm_traj[i].start_time << endl;
        // cout << "future_times!!!!!!::::" << future_times << endl;
        // cout << "planner_manager_->traj_.swarm_traj[i].traj.getTotalDuration()!::::" << planner_manager_->traj_.swarm_traj[i].traj.getTotalDuration() << endl;
        double t_future = std::min(std::max(future_times, 0.0), planner_manager_->traj_.swarm_traj[i].traj.getTotalDuration());
        swarm_cur_set[i] = planner_manager_->traj_.swarm_traj[i].traj.getPos(t_future);
      }
    }
    double clc_swarm = planner_manager_->ploy_traj_opt_->clc_des_swarm(swarm_cur_set, swarm_des_id, future_time);
    if (clc_swarm >= 5.0)
    {
      planner_manager_->ploy_traj_opt_->set_max_vel(false);
      t_stop = ros::Time::now();
      return true;
    }
    else
    {
      planner_manager_->ploy_traj_opt_->set_max_vel(true);
    }
    return false;
  }
  void EGOReplanFSM::errorCallback(const ros::TimerEvent &e)
  {
    if(exec_state_ == EXEC_TRAJ && planner_manager_->traj_.swarm_traj.size()>=desired_points.size())
    {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec();
    Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.05));
    vector<Eigen::Vector3d> assig_pos;
    assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
    ros::Time t1 = ros::Time::now();
    double error_for_rebuttal =  matchAndComputeErrorWithScale(desired_points, assig_pos);
    ros::Time t2 = ros::Time::now();
    // std::cout<<"#################################################################################################################################################icp::"<<(t2-t1).toSec()<<std::endl;
    if(formation_change_)
    {
      formation_change_ = false;
      log_zy << 5.0<<endl;
    }
    else 
    {
    log_zy << error_for_rebuttal<<endl;
    }
    }
    return;

  }
  void EGOReplanFSM::formationCallback(const ros::TimerEvent &e)
  {
    time_opt_now = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> concave_poly; //
    ////////////////yuan///////////////
    // concave_poly = {{1.0, 0.0, 0.0},
    //                 {0.81, 0.59, 0.0},
    //                 {0.31, 0.95, 0.0},
    //                 {-0.31, 0.95, 0.0},
    //                 {-0.81, 0.59, 0.0},
    //                 {-1.0, 0.0, 0.0},
    //                 {-0.81, -0.59, 0.0},
    //                 {-0.31, -0.95, 0.0},
    //                 {0.31, -0.95, 0.0},
    //                 {0.81, -0.59, 0.0},
    //                 {1.0, 0.0, 0.0}};
    ////////////////yuan///////////////
    concave_poly = {{0.00, 0.40, 0},
                    {0.69, 1.18, 0},
                    {1.32, 1.10, 0},
                    {1.76, 0.3, 0},
                    {1.43, -0.65, 0},
                    {0.80, -1.50, 0},
                    {0.00, -1.92, 0},
                    {-0.80, -1.50, 0},
                    {-1.43, -0.65, 0},
                    {-1.76, 0.3, 0},
                    {-1.32, 1.10, 0},
                    {-0.69, 1.18, 0}};
    ////////////////yuan///////////////
    // concave_poly = {{1.0, 1.0, 0.0},
    //                 {-1.0, 1.0, 0.0},
    //                 {-1.0, -1.0, 0.0},
    //                 {1.0, -1.0, 0.0},
    //                 {1.0, 1.0, 0.0}};

    // Eigen::Vector3d center_poly = findMinEnclosingCircle(concave_poly);
    // for (int i_poly = 0; i_poly < concave_poly.size(); i_poly++)
    // {
    //   concave_poly[i_poly] = concave_poly[i_poly] - center_poly;
    // }
    const int iterations = 20;
    const int monter_num = 520;
    double dist_change = -5.0;
    /*********************************************************************TODO TODO********************************************************************* */
    
    {
      if ((odom_pos_[0] + 32.5) > dist_change && form_change == 0)
      {
        formation_change_ = true;
        N_POINTS = 24;
        swarm_des_id[20] = 20;
        swarm_des_id[21] = 21;
        swarm_des_id[22] = 22;
        swarm_des_id[23] = 23;
        assig_pos_[20] = Eigen::Vector3d(-33.0, -1.5, 1.0);
        assig_pos_[21] = Eigen::Vector3d(-31.5, -1.5, 1.0);
        assig_pos_[22] = Eigen::Vector3d(-33.0, 4.5, 1.0);
        assig_pos_[23] = Eigen::Vector3d(-31.5, 4.5, 1.0);
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = ros::Time::now().toSec();
        Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.05));
        vector<Eigen::Vector3d> assig_pos;
        assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
        for (int i = 0, fake_id = 0; i < swarm_des_set_.size(); i++)
        {
          if (swarm_des_set_[i][2] != -99.0)
          {
            assig_pos_[i] = assig_pos[fake_id];
            fake_id++;
          }
        }

        // Generate initial samples
        // 初始化样本点
        std::vector<Eigen::Vector3d> points = initialSamples3D(concave_poly, N_POINTS);
        // std::vector<Eigen::Vector3d> points = generateGridCenters(concave_poly, N_POINTS, 0.1);
        // 执行迭代
        std::vector<Eigen::Vector3d> samples = generate_3d_grid_samples(concave_poly, monter_num);
        for (int i = 0; i < iterations; ++i)
        {
          // points = lloydIteration3D(concave_poly, points, monter_num);
          points = lloyd_iteration_3d(samples, points);
        }
        std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
        std::vector<int> no_id_in{24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
        for (int ii = 0, fake_id = 0, no_id = 0; ii < size_swarm; ii++)
        {
          if (ii == no_id_in[no_id])
          {
            swarm_des_set[ii] = Eigen::Vector3d(0, 0, -99);
            no_id++;
          }
          else
          {
            swarm_des_set[ii] = points[fake_id];
            fake_id++;
          }
        }
        swarm_des_set_ = swarm_des_set;
        num_now_ = 24;
        Eigen::VectorXi assig_new = assignment_pub();
        assig_id = assig_new;
        vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);
        planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
        agent_add_ = true;
        leader_pos = assig_id_new[0];

        double min_dis_in = findMinDistance(points);
        scale_suit_size = 1.0 * (1.5 / min_dis_in);
        trajP_now[3] = scale_suit_size;
        form_change = 1;
        desired_points = points;
        planner_manager_->ploy_traj_opt_->change_id(planner_manager_->pp_.drone_id, 0);
      }

      if ((odom_pos_[0] + 22.0) > dist_change && form_change == 1)
      {
        formation_change_ = true;
        N_POINTS = 30;
        assig_pos_[24] = Eigen::Vector3d(-19.0, -2.5, 1.0);
        assig_pos_[25] = Eigen::Vector3d(-19.0, 1.5, 1.0);
        assig_pos_[26] = Eigen::Vector3d(-17.5, 1.5, 1.0);
        assig_pos_[27] = Eigen::Vector3d(-19.0, 3.0, 1.0);
        assig_pos_[28] = Eigen::Vector3d(-17.5, 3.0, 1.0);
        assig_pos_[29] = Eigen::Vector3d(-16.0, -3.0, 1.0);
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = ros::Time::now().toSec();
        Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.05));
        vector<Eigen::Vector3d> assig_pos;
        assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
        for (int i = 0, fake_id = 0; i < swarm_des_set_.size(); i++)
        {
          if (swarm_des_set_[i][2] != -99.0)
          {
            assig_pos_[i] = assig_pos[fake_id];
            fake_id++;
          }
        }
        // Generate initial samples
        // 初始化样本点
        std::vector<Eigen::Vector3d> points = initialSamples3D(concave_poly, N_POINTS);
        // std::vector<Eigen::Vector3d> points = generateGridCenters(concave_poly, N_POINTS, 0.1);

        // 执行迭代
        double tttt1 = ros::Time::now().toSec();
        std::vector<Eigen::Vector3d> samples = generate_3d_grid_samples(concave_poly, monter_num);
        double tttt2 = ros::Time::now().toSec();
        cout << "ttttttttttttttttttttttttttttttttttTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT:::" << tttt2 - tttt1 << endl;
        for (int i = 0; i < iterations; ++i)
        {
          // points = lloydIteration3D(concave_poly, points, monter_num);
          points = lloyd_iteration_3d(samples, points);
        }
        double tttt3 = ros::Time::now().toSec();
        cout << "ttttttttttttttttttttttttttttttttttTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT222:::" << tttt3 - tttt2 << endl;
        std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
        std::vector<int> no_id_in{30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
        for (int ii = 0, fake_id = 0, no_id = 0; ii < size_swarm; ii++)
        {
          if (ii == no_id_in[no_id])
          {
            swarm_des_set[ii] = Eigen::Vector3d(0, 0, -99);
            no_id++;
          }
          else
          {
            swarm_des_set[ii] = points[fake_id];
            fake_id++;
          }
        }
        swarm_des_set_ = swarm_des_set;
        num_now_ = 30;
        Eigen::VectorXi assig_new = assignment_pub();
        assig_id = assig_new;
        vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);
        planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
        agent_add_ = true;
        leader_pos = assig_id_new[0];

        double min_dis_in = findMinDistance(points);
        scale_suit_size = 1.0 * (1.5 / min_dis_in);

        trajP_now[3] = scale_suit_size;
        form_change = 2;
        planner_manager_->ploy_traj_opt_->change_id(planner_manager_->pp_.drone_id, 1);
        desired_points = points;
      }

      /******************************************************************************************************************************************************************** */
      if ((odom_pos_[0] + 10.0) > 0.0 && form_change == 2)
      {
        formation_change_ = true;
        N_POINTS = 27;
        swarm_des_id[3] = -99;
        swarm_des_id[4] = -99;
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = ros::Time::now().toSec();
        Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.05));
        vector<Eigen::Vector3d> assig_pos;
        assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
        for (int i = 0, fake_id = 0; i < swarm_des_set_.size(); i++)
        {
          if (swarm_des_set_[i][2] != -99.0)
          {
            assig_pos_[i] = assig_pos[fake_id];
            fake_id++;
          }
        }

        // Generate initial samples
        // 初始化样本点
        std::vector<Eigen::Vector3d> points = initialSamples3D(concave_poly, N_POINTS);
        // std::vector<Eigen::Vector3d> points = generateGridCenters(concave_poly, N_POINTS, 0.1);
        // 执行迭代
        std::vector<Eigen::Vector3d> samples = generate_3d_grid_samples(concave_poly, monter_num);
        for (int i = 0; i < iterations; ++i)
        {
          // points = lloydIteration3D(concave_poly, points, monter_num);
          points = lloyd_iteration_3d(samples, points);
        }
        std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
        std::vector<int> no_id_in{5, 6, 7};
        // std::vector<int> no_id_in{5, 6, 7, 8, 9, 16, 17};
        for (int ii = 0, fake_id = 0, no_id = 0; ii < size_swarm; ii++)
        {
          if (ii == no_id_in[no_id])
          {
            swarm_des_set[ii] = Eigen::Vector3d(0, 0, -99);
            no_id++;
          }
          else
          {
            swarm_des_set[ii] = points[fake_id];
            fake_id++;
          }
        }
        swarm_des_set_ = swarm_des_set;

        num_now_ = 27;
        Eigen::VectorXi assig_new = assignment_pub();
        assig_id = assig_new;
        vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);
        planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
        agent_add_ = false;
        leader_pos = assig_id_new[0];

        double min_dis_in = findMinDistance(points);
        scale_suit_size = 1.0 * (1.5 / min_dis_in);
        trajP_now[3] = scale_suit_size;
        form_change = 3;
        planner_manager_->ploy_traj_opt_->change_id(planner_manager_->pp_.drone_id, 2);
        desired_points = points;
      }

      if ((odom_pos_[0] + 0.0) > 0.0 && form_change == 3)
      {
        formation_change_ = true;
        N_POINTS = 23;
        swarm_des_id[3] = -99;
        swarm_des_id[4] = -99;
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = ros::Time::now().toSec();
        Eigen::Vector3d pos = info->traj.getPos(min(info->duration, t_cur - info->start_time + 0.05));
        vector<Eigen::Vector3d> assig_pos;
        assig_pos = planner_manager_->getCurrentPos_all(t_cur, pos, swarm_des_set_);
        for (int i = 0, fake_id = 0; i < swarm_des_set_.size(); i++)
        {
          if (swarm_des_set_[i][2] != -99.0)
          {
            assig_pos_[i] = assig_pos[fake_id];
            fake_id++;
          }
        }

        // Generate initial samples
        // 初始化样本点
        std::vector<Eigen::Vector3d> points = initialSamples3D(concave_poly, N_POINTS);
        // std::vector<Eigen::Vector3d> points = generateGridCenters(concave_poly, N_POINTS, 0.1);
        // 执行迭代
        std::vector<Eigen::Vector3d> samples = generate_3d_grid_samples(concave_poly, monter_num);
        for (int i = 0; i < iterations; ++i)
        {
          // points = lloydIteration3D(concave_poly, points, monter_num);
          points = lloyd_iteration_3d(samples, points);
        }
        std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
        std::vector<int> no_id_in{5, 6, 7, 8, 9, 10, 11};
        // std::vector<int> no_id_in{5, 6, 7, 8, 9, 16, 17};
        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@::" << endl;
        for (int ii = 0, fake_id = 0, no_id = 0; ii < size_swarm; ii++)
        {
          if (ii == no_id_in[no_id])
          {
            swarm_des_set[ii] = Eigen::Vector3d(0, 0, -99);
            no_id++;
          }
          else
          {
            swarm_des_set[ii] = points[fake_id];
            fake_id++;
          }

          cout << "[" << swarm_des_set[ii][0] << "," << swarm_des_set[ii][1] << "," << swarm_des_set[ii][2] << "]," << endl;
          cout << endl;
        }
        swarm_des_set_ = swarm_des_set;

        num_now_ = 23;
        Eigen::VectorXi assig_new = assignment_pub();
        assig_id = assig_new;
        vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);

        planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
        agent_add_ = false;
        leader_pos = assig_id_new[0];

        double min_dis_in = findMinDistance(points);
        scale_suit_size = 1.0 * (1.5 / min_dis_in);
        trajP_now[3] = scale_suit_size;
        form_change = 4;
        planner_manager_->ploy_traj_opt_->change_id(planner_manager_->pp_.drone_id, 3);
        desired_points = points;
      }

    }
  

    /*********************************************************************TODO TODO********************************************************************* */
    if (exec_state_ == REPLAN_TRAJ || exec_state_ == EXEC_TRAJ)
    {
      Eigen::Matrix3d start_leader_;
      Eigen::Matrix3d end_leader_;
      vector<double> rrr_start_;
      vector<double> rrr_end_;
      vector<double> lll_start_;
      vector<double> lll_end_;
      start_leader_.setZero();
      end_leader_.setZero();
      // end_leader_.col(0) = local_target_pt_;
      // end_leader_.col(1) = local_target_vel_;

      double future_time = 8.5;
      // if ((stopCallback() && traj_lll_now > 0)||(ros::Time::now()-t_stop).toSec()<=1.0)
      if (false)
      {
        start_leader_.col(0) = odom_pos_ - leader_pos;
        start_leader_.col(1) = odom_vel_ * 0.0;
        start_leader_.col(2) = start_acc_ * 0.0;
        // end_leader_.col(0) = Eigen::Vector3d(10.0, 4.0, 1.0);
        end_leader_.col(0) = local_target_pt_;
        end_leader_.col(1) = odom_vel_ * 0.0;
        rrr_start_.resize(3);
        rrr_end_.resize(3);
        lll_start_.resize(3);
        lll_end_.resize(3);
        rrr_start_[0] = trajP_now[3];
        rrr_end_[0] = scale_suit_size;
        lll_start_[0] = traj_lll_now;
        lll_end_[0] = 1.0;
      }
      else
      {
        if (flag_have_pathr)
        {
          double leader_replan_time = ros::Time::now().toSec() - planner_manager_->ploy_traj_opt_->leader_opt_time + 0.1;
          Eigen::Vector3d trajV;
          Eigen::Vector3d trajA;
          Eigen::Vector3d trajP;
          trajP = planner_manager_->ploy_traj_opt_->trajROpt.getPos_(leader_replan_time);

          trajV = planner_manager_->ploy_traj_opt_->trajROpt.getVel_(leader_replan_time);
          trajA = planner_manager_->ploy_traj_opt_->trajROpt.getAcc_(leader_replan_time);
          double tt_add = 0.0;
          while (trajV.norm() <= 0.6 && form_change >= 3)
          {
            tt_add = tt_add + 0.1;
            leader_replan_time = leader_replan_time + 0.1;
            trajV = planner_manager_->ploy_traj_opt_->trajROpt.getVel_(leader_replan_time);
            trajA = planner_manager_->ploy_traj_opt_->trajROpt.getAcc_(leader_replan_time);
          }

          start_leader_.col(0) = odom_pos_ - leader_pos + Eigen::Vector3d(0.0, trajV[1] * tt_add, 0.0);
          // start_leader_.col(0) = trajP;
          start_leader_.col(1) = trajV;
          start_leader_.col(2) = trajA;
          // end_leader_.col(0) = Eigen::Vector3d(10.0, 4.0, 1.0);
          end_leader_.col(0) = local_target_pt_;
          end_leader_.col(1) = local_target_vel_;
          rrr_start_.resize(3);
          rrr_end_.resize(3);
          lll_start_.resize(3);
          lll_end_.resize(3);
          rrr_start_[0] = trajP_now[3];
          rrr_end_[0] = scale_suit_size;
          lll_start_[0] = traj_lll_now;
          lll_end_[0] = 1.0;
        }
        else
        {
          start_leader_.col(0) = odom_pos_ - leader_pos;
          start_leader_.col(1) = odom_vel_;
          start_leader_.col(2) = odom_acc_;
          // end_leader_.col(0) = Eigen::Vector3d(10.0, 4.0, 1.0);
          end_leader_.col(0) = local_target_pt_;
          end_leader_.col(1) = local_target_vel_;
          rrr_start_.resize(3);
          rrr_end_.resize(3);
          lll_start_.resize(3);
          lll_end_.resize(3);
          rrr_start_[0] = 1.5;
          rrr_end_[0] = scale_suit_size;
          lll_start_[0] = 1.0;
          lll_end_[0] = 1.0;
        }
      }

      bool flag_opt_formation = opt_formation(start_leader_, end_leader_, rrr_start_, rrr_end_, lll_start_, lll_end_);
    }

    return;
  }
  void EGOReplanFSM::centralFSMCallback(const ros::TimerEvent &e)
  {
    centra_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 20)
    {
      fsm_num = 0;
      printFSMCentralState();
    }

    switch (central_state_)
    {
    case INIT_2:
    {
      if (!have_odom_ || !have_target_)
      {
        goto force_return; // return;
      }
      changeFSMCentralState(CHECK, "Central FSM");
      break;
    }

    case CHECK:
    {
      // for the first time
      static int wait_num = 0;
      if (wait_num < 20)
      {
        wait_num++;
        goto force_return;
      }
      wait_num = 0;

      if (have_local_goal_near_target_)
        goto force_return;

      if (callCARemapCheck())
      { // 依靠constraints程度判断是否需要need_remap，其中softmaxLayer尚未仔细看
        changeFSMCentralState(CA_REMAP_ASSIGNMENT, "Central FSM");
      } // 利用odom信息与其他飞机位置检查当前formation_error考虑是否需要need_remap。formation_error考虑使用其他飞机的里程计的信息更加可靠???
      else if (callSERemapCheck())
      {
        changeFSMCentralState(SE_REMAP_ASSIGNMENT, "Central FSM");
      }

      goto force_return;
      break;
    }

    case CA_REMAP_ASSIGNMENT: // contraints
    {                         // 依靠nodes_horizon_pos即3.5s后集群uav的位置，现在的uav位置即nodes_cur，aware_weight重新得到remap_lg_pos与assignment其中callAlignAssign函数尚未仔细看
      bool success = callCALocalGoalRemap(remap_lg_pos_, remap_lg_vel_, assignment_);
      if (success)
      {
        // publish remap result
        traj_utils::RemapLocalGoalList msg;
        remapLocalGoalList2ROSMsg(msg);
        broadcast_remaplocalgoal_pub_.publish(msg);
        changeFSMCentralState(REPLAN_GLOBAL_TRAJ, "Central FSM");
        // update the visualization_
        // visualization_->updateAssignment(assignment_);
        // visualization_->displayRemapSwarmLocal(remap_lg_pos_);
      }
      break;
    }

    case SE_REMAP_ASSIGNMENT: // similarity
    {
      bool success = callSELocalGoalRemap(remap_lg_pos_, remap_lg_vel_, assignment_);
      if (success)
      {
        // publish remap result
        traj_utils::RemapLocalGoalList msg;
        remapLocalGoalList2ROSMsg(msg);
        broadcast_remaplocalgoal_pub_.publish(msg);
        changeFSMCentralState(REPLAN_GLOBAL_TRAJ, "Central FSM");
        // update the visualization_
        // visualization_->updateAssignment(assignment_);
        // visualization_->displayRemapSwarmLocal(remap_lg_pos_);
      }
      break;
    }

    case REPLAN_GLOBAL_TRAJ:
    {
      bool success = callGlobalTrajReplan();
      if (success)
      {
        changeFSMCentralState(EXEC_ASSIGNMENT, "Central FSM");
      }
      break;
    }

    case EXEC_ASSIGNMENT:
    {
      static int wait_num = 0;
      if (wait_num < 60)
      {
        wait_num++;
        goto force_return;
      }
      wait_num = 0;
      changeFSMCentralState(CHECK, "Central FSM");
      break;
    }

    default:
      break;
    }

  force_return:;
    centra_timer_.start();
  }
  // 暂时不用看，因为目前是SWARM_MANUAL_TARGET rviz制定发布goal，和target_type_有关
  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    // cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }
  // 暂时不用看，因为目前是SWARM_MANUAL_TARGET rviz制定发布goal，和target_type_有关
  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {

    if (msg->pose.position.z < -0.1)
      return;

    // cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->pose.position.x, msg->pose.position.y, 1.0;

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // success = planner_manager_->planGlobalTrajWaypoints(
    //     odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
    //     one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      have_local_goal_near_target_ = false;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }
  // 制定swarm目标点,检查状态机
  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    // cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    bool success = false;
    // swarm_central_pos_(0) = msg->pose.position.x;
    // swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(0) = 55.0;
    swarm_central_pos_(1) = 0.0;
    swarm_central_pos_(2) = 1.0;
    //
    int id = assignment_(planner_manager_->pp_.drone_id);

    Eigen::Vector3d relative_pos;
    relative_pos << swarm_relative_pts_[id][0],
        swarm_relative_pts_[id][1],
        swarm_relative_pts_[id][2];
    end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;
    // log_ztr2<<"endpt"<<end_pt_.transpose()<<endl;

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // success = planner_manager_->planGlobalTrajWaypoints(
    //     odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
    //     one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);

      /*** display ***/ // 0.1时间分辨率可视化轨迹
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      have_local_goal_near_target_ = false;

      /*** FSM ***/
      if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // debug
      if (central_state_ == EXEC_ASSIGNMENT)
        changeFSMCentralState(CHECK, "Central FSM");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  // ok
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // this is worked, but still need to improve for more robust
    // debug
    if (target_type_ == TARGET_TYPE::GLOBAL_PLANNER_TARGET)
    {
      if (swarm_global_path_.size() > 0 && (odom_pos_ - swarm_global_path_.front()).norm() < 5.0)
        swarm_global_path_.erase(swarm_global_path_.begin(), swarm_global_path_.begin() + 1);
    }

    have_odom_ = true;
  }

  // 接受广播的轨迹，按照id号存入缓存区planner_manager_->traj_.swarm_traj  traj_utils::pathR_lll
  void EGOReplanFSM::pathR_TrajCallback(const traj_utils::pathR_lllConstPtr &msg)
  {
    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_r.size())
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
               msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return; // 不接受自己的轨迹
    planner_manager_->ploy_traj_opt_->use_leader_ = msg->use_leader;

    Eigen::VectorXi assig_new(msg->ass_num);
    for (size_t i = 0; i < msg->ass_num; ++i)
    {
      assig_new[i] = msg->assig_id[i];
    }

    if (msg->change_front)
    {
      planner_manager_->ploy_traj_opt_->number_change = true;
    }
    if (msg->agent_add)
    {
      std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
      for (int i = 0; i < size_swarm; i++)
      {
        swarm_des_set[i] = Eigen::Vector3d(msg->assi_x_pos[i], msg->assi_y_pos[i], msg->assi_z_pos[i]);
      }
      swarm_des_set_ = swarm_des_set;
      vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);
      planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
      have_target_ = true;
    }
    if (!msg->agent_add)
    {
      std::vector<Eigen::Vector3d> swarm_des_set(size_swarm);
      for (int i = 0; i < size_swarm; i++)
      {
        swarm_des_set[i] = Eigen::Vector3d(msg->assi_x_pos[i], msg->assi_y_pos[i], msg->assi_z_pos[i]);
        if (msg->assi_z_pos[i] == -99.0 && planner_manager_->pp_.drone_id == i)
        {
          changeFSMExecState(WAIT_TARGET, "FSM");
          flag_escape_emergency_ = true;
          have_target_ = false;
          callEmergencyStop(odom_pos_);
        }
      }
      swarm_des_set_ = swarm_des_set;
      vector<Eigen::Vector3d> assig_id_new = assignment_pos_get(assig_new);

      planner_manager_->ploy_traj_opt_->set_des_swarm(assig_id_new);
    }

    planner_manager_->ploy_traj_opt_->leader_opt_time = msg->start_time.toSec();
    planner_manager_->ploy_traj_opt_->front_traj_piecenum = msg->traj_piecenum;

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<Eigen::Matrix<double, 3, 6>> cMats_pos(piece_nums);
    std::vector<Eigen::Matrix<double, 1, 6>> cMats_r(piece_nums);
    std::vector<Eigen::Matrix<double, 1, 6>> cMats_l(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats_pos[i].row(0) << msg->coef_x_pos[i6 + 0], msg->coef_x_pos[i6 + 1], msg->coef_x_pos[i6 + 2],
          msg->coef_x_pos[i6 + 3], msg->coef_x_pos[i6 + 4], msg->coef_x_pos[i6 + 5];
      cMats_pos[i].row(1) << msg->coef_y_pos[i6 + 0], msg->coef_y_pos[i6 + 1], msg->coef_y_pos[i6 + 2],
          msg->coef_y_pos[i6 + 3], msg->coef_y_pos[i6 + 4], msg->coef_y_pos[i6 + 5];
      cMats_pos[i].row(2) << msg->coef_z_pos[i6 + 0], msg->coef_z_pos[i6 + 1], msg->coef_z_pos[i6 + 2],
          msg->coef_z_pos[i6 + 3], msg->coef_z_pos[i6 + 4], msg->coef_z_pos[i6 + 5];

      cMats_r[i] << msg->coef_r[i6 + 0], msg->coef_r[i6 + 1], msg->coef_r[i6 + 2],
          msg->coef_r[i6 + 3], msg->coef_r[i6 + 4], msg->coef_r[i6 + 5];
      cMats_l[i] << msg->coef_l[i6 + 0], msg->coef_l[i6 + 1], msg->coef_l[i6 + 2],
          msg->coef_l[i6 + 3], msg->coef_l[i6 + 4], msg->coef_l[i6 + 5];

      dura[i] = msg->duration[i];
    }
    poly_traj_leader::Trajectory<3> posTraj_sub_(dura, cMats_pos);
    poly_traj_leader::Trajectory<1> radiusTraj_sub_(dura, cMats_r);
    poly_traj_leader::Trajectory<1> lllTraj_sub_(dura, cMats_l);

    planner_manager_->ploy_traj_opt_->posTraj_sub = posTraj_sub_;
    planner_manager_->ploy_traj_opt_->radiusTraj_sub = radiusTraj_sub_;
    planner_manager_->ploy_traj_opt_->lllTraj_sub = lllTraj_sub_;
    planner_manager_->ploy_traj_opt_->opt_ok = true;
  }
  void EGOReplanFSM::add_out_Callback(const traj_utils::add_outConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return; // 不接受自己

    /********************************************TODO********************************************************** */
  }
  void EGOReplanFSM::RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
  {
    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
               msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return; // 不接受自己的轨迹

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1; // 表示还没接收到轨迹
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */ // swarm_traj中根据飞机id存储其轨迹
    // if(formation_change_fsm)
    // {
    //   formation_change_fsm = false;
    //   planner_manager_->form_change_get(formation_change_fsm);
    // }
    bool flag_id = true;
    for (int i = 0; i < group_id_[planner_manager_->pp_.drone_id].size(); i++)
    {
      if ((int)recv_id == group_id_[planner_manager_->pp_.drone_id][i])
      {
        flag_id = false;
      }
    }

    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
    planner_manager_->traj_.swarm_traj[recv_id].constraint_aware = msg->constraint_aware;
    planner_manager_->traj_.swarm_traj[recv_id].vis_aware = msg->vis_aware;

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    poly_traj::Trajectory trajectory(dura, cMats);
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    // TODO

    Eigen::Vector3d odom_near_from_traj;
    odom_near_from_traj[0] = trajectory.getPos(abs((ros::Time::now() - msg->start_time).toSec()))[0];
    odom_near_from_traj[1] = trajectory.getPos(abs((ros::Time::now() - msg->start_time).toSec()))[1];
    odom_near_from_traj[2] = trajectory.getPos(abs((ros::Time::now() - msg->start_time).toSec()))[2];
    std::vector<Eigen::Vector3d> near_odom;
    swarm_near[recv_id] = odom_near_from_traj;
    for (int j = 0; j < size_swarm; j++)
    {
      if ((odom_pos_ - swarm_near[j]).norm() < vis_hor)
      {
        near_odom.push_back(swarm_near[j]);
      }
    }
    planner_manager_->getneardrone(near_odom);
    assig_pos_[recv_id] = planner_manager_->traj_.swarm_traj[recv_id].traj.getPos((ros::Time::now() - msg->start_time).toSec());
    /* Check Collision */
    // if (planner_manager_->checkCollision(recv_id))
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    // }

    /* Check if receive agents have lower drone id */
    if (trajectory.getPos(0.0)[0] > x_dist)
    {
      x_dist = trajectory.getPos(0.0)[0];
    }
    // if (!have_recv_pre_agent_)
    // {
    //   if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
    //   {
    //     for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
    //     {
    //       if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
    //       {
    //         break;
    //       }
    //       if (abs(x_dist - odom_pos_[0]) <= 1.0)
    //       {
    //         have_recv_pre_agent_ = true;
    //       }
    //     }
    //   }
    // }
    if (planner_manager_->pp_.drone_id == 20 || planner_manager_->pp_.drone_id == 21 || planner_manager_->pp_.drone_id == 22 || planner_manager_->pp_.drone_id == 23 || planner_manager_->pp_.drone_id == 24 || planner_manager_->pp_.drone_id == 25 || planner_manager_->pp_.drone_id == 26 || planner_manager_->pp_.drone_id == 27 || planner_manager_->pp_.drone_id == 28 || planner_manager_->pp_.drone_id == 29 || planner_manager_->pp_.drone_id == 30 || planner_manager_->pp_.drone_id == 31 || planner_manager_->pp_.drone_id == 32 || planner_manager_->pp_.drone_id == 33 || planner_manager_->pp_.drone_id == 34 || planner_manager_->pp_.drone_id == 35 || planner_manager_->pp_.drone_id == 36 || planner_manager_->pp_.drone_id == 37 || planner_manager_->pp_.drone_id == 38 || planner_manager_->pp_.drone_id == 39)
    {
      if (!have_recv_pre_agent_)
      {
        if (abs(x_dist - odom_pos_[0]) <= 5.0)
        {
          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  // 接受广播的LocalGoal，按照id号存入缓存区planner_manager_->traj_.swarm_local_goal
  // 事实上只有drone_id为0的才有这个消息，仔细想想是否为bug???
  void EGOReplanFSM::RecvBroadcastLocalGoalCallback(const traj_utils::LocalGoalConstPtr &msg)
  {
    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return; // the message from own,no need

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_local_goal.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_local_goal.size(); i <= recv_id; i++)
      {
        LocalGoalData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_local_goal.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_local_goal[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_local_goal[recv_id].global_traj_id = msg->global_traj_id;
    planner_manager_->traj_.swarm_local_goal[recv_id].lg_pos << msg->lg_pos_x,
        msg->lg_pos_y,
        msg->lg_pos_z;
    planner_manager_->traj_.swarm_local_goal[recv_id].lg_vel << msg->lg_vel_x,
        msg->lg_vel_y,
        msg->lg_vel_z;
  }

  // 尚未看
  void EGOReplanFSM::RecvBroadcastRemapLocalGoalCallback(const traj_utils::RemapLocalGoalListConstPtr &msg)
  {
    const size_t guard_drone_id = (size_t)msg->guard_drone_id;
    if ((int)guard_drone_id == planner_manager_->pp_.drone_id)
      return;

    int size = msg->assignment.size();
    if (size != planner_manager_->ploy_traj_opt_->getFormationSize())
      return;

    /* Store data */
    remap_lg_pos_.resize(size);
    remap_lg_vel_.resize(size);
    assignment_.resize(size);

    for (int i = 0; i < size; i++)
    {
      remap_lg_pos_[i] = Eigen::Vector3d(msg->remap_lg_pos_x[i],
                                         msg->remap_lg_pos_y[i],
                                         msg->remap_lg_pos_z[i]);
      remap_lg_vel_[i] = Eigen::Vector3d(msg->remap_lg_vel_x[i],
                                         msg->remap_lg_vel_y[i],
                                         msg->remap_lg_vel_z[i]);
      assignment_(i) = msg->assignment[i];
    }

    callGlobalTrajReplan();

    return;
  }

  // 尚未看
  //  swarm_global_path_
  void EGOReplanFSM::RecvBroadcastSwarmGlobalPathCallback(const traj_utils::SwarmGlobalPathListConstPtr &msg)
  {
    const size_t guard_drone_id = (size_t)msg->guard_drone_id;
    if ((int)guard_drone_id == planner_manager_->pp_.drone_id)
      return;

    int size = msg->path_num;

    /* Store data */
    swarm_global_path_.resize(size);
    for (int i = 0; i < size; i++)
    {
      swarm_global_path_[i] = Eigen::Vector3d(msg->swarm_global_path_x[i],
                                              msg->swarm_global_path_y[i],
                                              msg->swarm_global_path_z[i]);
    }

    planGlobalTrajbyGivenWps(); // 目前这个函数暂且不看??????尚未有操作???
    return;
  }

  // ok
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  // ok
  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    // cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  // ok
  void EGOReplanFSM::changeFSMCentralState(CENTRAL_FSM_STATE new_state, string pos_call)
  {
    static string central_state_str[8] = {"INIT_2", "CHECK", "CA_REMAP_ASSIGNMENT", "SE_REMAP_ASSIGNMENT", "REPLAN_GLOBAL_TRAJ", "EXEC_ASSIGNMENT"};
    int pre_s = int(central_state_);
    central_state_ = new_state;
    // cout << "[" + pos_call + "]: from " + central_state_str[pre_s] + " to " + central_state_str[int(new_state)] << endl;
  }

  // 部分未看，softmaxLayer，constraint_aware如何计算的????
  void EGOReplanFSM::printFSMCentralState()
  {
    static string central_state_str[8] = {"INIT_2", "CHECK", "CA_REMAP_ASSIGNMENT", "SE_REMAP_ASSIGNMENT", "REPLAN_GLOBAL_TRAJ", "EXEC_ASSIGNMENT"};

    // cout << "\r[Central FSM]: state: " + central_state_str[int(central_state_)];
    std::cout << "central:::" << central_state_str[int(central_state_)] << std::endl;

    int swarm_size = planner_manager_->traj_.swarm_local_goal.size();
    if (swarm_size == planner_manager_->ploy_traj_opt_->getFormationSize() && !have_local_goal_near_target_)
    {
      double t1 = ros::Time::now().toSec();
      double formation_error = planner_manager_->getFormationError(t1, odom_pos_);

      // cout << " formation_error : \033[44m" << formation_error << "\033[0m " << endl;

      Eigen::VectorXd aware_vec(swarm_size);
      for (int i = 0; i < swarm_size; i++)
      {
        if (i == planner_manager_->pp_.drone_id)
          aware_vec(i) = planner_manager_->traj_.local_traj.constraint_aware;
        else
          aware_vec(i) = planner_manager_->traj_.swarm_traj[i].constraint_aware;
      }
      // cout << " aware_vec   : \033[44m" << aware_vec.transpose() << "\033[0m " << endl;

      Eigen::VectorXd aware_softmax_vec;
      planner_manager_->align_assign_.softmaxLayer(aware_vec, aware_softmax_vec);
      // cout << " softmax_vec : \033[44m" << aware_softmax_vec.transpose() << "\033[0m ";
    }
    // cout << endl;
  }

  // ok
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  // 将本机的local_traj数据转化为traj_utils::PolyTraj类型消息msg
  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &msg)
  {
    auto data = &planner_manager_->traj_.local_traj;

    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.traj_id = data->traj_id;
    msg.start_time = ros::Time(data->start_time);
    msg.order = 5; // todo, only support order = 5 now.
    msg.constraint_aware = data->constraint_aware;
    msg.vis_aware = data->vis_aware;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
  }

  void EGOReplanFSM::pathR2ROSMsg(traj_utils::pathR_lll &msg, ros::Time opt_end_time_, int front_traj_piece_, bool follow_leader_)
  {
    auto data_pos = planner_manager_->ploy_traj_opt_->trajROpt.posTraj_;
    auto data_rad = planner_manager_->ploy_traj_opt_->trajROpt.radiusTraj_;
    auto data_lll = planner_manager_->ploy_traj_opt_->trajROpt.lllTraj_;

    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.start_time = opt_end_time_;
    msg.order = 5; // todo, only support order = 5 now.
    msg.traj_piecenum = front_traj_piece_;
    msg.agent_add = agent_add_;
    msg.change_front = planner_manager_->ploy_traj_opt_->number_change;
    msg.use_leader = follow_leader_;

    msg.ass_num = num_now_;
    msg.assig_id.resize(num_now_);
    for (int i = 0; i < num_now_; i++)
    {
      msg.assig_id[i] = assig_id[i];
    }
    msg.assi_x_pos.resize(size_swarm);
    msg.assi_y_pos.resize(size_swarm);
    msg.assi_z_pos.resize(size_swarm);
    for (int i = 0; i < size_swarm; i++)
    {
      msg.assi_x_pos[i] = swarm_des_set_[i][0];
      msg.assi_y_pos[i] = swarm_des_set_[i][1];
      msg.assi_z_pos[i] = swarm_des_set_[i][2];
    }
    Eigen::VectorXd durs = data_pos.getDurations();
    int piece_num = data_pos.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x_pos.resize(6 * piece_num);
    msg.coef_y_pos.resize(6 * piece_num);
    msg.coef_z_pos.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      auto cMat = data_pos.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x_pos[i6 + j] = cMat(0, j);
        msg.coef_y_pos[i6 + j] = cMat(1, j);
        msg.coef_z_pos[i6 + j] = cMat(2, j);
      }
    }

    msg.coef_r.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      auto cMatr = data_rad.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_r[i6 + j] = cMatr(0, j);
      }
    }

    msg.coef_l.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      auto cMatl = data_lll.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_l[i6 + j] = cMatl(0, j);
      }
    }
  }
  // 将本机的localgoal数据转化为traj_utils::LocalGoal类型消息msg
  void EGOReplanFSM::localGoal2ROSMsg(traj_utils::LocalGoal &msg)
  {
    auto data = &planner_manager_->traj_.local_goal;

    msg.drone_id = data->drone_id;
    msg.global_traj_id = data->global_traj_id;

    msg.lg_pos_x = data->lg_pos(0);
    msg.lg_pos_y = data->lg_pos(1);
    msg.lg_pos_z = data->lg_pos(2);

    msg.lg_vel_x = data->lg_vel(0);
    msg.lg_vel_y = data->lg_vel(1);
    msg.lg_vel_z = data->lg_vel(2);
  }

  // 依靠remap_lg_pos_，remap_lg_vel_转化存入为rosmsg msg
  void EGOReplanFSM::remapLocalGoalList2ROSMsg(traj_utils::RemapLocalGoalList &msg)
  {
    int size = remap_lg_pos_.size();

    msg.assignment.resize(size);
    msg.remap_lg_pos_x.resize(size);
    msg.remap_lg_pos_y.resize(size);
    msg.remap_lg_pos_z.resize(size);
    msg.remap_lg_vel_x.resize(size);
    msg.remap_lg_vel_y.resize(size);
    msg.remap_lg_vel_z.resize(size);

    msg.guard_drone_id = planner_manager_->pp_.drone_id;
    for (int i = 0; i < size; i++)
    {
      msg.assignment[i] = assignment_(i);
      msg.remap_lg_pos_x[i] = remap_lg_pos_[i](0);
      msg.remap_lg_pos_y[i] = remap_lg_pos_[i](1);
      msg.remap_lg_pos_z[i] = remap_lg_pos_[i](2);
      msg.remap_lg_vel_x[i] = remap_lg_vel_[i](0);
      msg.remap_lg_vel_y[i] = remap_lg_vel_[i](1);
      msg.remap_lg_vel_z[i] = remap_lg_vel_[i](2);
    }
  }

  //????目前直接return,因为目前是SWARM_MANUAL_TARGET rviz制定发布goal，和target_type_有关???
  // 目前这个函数暂且不看??????
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps;
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      wps.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        wps[i](0) = waypoints_[i][0];
        wps[i](1) = waypoints_[i][1];
        wps[i](2) = waypoints_[i][2];
      }
      end_pt_ = wps.back();
      for (size_t i = 0; i < (size_t)waypoint_num_; i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep();
      }
    }
    else if (target_type_ == TARGET_TYPE::SWARM_PRESET_TARGET)
    {
      int id = planner_manager_->pp_.drone_id;
      Eigen::Vector3d relative_pos;
      relative_pos << swarm_relative_pts_[id][0],
          swarm_relative_pts_[id][1],
          swarm_relative_pts_[id][2];
      end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;
      wps.push_back(end_pt_);
    }
    else if (target_type_ == TARGET_TYPE::GLOBAL_PLANNER_TARGET)
    {
      int id = planner_manager_->pp_.drone_id;
      Eigen::Vector3d relative_pos;
      relative_pos << swarm_relative_pts_[id][0],
          swarm_relative_pts_[id][1],
          swarm_relative_pts_[id][2];
      int size = swarm_global_path_.size();
      for (int i = 0; i < size; i++)
      {
        Eigen::Vector3d wps_pos;
        wps_pos = swarm_global_path_[i] + swarm_scale_ * relative_pos;
        wps.push_back(wps_pos);
      }
      end_pt_ = wps.back();
      for (size_t i = 0; i < (size_t)size; i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep();
      }
    }
    else
      return;

    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero(), wps,
                                                             Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero());

    if (success)
    {
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      have_local_goal_near_target_ = false;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  // 起始点位置速度加速度start_pt_，start_vel_，start_acc_
  // 为当前odom反馈的odom_pos_，odom_vel_，0向量，调用trial_times次callReboundReplan(true, false)
  // 返回其返回数值
  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    log_ztr2 << "in planFromGlobalTraj " << endl;
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      // if (callReboundReplan(true, true))
      if (callReboundReplan(true, false))
      {
        return true;
      }
    }
    return false;
  }

  // 设置起始点位置速度加速度start_pt_，start_vel_，start_acc_
  // 为当前时刻局部轨迹计算出来的位置速度加速度，调用callReboundReplan返回其数值
  bool EGOReplanFSM::planFromLocalTraj(bool flag_use_poly_init, bool use_formation)
  {
    // double t_debug_start = ros::Time::now().toSec();
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(flag_use_poly_init, use_formation);

    if (!success)
      return false;

    // cout << "planFromLocalTraj : " << ros::Time::now().toSec() - t_debug_start << endl;

    return true;
  }
  bool EGOReplanFSM::opt_formation(const Eigen::Matrix3d &start_leader_,
                                   const Eigen::Matrix3d &end_leader_,
                                   const vector<double> &rrr_start_,
                                   const vector<double> &rrr_end_,
                                   const vector<double> &lll_start_,
                                   const vector<double> &lll_end_)
  {
    /******************************************************************************************************** */
    Eigen::Matrix3d start_leader = start_leader_;
    Eigen::Matrix3d end_leader = end_leader_;
    vector<double> rrr_start = rrr_start_;
    vector<double> rrr_end = rrr_end_;
    vector<double> lll_start = lll_start_;
    vector<double> lll_end = lll_end_;
    double opt_score = 0.0;
    path_R_vis = planner_manager_->get_pathR(start_leader, end_leader, rrr_start, rrr_end, lll_start, lll_end, time_opt_now, opt_score);

    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < path_R_vis.size(); i += 8)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world"; // 或者你使用的其他坐标系
      marker.header.stamp = ros::Time::now();
      marker.ns = "path_R_vis";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE; // 使用球体来模拟椭圆
      marker.action = visualization_msgs::Marker::ADD;

      // 设置位置
      marker.pose.position.x = path_R_vis[i][0];
      marker.pose.position.y = path_R_vis[i][1];
      marker.pose.position.z = 1.0; // 高度设置为1.0

      // 设置大小
      marker.scale.x = path_R_vis[i][3] * (path_R_vis[i][2]);       // x方向的大小
      marker.scale.y = path_R_vis[i][3] * (1.0 / path_R_vis[i][2]); // y方向的大小
      marker.scale.z = 0.1;                                         // z方向的大小（可以设置为较小的值）

      // 设置颜色
      marker.color.r = 0.0f;
      marker.color.g = 0.0f; // 绿色
      marker.color.b = 1.0f;
      marker.color.a = 0.3; // 不透明

      marker_array.markers.push_back(marker);
    }
    pathR_pub.publish(marker_array);
    marker_array.markers.clear();

    ros::Time opt_end_time;
    traj_utils::pathR_lll pathR_msg;

    planner_manager_->opt_pathR(start_leader, end_leader, rrr_start, rrr_end, lll_start, lll_end, opt_end_time);

    double traj_lll_to = planner_manager_->ploy_traj_opt_->trajROpt.get_lll(0.1);
    bool follow_leader = false;
    if (abs(traj_lll_to) - 1.0 >= 0.1 || opt_score < 4.0)
    {
      cout << "opt_score:::::::::::::::::::::::::::::::::" << opt_score << endl;
      follow_leader = true;
    }
    else
    {
      follow_leader = false;
    }
    follow_leader = true;
    pathR2ROSMsg(pathR_msg, opt_end_time, path_R_vis.size(), follow_leader);
    posR_traj_pub_.publish(pathR_msg);

    trajP_now = path_R_vis[4];
    traj_lll_now = path_R_vis[4][2];
    flag_have_pathr = true;
    path_R_vis.clear();

    visualization_msgs::MarkerArray marker_array_opt;
    double t_totle = planner_manager_->ploy_traj_opt_->trajROpt.getTotalDuration();
    int i_id = 0;
    double opt_ok_time = ros::Time::now().toSec() - time_opt_now;
    for (double t = opt_ok_time; t <= t_totle; t += t_totle / 6, i_id++)
    {
      Eigen::Vector4d trajP = planner_manager_->ploy_traj_opt_->trajROpt.getPwithR(t);
      double traj_lll = planner_manager_->ploy_traj_opt_->trajROpt.get_lll(t);
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world"; // 或者你使用的其他坐标系
      marker.header.stamp = ros::Time::now();
      marker.ns = "opt_R_vis";
      marker.id = i_id;
      marker.type = visualization_msgs::Marker::SPHERE; // 使用球体来模拟椭圆
      marker.action = visualization_msgs::Marker::ADD;

      // 设置位置
      marker.pose.position.x = trajP[0];
      marker.pose.position.y = trajP[1];
      marker.pose.position.z = trajP[2]; // 高度设置为1.0

      // 设置大小
      marker.scale.x = 2.0 * trajP[3] * (traj_lll);       // x方向的大小
      marker.scale.y = 2.0 * trajP[3] * (1.0 / traj_lll); // y方向的大小
      marker.scale.z = 0.1;                               // z方向的大小（可以设置为较小的值）

      // 设置颜色
      marker.color.r = 0.0f;
      marker.color.g = 1.0f; // 绿色
      marker.color.b = 0.0f;
      marker.color.a = 0.5; // 不透明

      marker_array_opt.markers.push_back(marker);
    }
    opt_pathR_pub.publish(marker_array_opt);
    return true;
    /******************************************************************************************************** */
  }
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool use_formation)
  {
    log_ztr2 << "in callReboundReplan" << endl;

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_);

    // publish its local goal to other aerial robots
    traj_utils::LocalGoal msg1;
    localGoal2ROSMsg(msg1);
    broadcast_localgoal_pub_.publish(msg1);

    // debug for broadcast local goal
    if (planner_manager_->pp_.drone_id == 0)
    {
      int size = planner_manager_->traj_.swarm_local_goal.size();
      vector<Eigen::Vector3d> pos_list, vel_list;
      for (int i = 0; i < size; i++)
      {
        pos_list.push_back(planner_manager_->traj_.swarm_local_goal[i].lg_pos);
        vel_list.push_back(planner_manager_->traj_.swarm_local_goal[i].lg_vel);
      }
      // visualization_->displayLocalGoalList(pos_list, vel_list, 0.2, Eigen::Vector4d(1.0, 0, 0, 1.0));
      log_ztr2
          << "after visualization_" << endl;
    }

    //-----------------------------------------------ztr debug---------------------------------------------------------
    double swarm_size = planner_manager_->traj_.swarm_local_goal.size();

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;
    double desired_start_time;

    // if (have_local_traj_ && use_formation)
    if (have_local_traj_ && use_formation)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      desired_start_pt = info->traj.getPos(desired_start_time - info->start_time);
      desired_start_vel = info->traj.getVel(desired_start_time - info->start_time);
      desired_start_acc = info->traj.getAcc(desired_start_time - info->start_time);
    }
    else
    {
      desired_start_pt = start_pt_;
      desired_start_vel = start_vel_;
      desired_start_acc = start_acc_;
    }
    // desired_start_pt = odom_pos_;
    bool plan_success;

    plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init), use_formation, have_local_traj_);

    have_new_target_ = false; // replan when have new global trajectory

    if (plan_success)
    {
      traj_utils::PolyTraj msg2;
      polyTraj2ROSMsg(msg2);
      poly_traj_pub_.publish(msg2);
      broadcast_ploytraj_pub_.publish(msg2);
      have_local_traj_ = true;
    }

    return plan_success;
  }

  // 生成emergency停止的minco轨迹，并可视化，尚未细看其中EmergencyStop函数???
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj msg;
    polyTraj2ROSMsg(msg);
    poly_traj_pub_.publish(msg);

    /* !!!! Do not broadcast emergency stop trajectory because of formation */
    // broadcast_ploytraj_pub_.publish(msg);

    return true;
  }

  // 依靠nodes_horizon_pos即3.5s后集群uav的位置，现在的uav位置即nodes_cur，aware_weight
  // 重新得到remap_lg_pos与assignment
  // 其中callAlignAssign函数尚未仔细看
  bool EGOReplanFSM::callCALocalGoalRemap(vector<Eigen::Vector3d> &remap_lg_pos,
                                          vector<Eigen::Vector3d> &remap_lg_vel,
                                          Eigen::VectorXi &assignment)
  {
    // printf("\033[43;30m\n[drone %d Constraint-aware]======================================\033[0m\n",
    //        planner_manager_->pp_.drone_id);

    int swarm_size = planner_manager_->traj_.swarm_local_goal.size();
    if (swarm_size < planner_manager_->ploy_traj_opt_->getFormationSize())
      return false;

    // set nodes horizon
    std::vector<Eigen::Vector3d> nodes_horizon_pos, nodes_horizon_vel; // delta_t即3.5s后集群uav的位置，速度等等
    double delta_t = ca_delta_t_;
    double t1 = ros::Time::now().toSec();
    planner_manager_->getFormationPosAndVel(t1, delta_t, nodes_horizon_pos, nodes_horizon_vel);
    // 依靠缓冲区swarm_traj，计算得到当前t再过delta_t时间所有uav的位置速度等等存入swarm_graph_pos与swarm_graph_vel

    // calculate mean velocity
    Eigen::Vector3d mean_vel = Eigen::Vector3d::Zero();
    for (int i = 0; i < swarm_size; i++)
    {
      mean_vel += nodes_horizon_vel[i];
    }
    mean_vel = mean_vel / swarm_size;
    remap_lg_vel.clear();

    if (isuse_local_vel_)
    { // 这里是true，remap_lg_vel全部换为了平均数值
      // use mean velocity
      for (int i = 0; i < swarm_size; i++)
      {
        mean_vel(2) = 0.0;
        remap_lg_vel.push_back(mean_vel);
      }
    }
    else
    {
      // use zero as local velocity
      for (int i = 0; i < swarm_size; i++)
      {
        remap_lg_vel.push_back(Eigen::Vector3d::Zero());
      }
    }

    // get current formation position
    std::vector<Eigen::Vector3d> nodes_cur; // 当前集群uav所有位置
    nodes_cur = planner_manager_->getFormationCurrentPos(t1, odom_pos_);

    // set aware weight
    Eigen::VectorXd aware_weight(swarm_size);
    for (int i = 0; i < swarm_size; i++)
    {
      if (i == planner_manager_->pp_.drone_id)
        aware_weight(i) = planner_manager_->traj_.local_traj.constraint_aware;
      else
        aware_weight(i) = planner_manager_->traj_.swarm_traj[i].constraint_aware;
    }

    // get optimal aligned and assigned local goal
    assignment.resize(swarm_size);
    planner_manager_->callAlignAssign(nodes_horizon_pos, nodes_cur, aware_weight, remap_lg_pos, assignment);

    // cout << "[assignment] : " << assignment.transpose() << endl;

    return true;
  }

  // 调用callAlignAssign依据traj_.swarm_local_goal作为nodes_horizon，当前uav所有位置作为nodes_cur，所有飞机的aware_weight
  // 重新得到remap_lg_pos，remap_lg_vel与assignment。
  // 其中callAlignAssign函数等待仔细看!!!!!
  bool EGOReplanFSM::callSELocalGoalRemap(vector<Eigen::Vector3d> &remap_lg_pos,
                                          vector<Eigen::Vector3d> &remap_lg_vel,
                                          Eigen::VectorXi &assignment)
  {
    // printf("\033[43;30m\n[drone %d Similarity-error]======================================\033[0m\n",
    //        planner_manager_->pp_.drone_id);

    int swarm_size = planner_manager_->traj_.swarm_local_goal.size();
    if (swarm_size < planner_manager_->ploy_traj_opt_->getFormationSize())
      return false;

    // set local_goal and calculate mean velocity
    std::vector<Eigen::Vector3d> nodes_horizon; // 存入当前集群的swarm_local_goal
    Eigen::Vector3d mean_vel = Eigen::Vector3d::Zero();
    for (int i = 0; i < swarm_size; i++)
    {
      nodes_horizon.push_back(planner_manager_->traj_.swarm_local_goal[i].lg_pos);
      mean_vel += planner_manager_->traj_.swarm_local_goal[i].lg_vel;
    }
    mean_vel = mean_vel / swarm_size;
    remap_lg_vel.clear();
    for (int i = 0; i < swarm_size; i++)
    {
      remap_lg_vel.push_back(mean_vel);
    }

    // get current formation position
    std::vector<Eigen::Vector3d> nodes_cur;
    double t1 = ros::Time::now().toSec();
    nodes_cur = planner_manager_->getFormationCurrentPos(t1, odom_pos_);

    // set aware weight
    Eigen::VectorXd aware_weight = Eigen::VectorXd::Ones(swarm_size);

    // get optimal aligned and assigned local goal
    assignment.resize(swarm_size);
    planner_manager_->callAlignAssign(nodes_horizon, nodes_cur, aware_weight, remap_lg_pos, assignment);
    // 依据localgoal作为nodes_horizon，现在的nodes_cur，还有aware_weight得到opt_goals与opt_assignment
    //  cout << "[assignment] : " << assignment.transpose() << endl;

    return true;
  }

  // 全局轨迹replan,其中调用了planGlobalTrajReMap，还没细看
  // 最后可视化得到的gloabl_traj即planner_manager_->traj_.global_traj.traj，在rviz中绿色轨迹点。
  bool EGOReplanFSM::callGlobalTrajReplan()
  {
    bool success = false;

    std::vector<Eigen::Vector3d> wps;
    // add reassigened local goal and global goal
    int goal_id = assignment_(planner_manager_->pp_.drone_id);
    int id = planner_manager_->pp_.drone_id;
    Eigen::Vector3d relative_pos, temp_pos;
    relative_pos << swarm_relative_pts_[goal_id][0],
        swarm_relative_pts_[goal_id][1],
        swarm_relative_pts_[goal_id][2];

    if (target_type_ == TARGET_TYPE::GLOBAL_PLANNER_TARGET)
    {
      for (int i = 0; i < swarm_global_path_.size(); i++)
      {
        temp_pos = swarm_global_path_[i] + swarm_scale_ * relative_pos;
        double dist = (temp_pos - odom_pos_).norm();
        if (dist > (remap_lg_pos_[id] - odom_pos_).norm())
          wps.push_back(temp_pos);
      }

      for (int i = 0; i < wps.size(); i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep();
      }
    }
    else
    {
      temp_pos = swarm_central_pos_ + swarm_scale_ * relative_pos;
      wps.push_back(temp_pos);
    }
    end_pt_ = wps.back(); // 终点
    // 获取minco全局轨迹使用分段，一段当前位置到local_goal，一段local_goal到终点
    success = planner_manager_->planGlobalTrajReMap(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                    remap_lg_pos_[id], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                    wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); // 获取minco全局轨迹使用traj_.setGlobalTraj

    // refine the planning_horizen_
    // double dist = (remap_lg_pos_[id] - odom_pos_).norm();
    // if (dist < planning_horizen_min_){
    //   planning_horizen_ = planning_horizen_min_;
    // } else if (dist > planning_horizen_max_){
    //   planning_horizen_ = planning_horizen_max_;
    // } else{
    //   planning_horizen_ = dist;
    // }

    if (success) // 在上面调用planGlobalTrajReMap函数获取了minco全局轨迹planner_manager_->traj_.global_traj
    {
      // update the swarm graph
      // planner_manager_->ploy_traj_opt_->updateSwarmGraph(assignment_);

      /*** display ***/ // 全局轨迹可视化操作
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++) // 以0.1时间分辨率显示绿色全局轨迹
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      } // 得到起点到终点global path即rviz中绿色点线

      end_vel_.setZero();

      have_target_ = true;
      have_new_target_ = true;
      have_local_goal_near_target_ = false;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0); // 显示起点到终点global path即rviz中绿色点线
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  // 依靠constraints程度判断是否需要need_remap，其中softmaxLayer尚未仔细看
  bool EGOReplanFSM::callCARemapCheck()
  {
    bool need_remap = false;

    int swarm_size = planner_manager_->traj_.swarm_local_goal.size();

    if (swarm_size < planner_manager_->ploy_traj_opt_->getFormationSize())
      return need_remap;

    /* The first local_goal remap situation: several local trajectories have been constrained by obstacles */
    Eigen::VectorXd aware_vec(swarm_size);
    for (int i = 0; i < swarm_size; i++)
    {
      if (i == planner_manager_->pp_.drone_id)
        aware_vec(i) = planner_manager_->traj_.local_traj.constraint_aware;
      else
        aware_vec(i) = planner_manager_->traj_.swarm_traj[i].constraint_aware;
    }

    Eigen::VectorXd aware_softmax_vec;
    planner_manager_->align_assign_.softmaxLayer(aware_vec, aware_softmax_vec);

    // check
    if (aware_vec.maxCoeff() > aware_clearance_ && aware_softmax_vec.maxCoeff() > softmax_aware_clearance_)
      need_remap = true;

    return need_remap;
  }

  // 利用odom信息与其他飞机位置检查当前formation_error考虑是否需要need_remap。formation_error考虑使用其他飞机的里程计的信息更加可靠???
  bool EGOReplanFSM::callSERemapCheck()
  {
    bool need_remap = false;

    int swarm_size = planner_manager_->traj_.swarm_local_goal.size();
    if (swarm_size < planner_manager_->ploy_traj_opt_->getFormationSize())
      return need_remap;

    /* The second local_goal remap situation: swarm formation similarity error is too large */
    double t1 = ros::Time::now().toSec();
    double formation_error = planner_manager_->getFormationError(t1, odom_pos_);

    log_ztr2 << index << "\t" << formation_error << endl;
    index++;

    if (formation_error > formation_error_clearance_)
      need_remap = true;

    return need_remap;
  }
  void EGOReplanFSM::FormationChange(const traj_utils::FormationType::ConstPtr &msg)
  {
    // log_ztr2<<"------------------FormationChange------------------------"<<endl;
    int swarm_size = planner_manager_->ploy_traj_opt_->getFormationSize();
    //  log_ztr2<<"---swarm_size-----"<<swarm_size<<endl;
    // Eigen::MatrixX3d pos(swarm_size,3);

    // printf("\033[43;30m\n[FORMATION CHANGE]=================================================\033[0m\n");
    for (int i = 0; i < swarm_size; i++)
    {
      nh_.param("t" + to_string((int)msg->formation_type) + "_relative_pos_" + to_string(i) + "/x", swarm_pts_[i][0], -1.0);
      nh_.param("t" + to_string((int)msg->formation_type) + "_relative_pos_" + to_string(i) + "/y", swarm_pts_[i][1], -1.0);
      nh_.param("t" + to_string((int)msg->formation_type) + "_relative_pos_" + to_string(i) + "/z", swarm_pts_[i][2], -1.0);
    }

    for (int i = 0; i < swarm_size; i++)
    {
      swarm_relative_pts_[i][0] = swarm_pts_[i][0];
      swarm_relative_pts_[i][1] = swarm_pts_[i][1];
      swarm_relative_pts_[i][2] = swarm_pts_[i][2];
    }
    changeFSMCentralState(SE_REMAP_ASSIGNMENT, "Central FSM");
    formationchangereplan();
    if (central_state_ == EXEC_ASSIGNMENT)
      changeFSMCentralState(CHECK, "Central FSM");
  } // namespace ego_planner

  void EGOReplanFSM::formationchangereplan()
  {

    init_pt_ = odom_pos_;
    bool success = false;
    int id = assignment_(planner_manager_->pp_.drone_id);
    Eigen::Vector3d relative_pos;
    relative_pos << swarm_relative_pts_[id][0],
        swarm_relative_pts_[id][1],
        swarm_relative_pts_[id][2];
    end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // success = planner_manager_->planGlobalTrajWaypoints(
    //     odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
    //     one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      formation_change_fsm = true;
      planner_manager_->form_change_get(formation_change_fsm);
      // if(false)
      // {
      //   bool success = callCALocalGoalRemap(remap_lg_pos_, remap_lg_vel_, assignment_);
      //   if (success)
      //   {
      //     traj_utils::RemapLocalGoalList msg;
      //     remapLocalGoalList2ROSMsg(msg);
      //     broadcast_remaplocalgoal_pub_.publish(msg);
      //     changeFSMCentralState(REPLAN_GLOBAL_TRAJ, "Central FSM");
      //   }
      // }
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      start_pub_.publish(flag_msg);

      /*** display ***/ // 0.1时间分辨率可视化轨迹
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      have_local_goal_near_target_ = false;

      /*** FSM ***/
      // debug
      if (central_state_ == EXEC_ASSIGNMENT)
        changeFSMCentralState(CHECK, "Central FSM");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  // 三维版Lloyd迭代
  std::vector<Eigen::Vector3d> EGOReplanFSM::lloydIteration3D(const std::vector<Eigen::Vector3d> &polygon,
                                                              const std::vector<Eigen::Vector3d> &points,
                                                              int sample_size)
  {
    // 计算投影包围盒
    Eigen::Vector2d min_coord(polygon[0].x(), polygon[0].y());
    Eigen::Vector2d max_coord = min_coord;
    for (const auto &p : polygon)
    {
      min_coord = min_coord.cwiseMin(Eigen::Vector2d(p.x(), p.y()));
      max_coord = max_coord.cwiseMax(Eigen::Vector2d(p.x(), p.y()));
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_x(min_coord.x(), max_coord.x());
    std::uniform_real_distribution<double> dis_y(min_coord.y(), max_coord.y());

    // 生成三维采样点（Z保持默认）
    std::vector<Eigen::Vector3d> valid_samples;
    for (int i = 0; i < sample_size; ++i)
    {
      Eigen::Vector3d s(dis_x(gen), dis_y(gen), 0);
      if (pointInPolygon3D(s, polygon))
        valid_samples.push_back(s);
    }

    // 最近邻分配（仅用XY坐标）
    std::vector<Eigen::Vector3d> new_points(points.size(), Eigen::Vector3d::Zero());
    std::vector<int> counts(points.size(), 0);

    for (const auto &s : valid_samples)
    {
      int nearest = -1;
      double min_dist = std::numeric_limits<double>::max();

      // 仅计算XY距离
      for (size_t i = 0; i < points.size(); ++i)
      {
        Eigen::Vector2d p1(s.x(), s.y());
        Eigen::Vector2d p2(points[i].x(), points[i].y());
        double dist = (p1 - p2).squaredNorm();
        if (dist < min_dist)
        {
          min_dist = dist;
          nearest = i;
        }
      }

      if (nearest != -1)
      {
        new_points[nearest].x() += s.x();
        new_points[nearest].y() += s.y();
        counts[nearest]++;
      }
    }

    // 更新质心（保留原始Z值）
    for (size_t i = 0; i < points.size(); ++i)
    {
      if (counts[i] > 10)
      {
        new_points[i].x() /= counts[i];
        new_points[i].y() /= counts[i];
        new_points[i].z() = points[i].z(); // 保持原始Z坐标
      }
      else
      {
        new_points[i] = points[i];
      }
    }

    return new_points;
  }

  double EGOReplanFSM::findMinDistance(const std::vector<Eigen::Vector3d> &points)
  {
    if (points.size() < 2)
    {
      throw std::logic_error("At least two points required");
    }

    double total_distance = 0.0;

    // 外层循环遍历每个点
    for (size_t i = 0; i < points.size(); ++i)
    {
      double min_squared = std::numeric_limits<double>::max();

      // 内层循环寻找当前点的最近邻
      for (size_t j = 0; j < points.size(); ++j)
      {
        if (i == j)
          continue; // 跳过自身比较

        const Eigen::Vector3d diff = points[i] - points[j];
        const double squared_dist = diff.squaredNorm();

        if (squared_dist < min_squared)
        {
          min_squared = squared_dist;
        }
      }
      // 累加实际距离（平方根转换）
      total_distance += std::sqrt(min_squared);
    }

    // 计算平均距离
    return total_distance / points.size();
  }

  std::vector<Eigen::Vector3d> EGOReplanFSM::generateGridCenters(
      const std::vector<Eigen::Vector3d> &polygon,
      int A,
      double tolerance)
  {
    // 1. 投影到XY平面并计算面积（网页3的鞋带公式）
    auto shoelaceArea = [&]()
    {
      double area = 0;
      for (size_t i = 0; i < polygon.size(); ++i)
      {
        const auto &p1 = polygon[i];
        const auto &p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
      }
      return std::abs(area) / 2.0;
    };
    const double area = shoelaceArea();

    // 2. 计算包围盒（网页5的边界处理思想）
    Eigen::Vector3d min_pt = polygon[0], max_pt = polygon[0];
    for (const auto &p : polygon)
    {
      min_pt.x() = std::min(min_pt.x(), p.x());
      min_pt.y() = std::min(min_pt.y(), p.y());
      max_pt.x() = std::max(max_pt.x(), p.x());
      max_pt.y() = std::max(max_pt.y(), p.y());
    }

    // 3. 动态调整栅格尺寸（网页7的面积估算优化）
    double grid_size = std::sqrt(area / A);
    double low = 0.01, high = std::hypot(max_pt.x() - min_pt.x(), max_pt.y() - min_pt.y());

    // 二分法优化（网页3的迭代思想）
    for (int iter = 0; iter < 20; ++iter)
    {
      grid_size = (low + high) / 2;
      int count = 0;

      // 遍历栅格中心点（网页8的空间遍历方法）
      for (double x = min_pt.x() + grid_size / 2; x < max_pt.x(); x += grid_size)
      {
        for (double y = min_pt.y() + grid_size / 2; y < max_pt.y(); y += grid_size)
        {
          // 射线法判断点是否在内部（网页2的点包含检测）
          Eigen::Vector3d p(x, y, 0);
          bool inside = false;
          for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
          {
            const auto &pi = polygon[i];
            const auto &pj = polygon[j];
            if (((pi.y() > p.y()) != (pj.y() > p.y())) &&
                (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x()))
              inside = !inside;
          }
          if (inside)
            count++;
        }
      }

      // 调整搜索范围（网页7的二分逻辑）
      if (count < A * (1 - tolerance))
        high = grid_size;
      else if (count > A * (1 + tolerance))
        low = grid_size;
      else
        break;
    }

    // 4. 收集有效点
    std::vector<Eigen::Vector3d> centers;
    for (double x = min_pt.x() + grid_size / 2; x < max_pt.x(); x += grid_size)
    {
      for (double y = min_pt.y() + grid_size / 2; y < max_pt.y(); y += grid_size)
      {
        Eigen::Vector3d p(x, y, 0);
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
          const auto &pi = polygon[i];
          const auto &pj = polygon[j];
          if (((pi.y() > p.y()) != (pj.y() > p.y())) &&
              (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x()))
            inside = !inside;
        }
        if (inside && centers.size() < static_cast<size_t>(A))
          centers.emplace_back(x, y, 0);
      }
    }

    // 5. 数量补偿机制（网页6的栅格分割策略）
    if (centers.size() < static_cast<size_t>(A))
    {
      // 计算多边形质心（网页3的几何中心思想）
      Eigen::Vector3d centroid(0, 0, 0);
      for (const auto &p : polygon)
        centroid += p;
      centroid /= polygon.size();

      // 生成次级栅格
      double sub_grid = grid_size / 2.0;
      std::vector<Eigen::Vector3d> candidates;
      for (int layer = 1; candidates.size() < (A - centers.size()); ++layer)
      {
        // 螺旋扫描次级栅格
        for (double dx = -layer * sub_grid; dx <= layer * sub_grid; dx += sub_grid)
        {
          for (double dy = -layer * sub_grid; dy <= layer * sub_grid; dy += sub_grid)
          {
            Eigen::Vector3d p(centroid.x() + dx, centroid.y() + dy, 0);
            // 点包含检测
            bool inside = false;
            for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
            {
              const auto &pi = polygon[i];
              const auto &pj = polygon[j];
              if (((pi.y() > p.y()) != (pj.y() > p.y())) &&
                  (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x()))
                inside = !inside;
            }
            if (inside && std::find(centers.begin(), centers.end(), p) == centers.end())
              candidates.push_back(p);
          }
        }
        // 按距离质心排序（网页3的向量思想）
        std::sort(candidates.begin(), candidates.end(),
                  [&](const auto &a, const auto &b) -> bool
                  {
                    return (a.template head<2>() - centroid.template head<2>()).squaredNorm() <
                           (b.template head<2>() - centroid.template head<2>()).squaredNorm();
                  });
        // 合并有效点
        while (centers.size() < static_cast<size_t>(A) && !candidates.empty())
        {
          centers.push_back(candidates.back());
          candidates.pop_back();
        }
      }
    }
    // 6. 数量筛选机制（网页5的重要性法）
    else if (centers.size() > static_cast<size_t>(A))
    {
      // 计算面积加权重心
      Eigen::Vector3d area_centroid(0, 0, 0);
      double total_weight = 0;
      for (size_t i = 0; i < polygon.size(); ++i)
      {
        const auto &p1 = polygon[i];
        const auto &p2 = polygon[(i + 1) % polygon.size()];
        double weight = p1.x() * p2.y() - p2.x() * p1.y();
        area_centroid += (p1 + p2) * weight;
        total_weight += weight;
      }
      area_centroid /= (3 * total_weight);

      // 按重心距离排序
      std::sort(centers.begin(), centers.end(),
                [&](const auto &a, const auto &b) -> bool
                {
                  return (a.template head<2>() - area_centroid.template head<2>()).squaredNorm() <
                         (b.template head<2>() - area_centroid.template head<2>()).squaredNorm();
                });

      // 均匀间隔删除（网页6的栅格优化）
      std::vector<Eigen::Vector3d> filtered;
      double step = static_cast<double>(centers.size()) / A;
      for (int i = 0; i < A; ++i)
        filtered.push_back(centers[std::round(i * step)]);
      centers = filtered;
    }

    return centers;
  }

  Eigen::Vector3d EGOReplanFSM::findMinEnclosingCircle(const std::vector<Eigen::Vector3d> &concave_poly)
  {
    // 三维转二维并去重（网页1的投影处理思想）
    std::vector<Eigen::Vector2d> points;
    for (const auto &p3d : concave_poly)
    {
      Eigen::Vector2d p(p3d.x(), p3d.y());
      if (std::none_of(points.begin(), points.end(),
                       [&](const Eigen::Vector2d &exist)
                       {
                         return exist.isApprox(p, 1e-6);
                       }))
      {
        points.push_back(p);
      }
    }

    // 随机化点集顺序（网页3的算法稳定性优化）
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(points.begin(), points.end(), g);

    Eigen::Vector2d center = Eigen::Vector2d::Zero();

    // C++14兼容的圆计算Lambda（网页7的泛型Lambda特性）
    auto computeCenter = [](const std::vector<Eigen::Vector2d> &support) -> Eigen::Vector2d
    {
      switch (support.size())
      {
      case 0:
        return Eigen::Vector2d::Zero();
      case 1:
        return support[0];
      case 2:
        return (support[0] + support[1]) * 0.5;
      case 3:
      {
        const Eigen::Vector2d &a = support[0];
        const Eigen::Vector2d &b = support[1];
        const Eigen::Vector2d &c = support[2];

        // 三点共线处理（网页6的几何退化处理）
        const double d = 2 * ((b.x() - a.x()) * (c.y() - a.y()) -
                              (b.y() - a.y()) * (c.x() - a.x()));
        if (std::abs(d) < 1e-9)
        {
          auto cmp = [&a](const Eigen::Vector2d &u, const Eigen::Vector2d &v)
          {
            return (u - a).squaredNorm() < (v - a).squaredNorm();
          };
          const auto p1 = *std::max_element(support.begin(), support.end(), cmp);
          const auto p2 = *std::min_element(support.begin(), support.end(), cmp);
          return (p1 + p2) * 0.5;
        }

        // 行列式法求圆心（网页3的几何计算）
        return Eigen::Vector2d(
            ((c.y() - a.y()) * (b.squaredNorm() - a.squaredNorm()) -
             (b.y() - a.y()) * (c.squaredNorm() - a.squaredNorm())) /
                d,
            ((b.x() - a.x()) * (c.squaredNorm() - a.squaredNorm()) -
             (c.x() - a.x()) * (b.squaredNorm() - a.squaredNorm())) /
                d);
      }
      default:
        return Eigen::Vector2d::Zero();
      }
    };

    // 迭代式Welzl算法实现（网页3的算法改进）
    std::function<void(size_t, std::vector<Eigen::Vector2d>)> welzl =
        [&](size_t n, std::vector<Eigen::Vector2d> R)
    {
      if (n == 0 || R.size() == 3)
      {
        const Eigen::Vector2d candidate = computeCenter(R);

        // 半径验证逻辑替换为直接更新（网页5的算法优化）
        const bool is_better = std::all_of(points.begin(), points.end(),
                                           [&](const Eigen::Vector2d &p)
                                           {
                                             return (p - candidate).squaredNorm() <=
                                                    (p - center).squaredNorm();
                                           });

        if (is_better)
          center = candidate;
        return;
      }

      // 选择最后一个点（网页3的随机化策略）
      const Eigen::Vector2d p = points[n - 1];

      // 递归处理非包含情况
      welzl(n - 1, R);

      // 递归处理包含情况
      R.push_back(p);
      welzl(n - 1, R);
    };

    welzl(points.size(), {});
    return Eigen::Vector3d(center.x(), center.y(), 0.0);
  }

  vector<Eigen::Vector3d> EGOReplanFSM::lloyd_iteration_3d(
      const std::vector<Eigen::Vector3d> &samples,
      const vector<Eigen::Vector3d> &points)
  {
    // 建立簇容器
    double tttt1 = ros::Time::now().toSec();

    vector<vector<Eigen::Vector3d>> clusters(points.size());
    // 分配采样点（仅比较XY距离）
    for (const auto &sample : samples)
    {
      int nearest = 0;
      double min_sqdist = numeric_limits<double>::max();

      for (size_t i = 0; i < points.size(); ++i)
      {
        // 仅计算XY平面距离
        Eigen::Vector2d delta = sample.head<2>() - points[i].head<2>();
        double sqdist = delta.squaredNorm();

        if (sqdist < min_sqdist)
        {
          min_sqdist = sqdist;
          nearest = i;
        }
      }
      clusters[nearest].push_back(sample);
    }
    // vector<vector<Eigen::Vector3d>> clusters = clusterWithKDTree(points, samples);
    // 分配样本点到最近的聚类中心
    // assign_clusters(samples, points, clusters);
    double tttt2 = ros::Time::now().toSec();
    cout << "ttttttttttttttttttttttttttttttttttTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT222:::" << tttt2 - tttt1 << endl;
    // 计算新质心（保留原始z值）
    vector<Eigen::Vector3d> new_points;
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      if (clusters[i].empty())
      {
        new_points.push_back(points[i]);
        continue;
      }

      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const auto &p : clusters[i])
      {
        sum += p;
      }
      Eigen::Vector3d centroid = sum / clusters[i].size();

      // 保留原始z坐标（或设置默认值）
      centroid.z() = points[i].z();
      new_points.push_back(centroid);
    }
    return new_points;
  }
} // namespace ego_planner
