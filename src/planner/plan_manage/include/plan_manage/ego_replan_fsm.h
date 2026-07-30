#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/pathR_lll.h>
#include <traj_utils/add_out.h>
#include <traj_utils/LocalGoal.h>
#include <traj_utils/RemapLocalGoalList.h>
#include <traj_utils/SwarmGlobalPathList.h>
#include <traj_utils/FormationType.h>
#include <fstream>

#include <iostream>
#include <random>
#include <omp.h>

#include <boost/geometry.hpp>
using std::vector;
namespace ego_planner
{
  // 修改KDNode结构以保存原始索引
  struct KDNode
  {
    Eigen::Vector3d point;
    int index; // 新增：存储该节点在原始points中的索引
    int axis;
    KDNode *left;
    KDNode *right;

    KDNode(const Eigen::Vector3d &p, int idx, int a)
        : point(p), index(idx), axis(a), left(nullptr), right(nullptr) {}
  };
  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      SWARM_MANUAL_TARGET = 3,
      SWARM_PRESET_TARGET = 4,
      GLOBAL_PLANNER_TARGET = 5
    };
    enum CENTRAL_FSM_STATE
    {
      INIT_2,
      CHECK,
      CA_REMAP_ASSIGNMENT,
      SE_REMAP_ASSIGNMENT,
      REPLAN_GLOBAL_TRAJ,
      EXEC_ASSIGNMENT
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    // for formation change-------------------
    ros::Subscriber formation_change;
    void FormationChange(const traj_utils::FormationType::ConstPtr &msg);
    int formation_type;
    std::vector<Eigen::Vector4d> path_R_vis;
    Eigen::Vector4d trajP_now;
    double traj_lll_now = 0.0;
    int agent_add_ = 0;
    bool flag_have_pathr = false;
    double time_opt_now = 0.0;
    std::vector<int> swarm_des_id;
    // for formation change-------------------
    /* parameters */
    int target_type_; // in current version it's 3 (1 mannual select, 2 hard code)
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[200][3];
    int waypoint_num_;
    double planning_horizen_max_, planning_horizen_min_, planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    int last_end_id_;
    double formation_error_clearance_, aware_clearance_, softmax_aware_clearance_;
    double ca_delta_t_;
    double replan_trajectory_time_; // the max time of replanning trajectory(s)
    int size_swarm;
    double vis_hor;
    std::vector<std::vector<int>> group_id_;
    // global goal setting for swarm
    Eigen::Vector3d swarm_central_pos_;
    double swarm_scale_;
    double swarm_relative_pts_[200][3];
    double swarm_pts_[200][3]; // formation change
    std::vector<Eigen::Vector3d> swarm_global_path_;
    std::vector<Eigen::Vector3d> swarm_near;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, have_local_goal_near_target_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    CENTRAL_FSM_STATE central_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;                 // odometry oriention

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;
    bool isuse_local_vel_;

    vector<Eigen::Vector3d> remap_lg_pos_, remap_lg_vel_;
    vector<Eigen::Vector3d> assig_pos_;
    Eigen::VectorXi assignment_;

    GlobalTrajData frontend_traj_;

    bool formation_change_fsm;
    bool formation_change_{false};

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, centra_timer_, pathR_timer_, stop_timer_, error_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, swarm_trajs_sub_, broadcast_bspline_sub_, trigger_sub_, assignment_sub_, odom_sub_z;
    ros::Subscriber swarm_formation_waypoint_sub_, swarm_formation_trigger_sub_;
    ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_, posR_traj_pub_;
    ros::Publisher broadcast_ploytraj_pub_, broadcast_localgoal_pub_, broadcast_remaplocalgoal_pub_, broadcast_swarmglobalpath_pub_;
    ros::Subscriber broadcast_ploytraj_sub_, broadcast_localgoal_sub_, broadcast_remaplocalgoal_sub_, broadcast_swarmglobalpath_sub_, posR_traj_sub_;
    ros::Publisher reached_pub_, start_pub_;
    ros::Publisher pathR_pub;
    ros::Publisher opt_pathR_pub;

    ros::Publisher agent_addout_pub;
    ros::Subscriber agent_addout_sub;

    /* helper functions */
    void errorCallback(const ros::TimerEvent &e);
    bool callReboundReplan(bool flag_use_poly_init, bool use_formation); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                    // front-end and back-end method
    bool callGlobalTrajReplan();
    bool callCARemapCheck();
    bool callSERemapCheck();
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    Eigen::VectorXi assignment_pub();
    vector<Eigen::Vector3d> assignment_pos_get(const Eigen::VectorXi &assignment_pub_get);
    void add_out_Callback(const traj_utils::add_outConstPtr &msg);
    bool opt_formation(const Eigen::Matrix3d &start_leader_,
                       const Eigen::Matrix3d &end_leader_,
                       const vector<double> &rrr_start_,
                       const vector<double> &rrr_end_,
                       const vector<double> &lll_start_,
                       const vector<double> &lll_end_);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void changeFSMCentralState(CENTRAL_FSM_STATE new_state, string pos_call);
    void printFSMExecState();
    void printFSMCentralState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    bool callCALocalGoalRemap(vector<Eigen::Vector3d> &remap_lg_pos,
                              vector<Eigen::Vector3d> &remap_lg_vel,
                              Eigen::VectorXi &assignment);
    bool callSELocalGoalRemap(vector<Eigen::Vector3d> &remap_lg_pos,
                              vector<Eigen::Vector3d> &remap_lg_vel,
                              Eigen::VectorXi &assignment);

    void planGlobalTrajbyGivenWps();

    /* ROS functions */
    void setGlobalGoal(ros::NodeHandle &nh);
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void centralFSMCallback(const ros::TimerEvent &e);
    void formationCallback(const ros::TimerEvent &e);
    bool stopCallback();

    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg);
    void RecvBroadcastLocalGoalCallback(const traj_utils::LocalGoalConstPtr &msg);
    void RecvBroadcastRemapLocalGoalCallback(const traj_utils::RemapLocalGoalListConstPtr &msg);
    void RecvBroadcastSwarmGlobalPathCallback(const traj_utils::SwarmGlobalPathListConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &msg);
    void localGoal2ROSMsg(traj_utils::LocalGoal &msg);
    void pathR2ROSMsg(traj_utils::pathR_lll &msg, ros::Time opt_end_time_, int front_traj_piece_, bool follow_leader_);
    void remapLocalGoalList2ROSMsg(traj_utils::RemapLocalGoalList &msg);
    void formationchangereplan();
    bool frontEndPathSearching();
    bool checkCollision();
    void rcvOtherOdomCallback(const nav_msgs::Odometry &odom);
    void pathR_TrajCallback(const traj_utils::pathR_lllConstPtr &msg);
    std::vector<Eigen::Vector3d> lloydIteration3D(const std::vector<Eigen::Vector3d> &polygon,
                                                  const std::vector<Eigen::Vector3d> &points,
                                                  int sample_size);
    double findMinDistance(const std::vector<Eigen::Vector3d> &points);
    vector<Eigen::Vector3d> lloyd_iteration_3d(
        const std::vector<Eigen::Vector3d> &samples,
        const vector<Eigen::Vector3d> &points);

  private:
    // 修改后的buildKDTree函数
    KDNode *buildKDTree(vector<Eigen::Vector3d> &points, vector<size_t> &indices, int depth = 0)
    {
      if (indices.empty())
        return nullptr;

      int axis = depth % 2;
      auto mid = indices.begin() + indices.size() / 2;

      // 按当前轴对索引排序
      nth_element(indices.begin(), mid, indices.end(),
                  [axis, &points](size_t a, size_t b)
                  {
                    return points[a][axis] < points[b][axis];
                  });

      // 创建带索引的节点
      KDNode *node = new KDNode(points[*mid], *mid, axis);

      // 递归构建子树
      vector<size_t> left_indices(indices.begin(), mid);
      vector<size_t> right_indices(mid + 1, indices.end());
      node->left = buildKDTree(points, left_indices, depth + 1);
      node->right = buildKDTree(points, right_indices, depth + 1);
      return node;
    }
    // KD树最近邻查询[1,6](@ref)
    // 修改后的findNearest函数
    int findNearest(KDNode *root, const Eigen::Vector3d &sample)
    {
      KDNode *best_node = nullptr;
      double best_sqdist = numeric_limits<double>::max();

      // 使用优先队列存储搜索路径[8](@ref)
      using QueueElem = pair<KDNode *, double>;
      auto cmp = [](const QueueElem &a, const QueueElem &b)
      {
        return a.second > b.second;
      };
      priority_queue<QueueElem, vector<QueueElem>, decltype(cmp)> pq(cmp);

      // 主搜索循环
      KDNode *cur = root;
      while (cur || !pq.empty())
      {
        while (cur)
        {
          // 计算当前节点距离
          Eigen::Vector2d delta = sample.head<2>() - cur->point.head<2>();
          double sqdist = delta.squaredNorm();

          // 更新最近节点
          if (sqdist < best_sqdist)
          {
            best_sqdist = sqdist;
            best_node = cur;
          }

          // 将另一分支加入优先队列
          int axis = cur->axis;
          double split = cur->point[axis];
          double sample_val = sample[axis];
          KDNode *other = (sample_val < split) ? cur->right : cur->left;
          if (other)
            pq.emplace(other, pow(sample_val - split, 2));

          // 移动到更近的分支
          cur = (sample_val < split) ? cur->left : cur->right;
        }

        // 检查优先队列中的候选节点
        if (!pq.empty())
        {
          auto [node, min_possible_sqdist] = pq.top();
          pq.pop();
          if (min_possible_sqdist < best_sqdist)
          {
            cur = node;
          }
        }
      }

      // 返回最近点在原始points中的索引（需预先保存索引）
      return best_node->index; // 直接返回存储的索引
    }
    vector<vector<Eigen::Vector3d>> clusterWithKDTree(
        const vector<Eigen::Vector3d> &points,
        const vector<Eigen::Vector3d> &samples)
    {
      // 生成索引列表
      vector<size_t> indices(points.size());
      iota(indices.begin(), indices.end(), 0);

      // 构建带索引的KD树
      vector<Eigen::Vector3d> mod_points = points;
      KDNode *root = buildKDTree(mod_points, indices);
      
      // 初始化聚类容器
      vector<vector<Eigen::Vector3d>> clusters(points.size());

// 并行化处理样本点（OpenMP加速）
#pragma omp parallel for
      for (size_t i = 0; i < samples.size(); ++i)
      {
        int nearest = findNearest(root, samples[i]);
#pragma omp critical
        clusters[nearest].push_back(samples[i]);
      }

      // 释放KD树内存（实际应用应使用智能指针）
      function<void(KDNode *)> deleteTree = [&](KDNode *node)
      {
        if (!node)
          return;
        deleteTree(node->left);
        deleteTree(node->right);
        delete node;
      };
      deleteTree(root);

      return clusters;
    }
    bool ray_casting_3d(const Eigen::Vector3d &point,
                        const vector<Eigen::Vector3d> &polygon)
    {
      // 投影到XY平面
      Eigen::Vector2d point_2d(point.x(), point.y());
      vector<Eigen::Vector2d> polygon_2d;
      for (const auto &p : polygon)
      {
        polygon_2d.emplace_back(p.x(), p.y());
      }
      return ray_casting(point_2d, polygon_2d); // 沿用原二维射线法
    }
    bool ray_casting(const Eigen::Vector2d &point, const vector<Eigen::Vector2d> &polygon)
    {
      double x = point.x(), y = point.y();
      bool inside = false;
      const size_t n = polygon.size();

      for (size_t i = 0, j = n - 1; i < n; j = i++)
      {
        const Eigen::Vector2d &pi = polygon[i];
        const Eigen::Vector2d &pj = polygon[j]; // 修正索引范围检查

        const double yi = pi.y();
        const double yj = pj.y(); // 修正变量名拼写错误
        if ((yi > y) != (yj > y))
        {
          const double xi = pi.x();
          const double xj = pj.x(); // 修正变量名拼写错误
          const double slope = (xj - xi) / (yj - yi + 1e-8);
          const double intersect_x = xi + (y - yi) * slope;

          if (x < intersect_x)
          {
            inside = !inside;
          }
        }
      }
      return inside;
    }
    // 生成三维网格采样点（z轴固定为0）
    vector<Eigen::Vector3d> generate_3d_grid_samples(
        const vector<Eigen::Vector3d> &polygon,
        int resolution)
    {
      // 获取XY平面范围
      Eigen::Vector2d min_xy = Eigen::Vector2d::Constant(INFINITY);
      Eigen::Vector2d max_xy = Eigen::Vector2d::Constant(-INFINITY);
      for (const auto &p : polygon)
      {
        min_xy = min_xy.cwiseMin(p.head<2>());
        max_xy = max_xy.cwiseMax(p.head<2>());
      }

      // 生成二维网格并添加z=0
      vector<Eigen::Vector3d> samples;
      const Eigen::Vector2d step = (max_xy - min_xy) / resolution;

      for (int i = 0; i <= resolution; ++i)
      {
        for (int j = 0; j <= resolution; ++j)
        {
          Eigen::Vector3d p(
              min_xy.x() + i * step.x(),
              min_xy.y() + j * step.y(),
              0.0 // z轴固定值
          );
          if (ray_casting_3d(p, polygon))
          {
            samples.push_back(p);
          }
        }
      }
      return samples;
    }
    // 三维版射线法判断点是否在投影多边形内[6](@ref)
    bool pointInPolygon3D(const Eigen::Vector3d &point, const std::vector<Eigen::Vector3d> &polygon)
    {
      // 投影到XY平面
      Eigen::Vector2d proj_point(point.x(), point.y());
      std::vector<Eigen::Vector2d> proj_poly;
      for (const auto &p : polygon)
      {
        proj_poly.emplace_back(p.x(), p.y());
      }

      bool inside = false;
      size_t n = proj_poly.size();
      for (size_t i = 0, j = n - 1; i < n; j = i++)
      {
        const Eigen::Vector2d &pi = proj_poly[i];
        const Eigen::Vector2d &pj = proj_poly[j];

        if (((pi.y() > proj_point.y()) != (pj.y() > proj_point.y())) &&
            (proj_point.x() < (pj.x() - pi.x()) * (proj_point.y() - pi.y()) / (pj.y() - pi.y()) + pi.x()))
          inside = !inside;
      }
      return inside;
    }

    // 三维版初始样本生成
    std::vector<Eigen::Vector3d> initialSamples3D(const std::vector<Eigen::Vector3d> &polygon, int n_points)
    {
      Eigen::Vector2d min_coord(polygon[0].x(), polygon[0].y());
      Eigen::Vector2d max_coord = min_coord;

      // 计算投影包围盒
      for (const auto &p : polygon)
      {
        min_coord = min_coord.cwiseMin(Eigen::Vector2d(p.x(), p.y()));
        max_coord = max_coord.cwiseMax(Eigen::Vector2d(p.x(), p.y()));
      }

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<double> dis_x(min_coord.x(), max_coord.x());
      std::uniform_real_distribution<double> dis_y(min_coord.y(), max_coord.y());

      std::vector<Eigen::Vector3d> points;
      while (points.size() < n_points)
      {
        Eigen::Vector3d p(dis_x(gen), dis_y(gen), 0); // Z保持默认值
        if (pointInPolygon3D(p, polygon))
          points.push_back(p);
      }
      return points;
    }

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM();
    //-----------------------------------------------ztr debug-------------------------------
    std::ofstream log_ztr2;
    std::ofstream log_zy;
    long int index{0};
    ros::NodeHandle nh_;
    ros::Time t_stop;
    std::vector<Eigen::Vector3d> desired_points;
    double matchAndComputeErrorWithScale(const std::vector<Eigen::Vector3d>& expected,
                                            const std::vector<Eigen::Vector3d>& current); 
    std::vector<Eigen::Vector3d> swarm_des_set_ = {{0.0, 0.0, 0.0},
                                                   {0.4, 1.34, 0.0},
                                                   {0.31, 2.63, 0.0},
                                                   {-1.09, 1.83, 0.0},
                                                   {-2.60, 2.83, 0.0},
                                                   {-3.68, 2.17, 0.0},
                                                   {-3.60, 1.01, 0.0},
                                                   {-3.13, 0.01, 0.0},
                                                   {-2.51, -0.92, 0.0},
                                                   {-1.88, -1.63, 0.0},
                                                   {-0.84, -2.01, 0.0},
                                                   {0.09, -1.43, 0.0},
                                                   {0.91, -0.64, 0.0},
                                                   {1.41, 0.39, 0.0},
                                                   {1.84, 1.57, 0.0},
                                                   {1.41, 2.69, 0.0},
                                                   {-2.31, 1.49, 0.0},
                                                   {-1.94, 0.25, 0.0},
                                                   {-1.05, -0.68, 0.0},
                                                   {-0.9, 0.74, 0.0},

                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0},
                                                   {0.0, 0.0, -99.0}};
    double scale_suit_size = 1.0;
    double x_dist = -99;
    Eigen::VectorXi assig_id;
    int num_now_;
    int form_change;
    int form_change_count = 0;
    Eigen::Vector3d leader_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    int N_POINTS = 20;

    std::vector<Eigen::Vector3d> initial_samples(const std::vector<Eigen::Vector3d> &poly_points, int n_points);
    std::vector<Eigen::Vector3d> generateGridCenters(const std::vector<Eigen::Vector3d> &polygon, int A, double tolerance);
    Eigen::Vector3d findMinEnclosingCircle(const std::vector<Eigen::Vector3d> &concave_poly);
    //-----------------------------------------------ztr debug-------------------------------
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif