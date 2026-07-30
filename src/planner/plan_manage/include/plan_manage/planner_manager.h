/***
 * @Author: ztr
 * @Date: 2022-05-12 00:00:56
 * @LastEditTime: 2022-05-12 21:17:11
 * @LastEditors: ztr
 * @Description:
 * @FilePath: /Pipline-Swarm-Formation/src/planner/plan_manage/include/plan_manage/planner_manager.h
 */
#pragma once

#include <stdlib.h>
#include <ros/ros.h>
#include <plan_env/grid_map.h>
// #include <path_searching/topo_prm.h>
#include <traj_utils/DataDisp.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>
#include <optimizer/poly_traj_optimizer.h>
#include <path_searching/align_assign.hpp>
#include <fstream>
#include <nav_msgs/Odometry.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    std::ofstream log_ztr;    // for data analysis,ztr debug
    ros::Subscriber odom_sub; // local plan
    nav_msgs::Odometry odom;
    vector<Eigen::Vector3d> near_drone;

    void odom_cmb(const nav_msgs::OdometryConstPtr &msg) { odom = *msg; }

    bool last_kino_success;

    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const double trajectory_start_time,
        const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel,
        const bool flag_polyInit, const bool use_formation, const bool have_local_traj);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool computeInitReferenceState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const double &ts, poly_traj::MinJerkOpt &initMJO,
        const bool flag_polyInit);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajReMap(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
        const Eigen::Vector3d &remap_lg_pos, const Eigen::Vector3d &remap_lg_vel, const Eigen::Vector3d &remap_lg_acc,
        const std::vector<Eigen::Vector3d> &wps, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planSwarmGlobalPath(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos, const double &swarm_scale,
        std::vector<Eigen::Vector3d> &swarm_global_path);
    void getLocalTarget(
        const double planning_horizen, const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
        Eigen::Vector3d &local_target_vel);
    void getLocalTargetHorizenTime(
        const double planning_horizen_time,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel);

    void callAlignAssign(const std::vector<Eigen::Vector3d> &nodes_horizon, const std::vector<Eigen::Vector3d> &nodes_cur,
                         const Eigen::VectorXd &aware_weight, std::vector<Eigen::Vector3d> &opt_goals,
                         Eigen::VectorXi &opt_assignment);
    void callAssign(const std::vector<Eigen::Vector3d> &nodes_cur,
                    const std::vector<Eigen::Vector3d> &nodes_des,
                    Eigen::VectorXi &opt_assignment);
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    void deliverTrajToOptimizer(void) { ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj); };
    void setDroneIdtoOpt(void) { ploy_traj_opt_->setDroneId(pp_.drone_id); }
    double getSwarmClearance(void) { return ploy_traj_opt_->getSwarmClearance(); }
    bool checkCollision(int drone_id);
    void fakeSwarmTrajs(void);
    void form_change_get(bool get_change_formation);
    double getFormationError(double t, Eigen::Vector3d pos);
    std::vector<Eigen::Vector3d> getFormationCurrentPos(double t, Eigen::Vector3d pos);
    void getFormationPosAndVel(double t, double delta_t,
                               vector<Eigen::Vector3d> &swarm_graph_pos,
                               vector<Eigen::Vector3d> &swarm_graph_vel);
    void getneardrone(const vector<Eigen::Vector3d> &near_odom) { near_drone = near_odom; };
    // topo
    int findRank(std::vector<double> &data, double value);
    std::vector<Eigen::Vector4d> get_pathR(const Eigen::Matrix3d &start_leader_,
                                           const Eigen::Matrix3d &end_leader_,
                                           const vector<double> &rrr_start_,
                                           const vector<double> &rrr_end_,
                                           const vector<double> &lll_start_,
                                           const vector<double> &lll_end_,
                                           const double &opt_time_now,
                                           double &opt_score);
    void opt_pathR(const Eigen::Matrix3d &start_leader_,
                   const Eigen::Matrix3d &end_leader_,
                   const vector<double> &rrr_start_,
                   const vector<double> &rrr_end_,
                   const vector<double> &lll_start_,
                   const vector<double> &lll_end_,
                   ros::Time &time_opt_end);
    vector<Eigen::Vector3d> getCurrentPos_all(double t, Eigen::Vector3d pos, vector<Eigen::Vector3d> nodes_des);
    // void get_pathR_lll(const double &t, const Eigen::Vector4d &pos_R, const double &lll);
    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;
    // unique_ptr<TopologyPRM> topo_prm_;
    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    AlignAssign align_assign_;
    std::vector<Eigen::Vector4d> path_R_;

    bool start_flag_, reach_flag_;
    ros::Time global_start_time_;
    double start_time_, reach_time_, average_plan_time_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;
    bool use_kino;
    int continous_failures_count_{0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;
    ros::Publisher Astar_pub, init_traj_pub;
    // !SECTION
  };
} // namespace ego_planner