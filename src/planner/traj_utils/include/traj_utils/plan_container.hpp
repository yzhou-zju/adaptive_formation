#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace ego_planner
{

  struct GlobalTrajData
  {
    poly_traj::Trajectory traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double constraint_aware;   // constraint-aware of this traj in the formation
    double vis_aware;   // constraint-aware of this traj in the formation
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;
  };

  struct LocalGoalData
  {
    int drone_id;
    int global_traj_id;   // the time of global trajectory replan

    Eigen::Vector3d lg_pos;
    Eigen::Vector3d lg_vel;
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;
  typedef std::vector<LocalGoalData> SwarmLocalGoalData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;//std::vector<ego_planner::LocalTrajData>
    LocalGoalData local_goal;
    SwarmLocalGoalData swarm_local_goal;//std::vector<LocalGoalData>

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;//The corresponding global trajectory time of the current local target. Used in local target selection process
      global_traj.last_glb_t_of_lc_tgt = -1.0;//The corresponding global trajectory time of the last local target. Used in initial-path-from-last-optimal-trajectory generation process

      local_traj.drone_id = -1;
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
      local_traj.constraint_aware = 0.0;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const double &world_time, const double vis_aware_, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.constraint_aware = 0.0;    // 0.0 : no formation
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
    }

    void setLocalTraj(const poly_traj::Trajectory &trajectory, const double &world_time, 
                      const double aware, const double vis_aware_, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.constraint_aware = aware;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj = trajectory;
    }

    void setLocalGoal(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const int drone_id = 0)
    {
      local_goal.drone_id = drone_id;
      local_goal.global_traj_id++;// the time of global trajectory replan
      local_goal.lg_pos = pos;
      local_goal.lg_vel = vel;

      /* Fill up the buffer */
      if (swarm_local_goal.size() <= drone_id)
      {
        for (size_t i = swarm_local_goal.size(); i <= drone_id; i++)
        {
          LocalGoalData blank;
          blank.drone_id = -1;
          swarm_local_goal.push_back(blank);
        }
      }

      swarm_local_goal[drone_id] = local_goal;
    }

  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double max_acc_z;     // physical limits
    double ctrl_pt_dist;           // distance between adjacient B-spline control points
    double polyTraj_piece_length;  // distance between adjacient B-spline control points
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_distinctive_trajs;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif