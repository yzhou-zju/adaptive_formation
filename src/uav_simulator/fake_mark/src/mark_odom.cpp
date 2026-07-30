#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <traj_utils/PolyTraj.h>
#include <traj_utils/plan_container.hpp>
#include <plan_env/grid_map.h>
#include <relative_loc/DistanceMeas.h>
using namespace ego_planner;

ros::Subscriber _cmd_sub, broadcast_ploytraj_sub_;
ros::Publisher _mark_pub;
ego_planner::TrajContainer traj_z;
quadrotor_msgs::PositionCommand _cmd;
GridMap::Ptr grid_map_;
double _init_x, _init_y, _init_z;
int mydrone_id;
double safe_l = 0.2;
double vis_l  = 7.5;

bool rcv_cmd = false;
// fstream file
void RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
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
	if ((int)recv_id == mydrone_id)
		return; // 不接受自己的轨迹

	/* Fill up the buffer */
	if (traj_z.swarm_traj.size() <= recv_id)
	{
		for (size_t i = traj_z.swarm_traj.size(); i <= recv_id; i++)
		{
			LocalTrajData blank;
			blank.drone_id = -1; // 表示还没接收到轨迹
			traj_z.swarm_traj.push_back(blank);
		}
	}

	/* Store data */ // swarm_traj中根据飞机id存储其轨迹
	// if(formation_change_fsm)
	// {
	//   formation_change_fsm = false;
	//   planner_manager_->form_change_get(formation_change_fsm);
	// }

	traj_z.swarm_traj[recv_id].drone_id = recv_id;
	traj_z.swarm_traj[recv_id].traj_id = msg->traj_id;
	traj_z.swarm_traj[recv_id].start_time = msg->start_time.toSec();
	traj_z.swarm_traj[recv_id].constraint_aware = msg->constraint_aware;

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
	traj_z.swarm_traj[recv_id].traj = trajectory;

	traj_z.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
	traj_z.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

}

bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh) {

  Eigen::Vector3d ray_pt;
  double dist;
  double res_test = 200;
  int i = 0;
  while (i<res_test) {
    
    ray_pt = p1 + i*(p2-p1)/200;
    dist = grid_map_->getDistance(ray_pt);

    if (dist <= thresh) {
      return false;
    }
	i++;
  }
  return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_marker");
	ros::NodeHandle nh("~");
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

	nh.param("init_x", _init_x, 0.0);
	nh.param("init_y", _init_y, 0.0);
	nh.param("init_z", _init_z, 0.0);
    // /bridge/distance_meas
    _mark_pub = nh.advertise<relative_loc::DistanceMeas>("planning/trajectory", 10);
	broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::PolyTraj>("planning/broadcast_traj_recv", 100, RecvBroadcastPolyTrajCallback);

	ros::Rate rate(100);
	bool status = ros::ok();
	while (status)
	{
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}

	return 0;
}