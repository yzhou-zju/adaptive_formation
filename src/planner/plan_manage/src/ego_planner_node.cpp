/*** 
 * @Author: ztr
 * @Date: 2022-05-12 00:00:43
 * @LastEditTime: 2022-05-12 00:00:43
 * @LastEditors: ztr
 * @Description: 
 * @FilePath: /Pipline-Swarm-Formation/src/planner/plan_manage/src/ego_planner_node.cpp
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
