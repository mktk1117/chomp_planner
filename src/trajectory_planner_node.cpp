/*
 * locomotion_planner_node.cpp
 *
 *  Created on: Mar 4, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <locomotion_planner/trajectory_planner/TrajectoryPlannerRos.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_planner");

  ros::NodeHandle nodeHandle("~");
  locomotion_planner::TrajectoryPlannerRos trajectoryPlannerRos(nodeHandle);
  ros::Rate rate(1.0);
  while (nodeHandle.ok()) {
    trajectoryPlannerRos.publishSubMap(grid_map::Position(0, 0), grid_map::Length(1, 1));
    rate.sleep();
  }


  ros::spin();
  return 0;
}
