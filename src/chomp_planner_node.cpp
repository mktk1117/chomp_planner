/*
 * chomp_planner_node.cpp
 *
 *  Created on: Mar 19, 2019
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 */

#include <ros/ros.h>
#include <chomp_planner/ChompPlannerNode.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_planner");

  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle("~");
  chomp_planner::ChompPlannerNode chomp_planner_node(nodeHandle, privateNodeHandle);
  // ros::Rate rate(1.0);
  // while (nodeHandle.ok()) {
  //   trajectoryPlannerRos.publishSubMap(grid_map::Position(0, 0), grid_map::Length(1, 1));
  //   rate.sleep();
  // }


  ros::spin();
  return 0;
}
