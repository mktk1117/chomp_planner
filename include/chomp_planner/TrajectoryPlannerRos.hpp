/*
 * TrajectoryPlannerRos.hpp
 *
 *  Created on: Sep 1, 2017
 *      Author: Takahiro Miki
 *	 Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#pragma once

#include "locomotion_planner/trajectory_planner/SimpleChompPlanner.hpp"
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"
// #include "locomotion_planner/common/type_defs.hpp"
// #include "locomotion_planner/common/Parameters.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
// #include <free_gait_ros/AdapterRos.hpp>

// #include <filters/filter_chain.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

// #include <thread>
using namespace grid_map;

namespace locomotion_planner {

class TrajectoryPlannerRos
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TrajectoryPlannerRos(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TrajectoryPlannerRos();

  void publishSubMap(const Position position, const Length length);

  // void startPoseCallback(const geometry_msgs::PoseStamped& message);
  // void goalPoseCallback(const geometry_msgs::PoseStamped& message);

 private:
  bool readParameters();

  bool callGetSubmap(const Position position, const Length length);

  // void clickedPointCallback(const geometry_msgs::PoseStamped& message);
  void clickedPointCallback(const geometry_msgs::PointStamped& message);

  void planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal);
  void publishLine(std::vector<Eigen::Vector3d> trajectory);
  void publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve);
  void publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos);

  // Trajectory planner
  SimpleChompPlanner chomp_;
  Eigen::Vector3d startPosition_;
  Eigen::Vector3d goalPosition_;

  // Chomp parameters
  int numberOfPoints_;
  double heightClearance_;
  double collisionThreshold_;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Goal/commamd topics.
  ros::Subscriber clickedPointSubscriber_;
  ros::Subscriber goalPoseSubscriber_;
  ros::Subscriber startPoseSubscriber_;
  bool updatedStartPoint_;

  //! Elevation map.
  ros::Publisher elevationMapPublisher_;
  ros::ServiceClient getElevationMapService_;
  std::string elevationMapServiceName_;
  std::string elevationMapWorldFrameId_;
  GridMap map_;

  // ROS publisher
  ros::Publisher sdfPublisher_;
  ros::Publisher trajectoryPublisher_;
  ros::Publisher markerPublisher_;


  // Parameters
  // std::string elevationMapTopicName_;
};

} /* namespace */
