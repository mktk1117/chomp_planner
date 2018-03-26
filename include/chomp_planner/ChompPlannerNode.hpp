/*
 * ChompPlannerNode.hpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 *
 */

#pragma once

#include "chomp_planner/SimpleChompPlanner.hpp"
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"
// #include <memory>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include "voxblox_msgs/Layer.h"
#include "voxblox_ros/tsdf_server.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace voxblox;

namespace chomp_planner {

class ChompPlannerNode
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ChompPlannerNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ChompPlannerNode();

 private:
  bool readParameters();
  //
  // void clickedPointCallback(const geometry_msgs::PointStamped& message);
  //
  // void planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal);
  void publishCommandTrajectory(const std::vector<Eigen::Vector4d>& trajectory);
  // void publishCommandTrajectory(const std::vector<Eigen::Vector3d>& trajectory);
  void publishCommandTrajectory(const PolynomialSplineQuinticVector3Curve& curve);
  void publishLine(const std::vector<Eigen::Vector4d>& trajectory);
  void publishLineFromCurve(const PolynomialSplineQuinticVector3Curve& curve);
  // void publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos);
  void goalPoseCallback(const geometry_msgs::PoseStamped& message);
  void odometryCallback(const nav_msgs::Odometry& message);
  // void commandPoseCallback(const geometry_msgs::Odometry& message);
  void esdfMapCallback(const voxblox_msgs::LayerConstPtr layerMsg);
  void planTrajectory(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
  //
  // Trajectory planner
  SimpleChompPlanner chomp_;
  Eigen::Vector3d startPosition_;
  Eigen::Vector3d startVelocity_;
  Eigen::Vector3d goalPosition_;
  double currentYaw_;
  double goalYaw_;

  // Coordinate parameters
  std::string baseFrameId_;

  // Chomp parameters
  bool fitCurve_;
  int numberOfPoints_;
  double heightClearance_;
  double collisionThreshold_;
  double velocity_;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;

  //! Goal/commamd topics.
  ros::Publisher commandTrajectoryPublisher_;
  ros::Publisher commandPosePublisher_;
  ros::Subscriber clickedPointSubscriber_;
  ros::Subscriber goalPoseSubscriber_;
  ros::Subscriber startPoseSubscriber_;
  ros::Subscriber odometrySubscriber_;
  bool updatedStartPoint_;


  //! ESDF Map.
  ros::Subscriber esdfMapSubcriber_;
  std::shared_ptr<EsdfMap> esdfMap_;

  // ROS publisher
  // ros::Publisher sdfPublisher_;
  ros::Publisher trajectoryPublisher_;
  ros::Publisher markerPublisher_;


  // Parameters
  // std::string elevationMapTopicName_;
};

} /* namespace */
