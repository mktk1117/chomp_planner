/*
 * TrajectoryPlannerRos.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: Takahiro Miki
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/trajectory_planner/TrajectoryPlannerRos.hpp"

// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_msgs/GetGridMap.h>
#include <locomotion_planner/common/type_defs.hpp>

using namespace grid_map;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType CDerivativeType;
typedef typename curves::Time Time;

namespace locomotion_planner {

TrajectoryPlannerRos::TrajectoryPlannerRos(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      updatedStartPoint_(false),
      numberOfPoints_(30),
      heightClearance_(0.1),
      collisionThreshold_(0.005),
      chomp_(SimpleChompPlanner(nodeHandle, 0.1, 0.01, 30))
{
  readParameters();
  // chomp_ = SimpleChompPlanner(heightClearance_, collisionThreshold_, numberOfPoints_);
  getElevationMapService_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(elevationMapServiceName_);
  elevationMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_submap", 1, true);
  trajectoryPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>( "line_marker", 0 );
  sdfPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("sdf_pointcloud", 1, true);
  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  clickedPointSubscriber_ = nodeHandle_.subscribe("/clicked_point", 1,
                                             &TrajectoryPlannerRos::clickedPointCallback, this);
  ros::spin();

}

TrajectoryPlannerRos::~TrajectoryPlannerRos()
{
}

bool TrajectoryPlannerRos::readParameters()
{
  // Elevation mapping parameters.
  if (!nodeHandle_.param<std::string>("elevation_mapping/service_name", elevationMapServiceName_, "/elevation_mapping/get_submap")) {
    ROS_ERROR("Could not load elevation map service name.");
  }
  if (!nodeHandle_.param<std::string>("elevation_mapping/world_frame_id", elevationMapWorldFrameId_, "map")) {
    ROS_ERROR("Could not load elevation map world frame id.");
  }
  if (!nodeHandle_.param<int>("trajectory_planner/number_of_points", numberOfPoints_, 40)) {
    ROS_ERROR("Could not load number of points.");
  }
  if (!nodeHandle_.param<double>("trajectory_planner/height_clearance", heightClearance_, 0.3)) {
    ROS_ERROR("Could not load height clearance.");
  }
  if (!nodeHandle_.param<double>("trajectory_planner/collision_threshold", collisionThreshold_, 0.05)) {
    ROS_ERROR("Could not load collision threshold.");
  }
}

bool TrajectoryPlannerRos::callGetSubmap(const grid_map::Position position, const grid_map::Length length)
{
  grid_map_msgs::GetGridMap serviceCall;
  serviceCall.request.frame_id = elevationMapWorldFrameId_;
  serviceCall.request.position_x = position.x();
  serviceCall.request.position_y = position.y();
  serviceCall.request.length_x = length.x();
  serviceCall.request.length_y = length.y();

  if (!getElevationMapService_.call(serviceCall)) {
    // const ros::Duration durationSinceLastUpdate = timerEvent.current_real - lastSuccessfulElevationMapUpdate_;
    // if (durationSinceLastUpdate < maxAgeOfLastElevationMapUpdate_) {
    //   MELO_WARN("Failed to call service get elevation map, using old map.");
    //   return;
    // } else {
    //   MELO_WARN("Failed to call service get elevation map, map is not used.");
    //   locomotionPlanner_.clearElevationMap();
    //   grid_map_msgs::GridMap message;
    //   elevationMapPublisher_.publish(message);
    //   return;
    // }
    ROS_INFO("call failed");
    return false;
  }
  grid_map::GridMapRosConverter::fromMessage(serviceCall.response.map, map_);
  return true;
}

void TrajectoryPlannerRos::publishSubMap(const grid_map::Position position, const grid_map::Length length)
{
  ROS_INFO("publishSubMap");
  if(callGetSubmap(position, length)){
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    elevationMapPublisher_.publish(message);
    ROS_INFO("call get submap succeeded");
  }
  else{
    ROS_INFO("call get submap failed");
  }
}

void TrajectoryPlannerRos::clickedPointCallback(const geometry_msgs::PointStamped& message)
{
  ROS_INFO_STREAM("recieved clicked point" << message);
  if (updatedStartPoint_){
    goalPosition_.x() = message.point.x;
    goalPosition_.y() = message.point.y;
    goalPosition_.z() = message.point.z;
    updatedStartPoint_ = false;
    publishStartGoalMarker(startPosition_, goalPosition_);
    ROS_INFO("updated goal pose");
    planTrajectory(startPosition_, goalPosition_);
  }
  else{
    startPosition_.x() = message.point.x;
    startPosition_.y() = message.point.y;
    startPosition_.z() = message.point.z;
    publishStartGoalMarker(startPosition_, goalPosition_);
    updatedStartPoint_ = true;
    ROS_INFO("updated start pose");
  }
  // Pose goalPose;

}

void TrajectoryPlannerRos::planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal)
{
  ROS_INFO("plan trajectory");
  Eigen::Vector3d diff = start - goal;
  diff.z() = 0;
  double d = diff.norm();
  Eigen::Vector3d middle = (start + goal) / 2;
  grid_map::Position position(middle.x(), middle.y());
  grid_map::Length length(4 * d, 4 * d);
  ROS_INFO_STREAM("call sub map at " << position << " l = " << length);
  // callGetSubmap(position, length);
  publishSubMap(position, length);
  chomp_.updateSignedDistanceField(map_, "elevation", heightClearance_ + 0.1);
  sensor_msgs::PointCloud2 pointcloud;
  chomp_.getSDFROSPointCloud(pointcloud);
  sdfPublisher_.publish(pointcloud);
  ROS_INFO("publish sdf");
  CDerivativeType initialVelocity(0, 0, 0);
  CDerivativeType initialAcceleration(0, 0, 1);
  CDerivativeType finalVelocity(0, 0, 0);
  CDerivativeType finalAcceleration(0, 0, -0.1);
  PolynomialSplineQuinticVector3Curve resultCurve;
  double goalTime = 1.0;
  ROS_INFO_STREAM("start = " << start << "goal = " << goal);
  bool chompResult = chomp_.plan(start, goal, goalTime, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration, resultCurve);
  ROS_INFO_STREAM("chomp plan " << chompResult);
  if(chompResult){
    publishLineFromCurve(resultCurve);
  }
}

void TrajectoryPlannerRos::publishLine(std::vector<Eigen::Vector3d> trajectory)
{
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    for (Eigen::Vector3d point : trajectory){
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      marker.points.push_back(p);
    }
    // Set the color -- be sure to set alpha to something non-zero!
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker_goal -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    trajectoryPublisher_.publish(marker);
    return;
}

void TrajectoryPlannerRos::publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve){
    CValueType value;
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 100; i ++){
      curve.evaluate(value, i * 4.0 / 100.0);
      Eigen::Vector3d linePoint(value[0], value[1], value[2]);
      points.push_back(linePoint);
    }

    publishLine(points);

    return;
}

void TrajectoryPlannerRos::publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos){
    visualization_msgs::MarkerArray markerarray;
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker_start;
    marker_start.header.frame_id = "/odom";
    marker_start.header.stamp = ros::Time::now();

    marker_start.ns = "basic_shapes";
    marker_start.id = 0;
    marker_start.type = shape;
    marker_start.action = visualization_msgs::Marker::ADD;
    marker_start.pose.position.x = startPos.x();
    marker_start.pose.position.y = startPos.y();
    marker_start.pose.position.z = startPos.z();
    marker_start.pose.orientation.x = 0.0;
    marker_start.pose.orientation.y = 0.0;
    marker_start.pose.orientation.z = 0.0;
    marker_start.pose.orientation.w = 1.0;
    // Set the scale of the marker_start -- 1x1x1 here means 1m on a side
    marker_start.scale.x = 0.04;
    marker_start.scale.y = 0.04;
    marker_start.scale.z = 0.04;
    // Set the color -- be sure to set alpha to something non-zero!
    marker_start.color.r = 0.0f;
    marker_start.color.g = 1.0f;
    marker_start.color.b = 0.0f;
    marker_start.color.a = 1.0;
    marker_start.lifetime = ros::Duration();
    markerarray.markers.push_back(marker_start);

    visualization_msgs::Marker marker_goal;
    marker_goal.header.frame_id = "/odom";
    marker_goal.header.stamp = ros::Time::now();

    marker_goal.ns = "basic_shapes";
    marker_goal.id = 1;
    marker_goal.type = shape;
    marker_goal.action = visualization_msgs::Marker::ADD;
    marker_goal.pose.position.x = goalPos.x();
    marker_goal.pose.position.y = goalPos.y();
    marker_goal.pose.position.z = goalPos.z();
    marker_goal.pose.orientation.x = 0.0;
    marker_goal.pose.orientation.y = 0.0;
    marker_goal.pose.orientation.z = 0.0;
    marker_goal.pose.orientation.w = 1.0;
    // Set the scale of the marker_goal -- 1x1x1 here means 1m on a side
    marker_goal.scale.x = 0.04;
    marker_goal.scale.y = 0.04;
    marker_goal.scale.z = 0.04;
    // Set the color -- be sure to set alpha to something non-zero!
    marker_goal.color.r = 0.0f;
    marker_goal.color.g = 1.0f;
    marker_goal.color.b = 1.0f;
    marker_goal.color.a = 1.0;
    marker_goal.lifetime = ros::Duration();
    markerarray.markers.push_back(marker_goal);

    // Publish the marker
    markerPublisher_.publish(markerarray);
    return;
}

// void TrajectoryPlannerRos::goalPoseCallback(const geometry_msgs::PoseStamped& message)
// {
//   // TODO Check for frame.
//   MELO_INFO("========================================================");
//   MELO_INFO("Locomotion planner received new goal pose.");
// }



} /* namespace */
