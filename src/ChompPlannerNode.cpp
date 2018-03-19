/*
 * ChompPlannerNode.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 */

#include "chomp_planner/ChompPlannerNode.hpp"

// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "voxblox_ros/ros_params.h"
#include <random>

// typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
// typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType CDerivativeType;
// typedef typename curves::Time Time;
using namespace voxblox;

namespace chomp_planner {

      // updatedStartPoint_(false),
      // numberOfPoints_(30),
      // heightClearance_(0.1),
      // collisionThreshold_(0.005)
ChompPlannerNode::ChompPlannerNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
    : nodeHandle_(nodeHandle),
      privateNodeHandle_(privateNodeHandle),
      chomp_(SimpleChompPlanner(0.1, 100))
{
  readParameters();
  // chomp_ = SimpleChompPlanner(heightClearance_, collisionThreshold_, numberOfPoints_);
  trajectoryPublisher_ = privateNodeHandle_.advertise<visualization_msgs::Marker>( "line_marker", 0 );
  // markerPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  // clickedPointSubscriber_ = nodeHandle_.subscribe("/clicked_point", 1,
  //                                            &ChompPlannerNode::clickedPointCallback, this);

  // EsdfIntegrator::Config esdf_integrator_config =
  //     getEsdfIntegratorConfigFromRosParam(privateNodeHandle_);
  EsdfMap::Config esdfConfig = getEsdfMapConfigFromRosParam(privateNodeHandle_);
  esdfMap_.reset(new EsdfMap(esdfConfig));

  esdfMapSubcriber_ = nodeHandle_.subscribe("voxblox_node/esdf_map", 1,
                                        &ChompPlannerNode::esdfMapCallback, this);

}

ChompPlannerNode::~ChompPlannerNode()
{
}

void ChompPlannerNode::esdfMapCallback(const voxblox_msgs::LayerConstPtr layerMsg) {
  bool success =
      deserializeMsgToLayer<EsdfVoxel>(*layerMsg, esdfMap_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    double d;
    Eigen::Vector3d grad;
    esdfMap_->getDistanceAndGradientAtPosition(Eigen::Vector3d(0, 1, 1), &d, &grad);
    std::cout << "d at (0, 1, 1) = " << d << std::endl;
    std::cout << "grad at (0, 1, 1) = " << grad << std::endl;
    chomp_.updateSignedDistanceField(esdfMap_);
    planTrajectory(Eigen::Vector3d(1.0, 0, 0.0), Eigen::Vector3d(2.5, 0, 0.8));
    // publishAllUpdatedEsdfVoxels();
    // publishSlices();
  }
}

bool ChompPlannerNode::readParameters()
{
  if (!privateNodeHandle_.param<std::string>("base_frame_id", baseFrameId_, "body")) {
    ROS_ERROR("Could not load base frame id.");
  }
  if (!privateNodeHandle_.param<int>("number_of_points", numberOfPoints_, 100)) {
    ROS_ERROR("Could not load number of points.");
  }
  if (!privateNodeHandle_.param<double>("collision_threshold", collisionThreshold_, 0.05)) {
    ROS_ERROR("Could not load collision threshold.");
  }
}
//
//
// void ChompPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped& message)
// {
//   ROS_INFO_STREAM("recieved clicked point" << message);
//   if (updatedStartPoint_){
//     goalPosition_.x() = message.point.x;
//     goalPosition_.y() = message.point.y;
//     goalPosition_.z() = message.point.z;
//     updatedStartPoint_ = false;
//     publishStartGoalMarker(startPosition_, goalPosition_);
//     ROS_INFO("updated goal pose");
//     planTrajectory(startPosition_, goalPosition_);
//   }
//   else{
//     startPosition_.x() = message.point.x;
//     startPosition_.y() = message.point.y;
//     startPosition_.z() = message.point.z;
//     publishStartGoalMarker(startPosition_, goalPosition_);
//     updatedStartPoint_ = true;
//     ROS_INFO("updated start pose");
//   }
//   // Pose goalPose;
//
// }
//
void ChompPlannerNode::planTrajectory(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
{
  ROS_INFO("plan trajectory");
  Eigen::Vector3d diff = goal - start;
  // diff.z() = 0;
  // double d = diff.norm();
  // Eigen::Vector3d middle = (start + goal) / 2;
  // CDerivativeType initialVelocity(0, 0, 0);
  // CDerivativeType initialAcceleration(0, 0, 0);
  // CDerivativeType finalVelocity(0, 0, 0);
  // CDerivativeType finalAcceleration(0, 0, 0.0);
  // PolynomialSplineQuinticVector3Curve resultCurve;
  // double goalTime = 1.0;
  // ROS_INFO_STREAM("start = " << start << "goal = " << goal);
  std::cout << "start = " << start << "goal = " << goal << std::endl;
  std::vector <Eigen::Vector3d> initialTrajectory, resultTrajectory;
  std::random_device rnd;
  std::mt19937 gen(rnd());
  // std::uniform_real_distribution<double> rand(-0.5, 0.5);
  std::normal_distribution<> d(0, 0.1);
  for (int i = 0; i < numberOfPoints_; i++) {
    Eigen::Vector3d p = start + diff / numberOfPoints_ * i;
    if (i != 0 and i != numberOfPoints_ - 1) {
      p.x() += d(gen) * (i - numberOfPoints_ / 2.0) / numberOfPoints_ * 10;
      p.y() += d(gen) * (i - numberOfPoints_ / 2.0) / numberOfPoints_ * 10;
      p.z() += d(gen) * (i - numberOfPoints_ / 2.0) / numberOfPoints_ * 10;
    }
    std::cout << p << std::endl;
    initialTrajectory.push_back(p);
  }
  resultTrajectory = chomp_.chompFit(initialTrajectory);
  for (int i = 0; i < numberOfPoints_; i++) {
    // ROS_INFO_STREAM("chomp plan " << i << ": ", resultTrajectory);
    std::cout << "chomp plan " << i << ": " << resultTrajectory[i] << std::endl;
  }
  publishLine(resultTrajectory);
  // ROS_INFO_STREAM("chomp plan " << resultTrajectory);
  // if(chompResult){
  //   publishLineFromCurve(resultCurve);
  // }
}
//
void ChompPlannerNode::publishLine(std::vector<Eigen::Vector3d> trajectory)
{
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
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
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    trajectoryPublisher_.publish(marker);
    return;
}
//
// void ChompPlannerNode::publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve){
//     CValueType value;
//     std::vector<Eigen::Vector3d> points;
//     for (int i = 0; i < 100; i ++){
//       curve.evaluate(value, i * 4.0 / 100.0);
//       Eigen::Vector3d linePoint(value[0], value[1], value[2]);
//       points.push_back(linePoint);
//     }
//
//     publishLine(points);
//
//     return;
// }
//
// void ChompPlannerNode::publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos){
//     visualization_msgs::MarkerArray markerarray;
//     uint32_t shape = visualization_msgs::Marker::CUBE;
//
//     visualization_msgs::Marker marker_start;
//     marker_start.header.frame_id = "/odom";
//     marker_start.header.stamp = ros::Time::now();
//
//     marker_start.ns = "basic_shapes";
//     marker_start.id = 0;
//     marker_start.type = shape;
//     marker_start.action = visualization_msgs::Marker::ADD;
//     marker_start.pose.position.x = startPos.x();
//     marker_start.pose.position.y = startPos.y();
//     marker_start.pose.position.z = startPos.z();
//     marker_start.pose.orientation.x = 0.0;
//     marker_start.pose.orientation.y = 0.0;
//     marker_start.pose.orientation.z = 0.0;
//     marker_start.pose.orientation.w = 1.0;
//     // Set the scale of the marker_start -- 1x1x1 here means 1m on a side
//     marker_start.scale.x = 0.04;
//     marker_start.scale.y = 0.04;
//     marker_start.scale.z = 0.04;
//     // Set the color -- be sure to set alpha to something non-zero!
//     marker_start.color.r = 0.0f;
//     marker_start.color.g = 1.0f;
//     marker_start.color.b = 0.0f;
//     marker_start.color.a = 1.0;
//     marker_start.lifetime = ros::Duration();
//     markerarray.markers.push_back(marker_start);
//
//     visualization_msgs::Marker marker_goal;
//     marker_goal.header.frame_id = "/odom";
//     marker_goal.header.stamp = ros::Time::now();
//
//     marker_goal.ns = "basic_shapes";
//     marker_goal.id = 1;
//     marker_goal.type = shape;
//     marker_goal.action = visualization_msgs::Marker::ADD;
//     marker_goal.pose.position.x = goalPos.x();
//     marker_goal.pose.position.y = goalPos.y();
//     marker_goal.pose.position.z = goalPos.z();
//     marker_goal.pose.orientation.x = 0.0;
//     marker_goal.pose.orientation.y = 0.0;
//     marker_goal.pose.orientation.z = 0.0;
//     marker_goal.pose.orientation.w = 1.0;
//     // Set the scale of the marker_goal -- 1x1x1 here means 1m on a side
//     marker_goal.scale.x = 0.04;
//     marker_goal.scale.y = 0.04;
//     marker_goal.scale.z = 0.04;
//     // Set the color -- be sure to set alpha to something non-zero!
//     marker_goal.color.r = 0.0f;
//     marker_goal.color.g = 1.0f;
//     marker_goal.color.b = 1.0f;
//     marker_goal.color.a = 1.0;
//     marker_goal.lifetime = ros::Duration();
//     markerarray.markers.push_back(marker_goal);
//
//     // Publish the marker
//     markerPublisher_.publish(markerarray);
//     return;
// }
//
void ChompPlannerNode::goalPoseCallback(const geometry_msgs::PoseStamped& message)
{
  ROS_INFO("========================================================");
  ROS_INFO("Chomp planner received new goal pose.");
}
//
//
//
} /* namespace */
