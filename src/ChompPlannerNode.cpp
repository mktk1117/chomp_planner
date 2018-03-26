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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>
#include "voxblox_ros/ros_params.h"
#include <random>

using namespace voxblox;

namespace chomp_planner {

ChompPlannerNode::ChompPlannerNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
    : nodeHandle_(nodeHandle),
      privateNodeHandle_(privateNodeHandle),
      chomp_(SimpleChompPlanner(privateNodeHandle))
{
  readParameters();
  trajectoryPublisher_ = privateNodeHandle_.advertise<visualization_msgs::Marker>( "line_marker", 0 );
  // markerPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
  // clickedPointSubscriber_ = nodeHandle_.subscribe("/clicked_point", 1,
  //                                            &ChompPlannerNode::clickedPointCallback, this);

  EsdfMap::Config esdfConfig = getEsdfMapConfigFromRosParam(privateNodeHandle_);
  esdfMap_.reset(new EsdfMap(esdfConfig));

  commandTrajectoryPublisher_ = nodeHandle_.advertise<trajectory_msgs::MultiDOFJointTrajectory>( "/firefly/command/trajectory", 0 );
  commandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>( "/firefly/command/pose", 0 );
  esdfMapSubcriber_ = privateNodeHandle_.subscribe("esdf_map", 1,
                                        &ChompPlannerNode::esdfMapCallback, this);
  odometrySubscriber_ = privateNodeHandle_.subscribe("odometry", 1,
                                        &ChompPlannerNode::odometryCallback, this);
  goalPoseSubscriber_ = privateNodeHandle_.subscribe("goal_pose", 1,
                                        &ChompPlannerNode::goalPoseCallback, this);

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
    chomp_.updateSignedDistanceField(esdfMap_);
    planTrajectory(startPosition_, goalPosition_);
    // publishAllUpdatedEsdfVoxels();
    // publishSlices();
  }
}

void ChompPlannerNode::odometryCallback(const nav_msgs::Odometry& msg) {
  startPosition_.x() = msg.pose.pose.position.x;
  startPosition_.y() = msg.pose.pose.position.y;
  startPosition_.z() = msg.pose.pose.position.z;
  startVelocity_.x() = msg.twist.twist.linear.x;
  startVelocity_.y() = msg.twist.twist.linear.y;
  startVelocity_.z() = msg.twist.twist.linear.z;
  Eigen::Quaterniond q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  currentYaw_ = euler[2];
  // ROS_INFO_STREAM("got odometry" << msg);
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
  if (!privateNodeHandle_.param<bool>("fit_curve", fitCurve_, false)) {
    ROS_ERROR("Could not load fit curve.");
  }
  if (!privateNodeHandle_.param<double>("velocity", velocity_, 1.0)) {
    ROS_ERROR("Could not load velocity.");
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
  double goalTime = diff.norm() / velocity_;
  // diff.z() = 0;
  // double d = diff.norm();
  // Eigen::Vector3d middle = (start + goal) / 2;
  PolynomialCurveVariable curveVariable;
  curveVariable.startPointWithTime = Eigen::Vector4d(start.x(), start.y(), start.z(), 0);
  curveVariable.goalPointWithTime = Eigen::Vector4d(goal.x(), goal.y(), goal.z(), goalTime);
  curveVariable.initialVelocity = startVelocity_;
  curveVariable.initialAcceleration = Eigen::Vector3d(0, 0, 0);
  curveVariable.finalVelocity = Eigen::Vector3d(0, 0, 0);
  curveVariable.finalAcceleration = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector4d trajectoryV = curveVariable.startPointWithTime - curveVariable.goalPointWithTime;
  double initialLength = trajectoryV.head(3).norm();

  PolynomialSplineQuinticVector3Curve resultCurve;
  chomp_.fitCurve(curveVariable, resultCurve);
  bool collision = chomp_.curveCollisionCheck(resultCurve);
  std::cout << "collision check result = " << collision << std::endl;;
  if (collision) {
    if (fitCurve_) {
      PolynomialSplineQuinticVector3Curve modifiedCurve;
      int chompResult = chomp_.chompFitFromCurve(curveVariable, resultCurve, modifiedCurve);
      publishLineFromCurve(modifiedCurve);
    }
    else{
      std::vector <Eigen::Vector4d> resultTrajectory;
      resultTrajectory = chomp_.chompFit(curveVariable);
      double trajectoryLength = chomp_.trajectoryEvaluation(resultTrajectory);
      std::cout << "chomp result trajectory length = " << trajectoryLength << std::endl;
      if (trajectoryLength > 0 and trajectoryLength < 2 * initialLength) {
        std::cout << "use chomp result" << std::endl;
        publishLine(resultTrajectory);
        publishCommandTrajectory(resultTrajectory);
      }
    }
  }
  else {
    double curveLength = chomp_.curveEvaluation(resultCurve);
    std::cout << "curve trajectory length = " << curveLength << std::endl;;
    if (curveLength > 0 and curveLength < 2 * initialLength) {
      std::cout << "use curve result" << std::endl;
      publishLineFromCurve(resultCurve);
      publishCommandTrajectory(resultCurve);
    }
  }


}
//
void ChompPlannerNode::publishCommandTrajectory(const std::vector<Eigen::Vector4d>& trajectory)
{
  trajectory_msgs::MultiDOFJointTrajectory trajectoryMsg;
  int i = 0;
  double dt = 0.5;
  if (trajectory.size() > 0) {
    double t = trajectory.back()[3] - trajectory.front()[3];
    dt = t / trajectory.size();
  }
  else {
    return;
  }
  std::cout << "dt = " << dt << std::endl;

  Eigen::Vector4d prevPoint = trajectory[0];
  Eigen::Quaterniond q = AngleAxisd(0, Vector3d::UnitX())
    * AngleAxisd(0, Vector3d::UnitY())
    * AngleAxisd(goalYaw_, Vector3d::UnitZ());
  for (Eigen::Vector4d point : trajectory){
    trajectory_msgs::MultiDOFJointTrajectoryPoint p;
    geometry_msgs::Transform transform;
    transform.translation.x = point.x();
    transform.translation.y = point.y();
    transform.translation.z = point.z();
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    p.transforms.push_back(transform);
    p.time_from_start = ros::Duration(point[3]);
    // Eigen::Vector3d velocity = ((point - prevPoint) / dt).head(3);
    // geometry_msgs::Twist twist;
    // twist.linear.x = velocity.x();
    // twist.linear.y = velocity.y();
    // twist.linear.z = velocity.z();
    // p.velocities.push_back(twist);
    trajectoryMsg.points.push_back(p);
    prevPoint = point;
    i++;
  }
  commandTrajectoryPublisher_.publish(trajectoryMsg);
}

// void ChompPlannerNode::publishCommandTrajectory(const std::vector<Eigen::Vector3d>& trajectory)
// {
//   double dt = 0.1;
//   if (trajectory.size() > 0) {
//     double t = trajectory.back()[3] - trajectory.front()[3];
//     dt = t / trajectory.size();
//   }
//   std::vector<Eigen::Vector4d> trajectoryWithTime;
//   double 
//   for (Eigen::Vector3d point : trajectory){
//     Eigen::Vector4d p;
//     p.head(3) = point;
//     p[3] = t;
//     trajectoryWithTime.push_back(
//   trajectory_msgs::MultiDOFJointTrajectory trajectoryMsg;
//   int i = 0;
//   double dt = 0.5;
//   if (trajectory.size() > 0)
//     dt = 10.0 / trajectory.size();
//   std::cout << "dt = " << dt << std::endl;
//
//   Eigen::Vector3d prevPoint = trajectory[0];
//   for (Eigen::Vector3d point : trajectory){
//     trajectory_msgs::MultiDOFJointTrajectoryPoint p;
//     geometry_msgs::Transform transform;
//     transform.translation.x = point.x();
//     transform.translation.y = point.y();
//     transform.translation.z = point.z();
//     p.transforms.push_back(transform);
//     std::cout << "i = " << i << std::endl;
//     p.time_from_start = ros::Duration((float)i * dt);
//     std::cout << "point = " << point << ", prevPoint = " << prevPoint << std::endl;
//     Eigen::Vector3d velocity = (point - prevPoint) / dt;
//     geometry_msgs::Twist twist;
//     twist.linear.x = velocity.x();
//     twist.linear.y = velocity.y();
//     twist.linear.z = velocity.z();
//     p.velocities.push_back(twist);
//     trajectoryMsg.points.push_back(p);
//     prevPoint = point;
//     i++;
//   }
//   commandTrajectoryPublisher_.publish(trajectoryMsg);
// }

void ChompPlannerNode::publishCommandTrajectory(const PolynomialSplineQuinticVector3Curve& curve){
  CValueType value;
  std::vector<Eigen::Vector4d> points;
  double startTime = curve.getMinTime();
  double endTime = curve.getMaxTime();
  double dt = (endTime - startTime) / 10.0;
  for (double t = startTime; t < endTime; t+=dt){
    curve.evaluate(value, t);
    Eigen::Vector4d linePoint(value[0], value[1], value[2], t);
    points.push_back(linePoint);
  }
  publishCommandTrajectory(points);

}
void ChompPlannerNode::publishLine(const std::vector<Eigen::Vector4d>& trajectory)
{
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    for (Eigen::Vector4d point : trajectory){
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
void ChompPlannerNode::publishLineFromCurve(const PolynomialSplineQuinticVector3Curve& curve){
  CValueType value;
  std::vector<Eigen::Vector4d> points;
  double startTime = curve.getMinTime();
  double endTime = curve.getMaxTime();
  for (double t = startTime; t < endTime; t+=0.01){
    curve.evaluate(value, t);
    Eigen::Vector4d linePoint(value[0], value[1], value[2], t);
    points.push_back(linePoint);
  }

  publishLine(points);

  return;
}
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
  // ROS_INFO("========================================================");
  // ROS_INFO("Chomp planner received new goal pose.");
  goalPosition_.x() = message.pose.position.x;
  goalPosition_.y() = message.pose.position.y;
  goalPosition_.z() = message.pose.position.z;
  Eigen::Quaterniond q;
  q.x() = message.pose.orientation.x;
  q.y() = message.pose.orientation.y;
  q.z() = message.pose.orientation.z;
  q.w() = message.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  goalYaw_ = euler[2];
  if ((goalPosition_ - startPosition_).norm() < 0.01) {
    if (fabs(goalYaw_ - currentYaw_) > 0.1) {
      commandPosePublisher_.publish(message);
    }
  }
  // ROS_INFO_STREAM("goalPosition = " << goalPosition_);
  // ROS_INFO_STREAM("goalyaw = " << goalYaw_);
}
//
//
//
} /* namespace */
