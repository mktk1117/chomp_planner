/*
 * OneStepFootstepPlanner.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <visualization_msgs/MarkerArray.h>
#include <locomotion_planner/trajectory_planner/SimpleChompPlanner.hpp>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;
using namespace Eigen;
using namespace curves;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
typedef typename curves::PolynomialSplineQuinticVector3Curve CurvePSQV3; 
typedef typename curves::Time Time;

namespace locomotion_planner {

    SimpleChompPlanner::SimpleChompPlanner(ros::NodeHandle& nodeHandle, const double heightClearance, const double collisionThreshold, const int numberOfPoints) :
    nodeHandle_(nodeHandle),
    objectFunctionLambda_(0.1),
    obstacleCostThreshold_(0.1),
    iterationConvergenceThreshold_(0.001),
    updateCoeff_(0.001),
    maxIteration_(1000),
    heightClearance_(heightClearance),
    collisionThreshold_(collisionThreshold),
    numberOfPoints_(numberOfPoints)
    {
      trajectoryPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>( "chomp_trajectory", 0 );
      markerPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>( "chomp_marker", 0 );
      
    }

  SimpleChompPlanner::~SimpleChompPlanner(){ }


  void SimpleChompPlanner::updateSignedDistanceField(GridMap& map, std::string layer, double heightMargin){
    sdf_.calculateSignedDistanceField(map, layer, heightClearance_ + heightMargin);
    map_ = map;
    layer_ = layer;
  }

  void SimpleChompPlanner::getSDFPointCloud(pcl::PointCloud<pcl::PointXYZI>& SDFpoints)
  {
    sdf_.convertToPointCloud(SDFpoints);
  }

  void SimpleChompPlanner::getSDFROSPointCloud(sensor_msgs::PointCloud2& pointcloud){
    pcl::PointCloud<pcl::PointXYZI> SDFpoints;
    sdf_.convertToPointCloud(SDFpoints);
    // for (int i = 0; i < SDFpoints.size(); i++)
    //   std::cout << SDFpoints[i] << std::endl;
    pcl::toROSMsg(SDFpoints, pointcloud);
    pointcloud.header.stamp = ros::Time::now();
    pointcloud.header.frame_id = "odom";
  }


  bool SimpleChompPlanner::plan(const Vector3d startPosition, const Vector3d goalPosition, const double goalTime, const CDerivativeType& initialVelocity, const CDerivativeType& initialAcceleration, const CDerivativeType& finalVelocity, const CDerivativeType& finalAcceleration, CurvePSQV3& resultCurve){
    initialVelocity_ = initialVelocity;
    initialAcceleration_ = initialAcceleration;
    finalVelocity_ = finalVelocity;
    finalAcceleration_ = finalAcceleration;
    dt_ = goalTime / numberOfPoints_;

    Vector3d difference = startPosition - goalPosition;
    difference.z() = 0;
    double d = difference.norm();
    double pertubationLength_ = 0.05 * d;
    int pertubationNumber_ = 10;
    double resolution = map_.getResolution();
    Position mapPosition = map_.getPosition();
    // double y_length = map.getLength().y();
    // double y_size = map.getSize()(1);
    // double resolution = map.getResolution();
    // Position mapPosition = map.getPosition();
    double minLength = 10000;
    int minTrial = 10000;
    CurvePSQV3 minCurve;
    for(int i = 0; i < pertubationNumber_; i++){
      for(int j = 0; j < pertubationNumber_; j++){
        Vector3d middlePosition = (startPosition + goalPosition) / 2;
        cout << "initial middle_pos = " << middlePosition << endl;
        // pertubation
        middlePosition.x() = middlePosition.x() + (i - pertubationNumber_ / 2.0) * pertubationLength_;
        middlePosition.y() = middlePosition.y() + (j - pertubationNumber_ / 2.0) * pertubationLength_;
        if (fabs(middlePosition.x() - mapPosition.x()) > map_.getLength().x()) continue;
        if (fabs(middlePosition.y() - mapPosition.y()) > map_.getLength().y()) continue;
        double middleZ = map_.atPosition(layer_, Position(middlePosition.x(), middlePosition.y()));
        cout << "map at position " << middlePosition.x() << ", " << middlePosition.y() << " = " << middleZ << endl;
        if(std::isnan(middleZ))
          middleZ = std::max(startPosition.z(), goalPosition.z());
        middleZ = std::max(std::max(startPosition.z(), goalPosition.z()), middleZ);
        middlePosition.z() = middleZ + heightClearance_;

        // double initial_length = (start_pos - middle_pos).norm() + (middle_pos - goal_pos).norm();
        // int number_of_points = initial_length / resolution + 1;
        cout << "--------------------------doing pertubation ----------------------------" << endl;
        cout << "middle_pos = " << middlePosition << endl;

        CurvePSQV3 curve, result;
        std::vector<curves::Time> times;
        std::vector<CValueType> values;
        //
        times.push_back(0.0);
        values.push_back(CValueType(startPosition.x(), startPosition.y(), startPosition.z()));
        times.push_back(goalTime / 2);
        values.push_back(CValueType(middlePosition.x(), middlePosition.y(), middlePosition.z()));
        times.push_back(goalTime);
        values.push_back(CValueType(goalPosition.x(), goalPosition.y(), goalPosition.z()));
       
        curve.fitCurve(times, values, initialVelocity_, initialAcceleration_, finalVelocity_, finalAcceleration_);

        int fitTrial = fitFromCurve(curve, result);
        if(fitTrial >= 0){
          cout << "fit from Curve succeeded" << endl;
          if (fitTrial <= minTrial){
            minTrial = fitTrial;
            double l = curveEvaluation(result);
            cout << "trajectory length = " << l << endl;
            if (l > 0 && l < minLength){
              minLength = l;
              minCurve = result;
            }
          }
        }
        else{
          cout << "fit from Curve failed" << endl;
          double l = curveEvaluation(curve);
          cout << "trajectory length = " << l << endl;
          if (l > 0 && l < minLength){
            minLength = l;
            minCurve = curve;
          }
        }

        // publish_line(trajectory);
        // ros::Rate rate(1000);
        // rate.sleep();
      }
    }
    cout << "min_trajectory length = " << minLength << endl;
    if (minLength == 10000)
      return false;
    else{
      resultCurve = minCurve;
      return true;
    }
  }

  double SimpleChompPlanner::trajectoryEvaluation(std::vector<Vector3d>& trajectory){
    double length = 0;
    Vector3d startPosition = trajectory[0];
    Vector3d goalPosition = trajectory[trajectory.size() - 1];
    for (int i = 1; i < trajectory.size() - 1; i++){
      Eigen::Vector3d p = trajectory[i];
      Eigen::Vector3d p_prev = trajectory[i - 1];
      double d = sdf_.getInterpolatedDistanceAt(p);
      if ((p - startPosition).norm() > collisionThreshold_ && (p - goalPosition).norm() > collisionThreshold_){
        if (d < collisionThreshold_)
          return -1;
      }
      length += (p - p_prev).norm();
    }
    return length;
  }

  double SimpleChompPlanner::curveEvaluation(CurvePSQV3& curve){
    std::vector<Vector3d> trajectory;
    double startTime = curve.getMinTime();
    double endTime = curve.getMaxTime();
    CValueType value;
    for (double t = startTime; t < endTime; t+=dt_){
      curve.evaluate(value, t);
      Vector3d p(value[0], value[1], value[2]);
      trajectory.push_back(p);
    }
    return trajectoryEvaluation(trajectory);
  }

  bool SimpleChompPlanner::curveCollisionCheck(CurvePSQV3& curve, std::vector<double>& collisionTimes){
    // cout << "collision check" << endl;
    collisionTimes.clear();
    CValueType value;
    double startTime = curve.getMinTime();
    double endTime = curve.getMaxTime();
    curve.evaluate(value, startTime);
    Eigen::Vector3d startPosition(value[0], value[1], value[2]);
    curve.evaluate(value, endTime);
    Eigen::Vector3d endPosition(value[0], value[1], value[2]);
    for (double t = startTime; t < endTime; t+=0.001){
      curve.evaluate(value, t);
      Eigen::Vector3d p(value[0], value[1], value[2]);
      if ((p - startPosition).norm() > collisionThreshold_ && (p - endPosition).norm() > collisionThreshold_){
      // cout << "p = " << p << "d = " << sdf_.getInterpolatedDistanceAt(p) << endl;
        if (sdf_.getInterpolatedDistanceAt(p) < collisionThreshold_)
          collisionTimes.push_back(t);
      }
    }
    if(collisionTimes.size() > 0)
      return true;
    else
      return false;
  }

  int SimpleChompPlanner::fitCurveFromTrajectory(double startTime, double endTime, std::vector<Vector3d>& trajectory, CurvePSQV3& fittedCurve){
    int maxCurveFitIteration_ = 10;
    std::vector<Vector4d> curvePointsWithTime;
    Vector4d startPoint, middlePoint, endPoint;
    Vector4d quaterPoint, quater3Point;

    double dt = (endTime - startTime) / trajectory.size();

    startPoint.head(3) = trajectory.front();
    middlePoint.head(3) = trajectory[trajectory.size() / 2];
    quaterPoint.head(3) = trajectory[trajectory.size() / 5];
    quater3Point.head(3) = trajectory[trajectory.size() * 3 / 4];
    endPoint.head(3) = trajectory.back();
    startPoint[3] = startTime;
    middlePoint[3] = (startTime + endTime) / 2;
    quaterPoint[3] = (startTime + endTime) / 5;
    quater3Point[3] = (startTime + endTime) * 3 / 4;
    endPoint[3] = endTime;

    curvePointsWithTime.push_back(startPoint);
    // curvePointsWithTime.push_back(quaterPoint);
    curvePointsWithTime.push_back(middlePoint);
    // curvePointsWithTime.push_back(quater3Point);
    curvePointsWithTime.push_back(endPoint);
    for (int i = 0; i < maxCurveFitIteration_; i++){
      cout << "line fit trial" << i << endl;
      PolynomialSplineQuinticVector3Curve curve;
      std::vector<curves::Time> times;
      std::vector<CValueType> values;
      for (Vector4d p : curvePointsWithTime){
        times.push_back(p[3]);
        values.push_back(CValueType(p.x(), p.y(), p.z()));
      }
      curve.fitCurve(times, values, initialVelocity_, initialAcceleration_, finalVelocity_, finalAcceleration_);
      std::vector<double> collisionTimes;
      if (curveCollisionCheck(curve, collisionTimes)){
        cout << "There is collision!" << endl;
        int prevIndex = 0;
        for (double t : collisionTimes){
          // cout << "collision time" << t  << endl;
          int collidedIndexInTrajectory = std::round((t - startTime) / dt);
          // cout << "collision index" << collidedIndexInTrajectory << endl;
          if (collidedIndexInTrajectory - prevIndex > 1){
            Vector4d pt;
            pt.head(3) = trajectory[collidedIndexInTrajectory];
            pt[3] = t;
            // cout << "collision point add" << pt << endl;
            curvePointsWithTime.push_back(pt);
          }
          prevIndex = collidedIndexInTrajectory;
        }
      }
      else{
        cout << "curve is fitted without collision" << endl;
        fittedCurve = curve;
        return i;
      }
    }
    // If no collision-free curve is generated
    return -1;
  }

  int SimpleChompPlanner::fitFromCurve(CurvePSQV3& initialCurve, CurvePSQV3& resultCurve){
      CValueType value;
      std::vector<Vector3d> trajectory;
      double startTime = initialCurve.getMinTime();
      double endTime = initialCurve.getMaxTime();
      for (double t = startTime; t < endTime; t+=dt_){
        initialCurve.evaluate(value, t);
        Eigen::Vector3d p(value[0], value[1], value[2]);
        trajectory.push_back(p);
        // cout << "trajectory from curve" << p << endl;
      }
      trajectory = chompFit(trajectory);
      // cout << "chomp fit done" << endl;
      if (trajectoryEvaluation(trajectory) < 0)
        return -1;
      // cout << "fit trajectory with curve" << trajectory << endl;
      return fitCurveFromTrajectory(startTime, endTime, trajectory, resultCurve);
  }

  std::vector<Vector3d> SimpleChompPlanner::chompFit(std::vector<Vector3d>& trajectory){
      for (int cnt = 0; cnt < maxIteration_; cnt++){
        // cout << "iteration " << cnt << endl;
        double diff = 0;
        for (int j = 0; j < trajectory.size(); j++){
          Eigen::Vector3d delta = updateCoeff_ * getObjectFunctionGradient(trajectory, j);
          trajectory[j] -= delta;
          diff += delta.norm();
        }
        if( diff < iterationConvergenceThreshold_)
          break;
      }
      return trajectory;
  }

  double SimpleChompPlanner::getObstacleCost(Vector3d x){
    double d = sdf_.getInterpolatedDistanceAt(x);
    if( d < 0)
      return -d + obstacleCostThreshold_ / 2.0;
    else if( d <= obstacleCostThreshold_)
      return 1.0 / (2 * obstacleCostThreshold_) * (d - obstacleCostThreshold_) * (d - obstacleCostThreshold_);
    else
      return 0;
  }

  Vector3d SimpleChompPlanner::getObstacleCostGradient(Vector3d x){
    double d = sdf_.getInterpolatedDistanceAt(x);
    if( d < 0)
      return -sdf_.getDistanceGradientAt(x);
    else if( d <= obstacleCostThreshold_)
      return 1.0 / obstacleCostThreshold_ * (d - obstacleCostThreshold_) * sdf_.getDistanceGradientAt(x);
    else
      return Vector3d(0, 0, 0);
  }

  Vector3d SimpleChompPlanner::getObstacleFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex){
    int j = trajectoryIndex;

    if( j > 0 && j < trajectory.size() - 1){
      double c = getObstacleCost(trajectory[j]);
      Vector3d x_dt, x_dt2, x_dt_normalized, c_grad, k;
      Matrix3d I = Matrix3d::Identity();
      x_dt = (trajectory[j] - trajectory[j - 1]) / dt_;
      x_dt2 = (trajectory[j + 1] - 2 * trajectory[j] + trajectory[j - 1]) / (dt_ * dt_);
      x_dt_normalized = x_dt / x_dt.norm();
      c_grad = getObstacleCostGradient(trajectory[j]);
      k = 1.0 / x_dt.norm() * (I - x_dt_normalized * x_dt_normalized.transpose()) * x_dt2;
      return x_dt.norm() * (I - x_dt_normalized * x_dt_normalized.transpose()) * c_grad - c * k;
    }
    else
      return Vector3d(0, 0, 0);
  }

  Vector3d SimpleChompPlanner::getSmoothFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex){
    int j = trajectoryIndex;
    if( j > 0 && j < trajectory.size() - 1){
      Vector3d x_dt2 = (trajectory[j + 1] - 2 * trajectory[j] + trajectory[j - 1]) / (dt_ * dt_);
      return -x_dt2;
    }
    else
      return Vector3d(0, 0, 0);
  }

  Vector3d SimpleChompPlanner::getObjectFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex){
    return getObstacleFunctionGradient(trajectory, trajectoryIndex) + objectFunctionLambda_ * getSmoothFunctionGradient(trajectory, trajectoryIndex);
  }

  void SimpleChompPlanner::publishLine(std::vector<Eigen::Vector3d> trajectory)
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

  void SimpleChompPlanner::publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve){
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

  void SimpleChompPlanner::publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos){
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


} /* namespace locomotion_planner */
