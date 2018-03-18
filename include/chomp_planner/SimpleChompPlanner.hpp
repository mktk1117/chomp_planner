/*
 * OneStepFootstepPlanner.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once
#include "grid_map_core/GridMap.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"

#include "locomotion_planner/trajectory_planner/ChompPlannerBase.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <Eigen/Core>

using namespace Eigen;
using namespace grid_map;
using namespace curves;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
typedef typename curves::PolynomialSplineQuinticVector3Curve CurvePSQV3; 
typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType CDerivativeType;
typedef typename curves::Time Time;

namespace locomotion_planner {

class SimpleChompPlanner : public ChompPlannerBase
{
  public:
    SimpleChompPlanner(ros::NodeHandle& nodeHandle, const double heightClearance, const double collisionThreshold, const int numberOfPoints);
    // SimpleChompPlanner(double obstacleCostThreshold, double objectFunctionLambda, double dt);
    virtual ~SimpleChompPlanner();
    void updateSignedDistanceField(GridMap& map, std::string layer, double clearanceHeight);
    void getSDFPointCloud(pcl::PointCloud<pcl::PointXYZI>& SDFpoints);
    void getSDFROSPointCloud(sensor_msgs::PointCloud2& pointcloud);
    int fitFromCurve(CurvePSQV3& initialCurve, CurvePSQV3& resultCurve);
    std::vector<Vector3d> chompFit(std::vector<Vector3d>& trajectory);
    bool plan(const Vector3d startPosition, const Vector3d goalPosition, const double goalTime, const CDerivativeType& initialVelocity, const CDerivativeType& initialAcceleration, const CDerivativeType& finalVelocity, const CDerivativeType& finalAcceleration, CurvePSQV3& resultCurve);

  private:
    ros::NodeHandle& nodeHandle_;
    ros::Publisher trajectoryPublisher_;
    ros::Publisher markerPublisher_;
    void publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos);
    void publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve);
    void publishLine(std::vector<Eigen::Vector3d> trajectory);
    // Within this distance, the point considered to be collided
    const double collisionThreshold_;
    // Obstacle cost of the point farther than this threshold becomes 0
    const double obstacleCostThreshold_;
    // Parameter of the objective function
    const double objectFunctionLambda_;
    // defines the number of points which is used in chomp optimization. 
    const double numberOfPoints_;
    // Height clearance of the trajectory
    const double heightClearance_;
    // CHOMP iteration finishes if the update becomes smaller than this value
    const double iterationConvergenceThreshold_;
    // How much to update at each iteration
    const double updateCoeff_;
    // Maximum number of CHOMP iteration
    const int maxIteration_;

    // defines the time difference between the trajectory points
    double dt_;

    // Initial velocity and acceleration
    CDerivativeType initialVelocity_;
    CDerivativeType initialAcceleration_;
    CDerivativeType finalVelocity_;
    CDerivativeType finalAcceleration_;

    SignedDistanceField sdf_;
    GridMap map_;

    std::string layer_;

    double getObstacleCost(Eigen::Vector3d x);
    double trajectoryEvaluation(std::vector<Vector3d>& trajectory);
    bool curveCollisionCheck(CurvePSQV3& curve, std::vector<double>& collisionTimes);
    double curveEvaluation(CurvePSQV3& curve);
    Eigen::Vector3d getObstacleCostGradient(Eigen::Vector3d x);

    Vector3d getSmoothFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObstacleFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObjectFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);

    int fitCurveFromTrajectory(double startTime, double endTime, std::vector<Vector3d>& trajectory, CurvePSQV3& fittedCurve);
};

} /* namespace locomotion_planner */
