/*
 * SimpleChompPlanner.hpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 */

#pragma once
#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include "voxblox_msgs/Layer.h"
// #include "voxblox_ros/tsdf_server.h"
#include "chomp_planner/ChompPlannerBase.hpp"

#include <Eigen/Core>

using namespace Eigen;
// using namespace curves;
// typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
// typedef typename curves::PolynomialSplineQuinticVector3Curve CurvePSQV3; 
// typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType CDerivativeType;
// typedef typename curves::Time Time;

namespace chomp_planner {

class SimpleChompPlanner : public ChompPlannerBase
{
  public:
    SimpleChompPlanner(const double collisionThreshold, const int numberOfPoints);
    // SimpleChompPlanner(double obstacleCostThreshold, double objectFunctionLambda, double dt);
    virtual ~SimpleChompPlanner();
    void updateSignedDistanceField(const std::shared_ptr<voxblox::EsdfMap>& esdfMap);

    // int fitFromCurve(CurvePSQV3& initialCurve, CurvePSQV3& resultCurve);

    std::vector<Vector3d> chompFit(std::vector<Vector3d>& trajectory);

    // bool plan(const Vector3d& startPosition, 
    //     const Vector3d& goalPosition, 
    //     const double& goalTime, 
    //     const CDerivativeType& initialVelocity, 
    //     const CDerivativeType& initialAcceleration, 
    //     const CDerivativeType& finalVelocity, 
    //     const CDerivativeType& finalAcceleration, 
    //     CurvePSQV3& resultCurve);


  private:
    // void publishStartGoalMarker(Eigen::Vector3d& startPos, Eigen::Vector3d& goalPos);
    // void publishLineFromCurve(PolynomialSplineQuinticVector3Curve curve);
    // void publishLine(std::vector<Eigen::Vector3d> trajectory);
    // Within this distance, the point considered to be collided
    double collisionThreshold_;
    // Obstacle cost of the point farther than this threshold becomes 0
    double obstacleCostThreshold_;
    // Parameter of the objective function
    double objectFunctionLambda_;
    // defines the number of points which is used in chomp optimization. 
    double numberOfPoints_;
    // CHOMP iteration finishes if the update becomes smaller than this value
    double iterationConvergenceThreshold_;
    // How much to update at each iteration
    double updateCoeff_;
    // Maximum number of CHOMP iteration
    int maxIteration_;

    // defines the time difference between the trajectory points
    double dt_;

    // Initial velocity and acceleration
    // CDerivativeType initialVelocity_;
    // CDerivativeType initialAcceleration_;
    // CDerivativeType finalVelocity_;
    // CDerivativeType finalAcceleration_;

    double getObstacleCost(Eigen::Vector3d x);
    double trajectoryEvaluation(std::vector<Vector3d>& trajectory);
    // bool curveCollisionCheck(CurvePSQV3& curve, std::vector<double>& collisionTimes);
    // double curveEvaluation(CurvePSQV3& curve);
    Eigen::Vector3d getObstacleCostGradient(Eigen::Vector3d x);

    std::shared_ptr<voxblox::EsdfMap> esdfMap_;

    Vector3d getSmoothFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObstacleFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObjectFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);

    // int fitCurveFromTrajectory(double startTime, double endTime, std::vector<Vector3d>& trajectory, CurvePSQV3& fittedCurve);
};

} /* namespace locomotion_planner */
