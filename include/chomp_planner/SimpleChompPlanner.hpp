/*
 * SimpleChompPlanner.hpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 */

#pragma once
#include <voxblox/core/esdf_map.h>
// #include <voxblox/integrator/esdf_integrator.h>
#include "voxblox_msgs/Layer.h"
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"
#include "chomp_planner/ChompPlannerBase.hpp"
#include <ros/ros.h>

#include <Eigen/Core>

using namespace Eigen;
using namespace curves;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
typedef typename curves::PolynomialSplineQuinticVector3Curve CurvePSQV3; 
typedef typename curves::PolynomialSplineQuinticVector3Curve::DerivativeType CDerivativeType;
typedef typename curves::Time Time;

namespace chomp_planner {

struct PolynomialCurveVariable {
  Eigen::Vector4d startPointWithTime;
  Eigen::Vector4d goalPointWithTime;
  Eigen::Vector3d initialVelocity;
  Eigen::Vector3d initialAcceleration;
  Eigen::Vector3d finalVelocity;
  Eigen::Vector3d finalAcceleration;
};

class SimpleChompPlanner : public ChompPlannerBase
{
  public:
    SimpleChompPlanner(const ros::NodeHandle& nh);
    virtual ~SimpleChompPlanner();
    void updateSignedDistanceField(const std::shared_ptr<voxblox::EsdfMap>& esdfMap);

    double chompFitFromCurve(const PolynomialCurveVariable& curveVariable, 
        const CurvePSQV3& initialCurve, CurvePSQV3& resultCurve);

    std::vector<Vector3d> chompFit(std::vector<Vector3d>& trajectory);
    std::vector<Vector4d> chompFit(const PolynomialCurveVariable& curveVariable);
    double trajectoryEvaluation(const std::vector<Vector3d>& trajectory);
    double trajectoryEvaluation(const std::vector<Vector4d>& trajectory);
    double curveEvaluation(const CurvePSQV3& curve);

    bool curveCollisionCheck(const CurvePSQV3& curve);
    int fitCurve(const PolynomialCurveVariable curveVariable, CurvePSQV3& resultCurve);


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
    int numberOfPoints_;
    // CHOMP iteration finishes if the update becomes smaller than this value
    double iterationConvergenceThreshold_;
    // How much to update at each iteration
    double updateCoeff_;
    // Maximum number of CHOMP iteration
    int maxIteration_;

    double chompNoiseVariance_;
    int chompTrialIteration_;
    bool chompTrialBreak_;
    double startSphereLength_;
    double startSphereTime_;


    // defines the time difference between the trajectory points
    double dt_;

    bool getRosParameters(const ros::NodeHandle& nh);

    double getObstacleCost(Eigen::Vector3d x);
    Eigen::Vector3d getObstacleCostGradient(Eigen::Vector3d x);

    std::shared_ptr<voxblox::EsdfMap> esdfMap_;

    Vector3d getSmoothFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObstacleFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);
    Vector3d getObjectFunctionGradient(std::vector<Vector3d>& trajectory, int trajectoryIndex);

    bool curveCollisionCheck(const CurvePSQV3& curve, std::vector<double>& collisionTimes);
    // bool curveCollisionCheck(const CurvePSQV3& curve);
    double fitCurveFromTrajectory(const PolynomialCurveVariable& curveVariable, 
    const std::vector<Vector3d>& trajectory, CurvePSQV3& fittedCurve);
    // int fitCurve(
    //     const Vector4d& startPointWithTime, 
    //     const Vector4d& goalPointWithTime, 
    //     const Vector3d& initialVelocity, 
    //     const Vector3d& initialAcceleration,
    //     const Vector3d& finalVelocity, 
    //     const Vector3d& finalAcceleration,
    //     CurvePSQV3& resultCurve);
};

} /* namespace locomotion_planner */
