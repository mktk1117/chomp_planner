/*
 * SimpleChompPlanner.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 *
 */

#include <chomp_planner/SimpleChompPlanner.hpp>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using namespace curves;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType CValueType;
typedef typename curves::PolynomialSplineQuinticVector3Curve CurvePSQV3; 
typedef typename curves::Time Time;

namespace chomp_planner {

SimpleChompPlanner::SimpleChompPlanner(const ros::NodeHandle& nh) :
  objectFunctionLambda_(0.05),
  obstacleCostThreshold_(1.0),
  iterationConvergenceThreshold_(0.001),
  updateCoeff_(0.001),
  maxIteration_(4000),
  collisionThreshold_(0.05),
  dt_(0.1)
  {
    getRosParameters(nh);
  }

SimpleChompPlanner::~SimpleChompPlanner(){ }

bool SimpleChompPlanner::getRosParameters(const ros::NodeHandle& nh) {

  if (!nh.param<double>("object_function_lambda", objectFunctionLambda_, 0.05)) {
    ROS_WARN("Could not load object function lambda.");
  }
  if (!nh.param<double>("obstacle_cost_threshold", obstacleCostThreshold_, 1.0)) {
    ROS_WARN("Could not load obstacle cost threshold.");
  }
  if (!nh.param<double>("iteration_conversion_min", iterationConvergenceThreshold_, 0.001)) {
    ROS_WARN("Could not load iteration_conversion_min.");
  }
  if (!nh.param<double>("update_coeff", updateCoeff_, 0.001)) {
    ROS_WARN("Could not load update_coeff.");
  }
  if (!nh.param<int>("max_iteration", maxIteration_, 4000)) {
    ROS_WARN("Could not load max_iteration.");
  }
  if (!nh.param<double>("collision_threshold", collisionThreshold_, 0.05)) {
    ROS_WARN("Could not load collision threshold.");
  }
  if (!nh.param<double>("trajectory_dt", dt_, 0.1)) {
    ROS_WARN("Could not load trajectory dt.");
  }
  if (!nh.param<double>("chomp_initial_noise_variance", chompNoiseVariance_, 0.2)) {
    ROS_WARN("Could not load chomp_initial_noise_variance.");
  }
  if (!nh.param<int>("chomp_trial_iteration", chompTrialIteration_, 1)) {
    ROS_WARN("Could not load chomp_trial_iteration.");
  }
  if (!nh.param<bool>("chomp_trial_break", chompTrialBreak_, true)) {
    ROS_WARN("Could not load chomp_trial_break.");
  }
  if (!nh.param<double>("chomp_start_sphere_length", startSphereLength_, 0.4)) {
    ROS_WARN("Could not load chomp_start_sphere_length.");
  }
  if (!nh.param<double>("chomp_start_sphere_time", startSphereTime_, 1.0)) {
    ROS_WARN("Could not load chomp_start_sphere_time.");
  }
}



void SimpleChompPlanner::updateSignedDistanceField(const std::shared_ptr<voxblox::EsdfMap>& esdfMap){
  esdfMap_ = esdfMap;
}


double SimpleChompPlanner::trajectoryEvaluation(const std::vector<Vector4d>& trajectory){
  double length = 0;
  Vector3d startPosition = trajectory[0].head(3);
  Vector3d goalPosition = trajectory[trajectory.size() - 1].head(3);
  for (int i = 1; i < trajectory.size() - 1; i++){
    Eigen::Vector3d p = trajectory[i].head(3);
    Eigen::Vector3d p_prev = trajectory[i - 1].head(3);
    double d;
    esdfMap_->getDistanceAtPosition(p, &d);
    // cout << "trajectory evaluation: d at " << p << " = " << d << endl;
    if (d < collisionThreshold_ and (p - startPosition).norm() > startSphereLength_)
      return -1;
    length += (p - p_prev).norm();
  }
  return length;
}

double SimpleChompPlanner::trajectoryEvaluation(const std::vector<Vector3d>& trajectory){
  double length = 0;
  Vector3d startPosition = trajectory[0];
  Vector3d goalPosition = trajectory[trajectory.size() - 1];
  for (int i = 1; i < trajectory.size() - 1; i++){
    Eigen::Vector3d p = trajectory[i];
    Eigen::Vector3d p_prev = trajectory[i - 1];
    double d;
    esdfMap_->getDistanceAtPosition(p, &d);
    // cout << "trajectory evaluation: d at " << p << " = " << d << endl;
    if (d < collisionThreshold_ and (p - startPosition).norm() > startSphereLength_)
      return -1;
    length += (p - p_prev).norm();
  }
  return length;
}

std::vector<Vector4d> SimpleChompPlanner::chompFit(const PolynomialCurveVariable& curveVariable){
  std::vector <Eigen::Vector3d> initialTrajectory, chompResultTrajectory;
  std::vector <Eigen::Vector4d> resultTrajectory;
  std::vector <double> trajectoryTime;
  std::random_device rnd;
  std::mt19937 gen(rnd());
  std::normal_distribution<> d(0, chompNoiseVariance_);
  int numberOfPoints = (curveVariable.goalPointWithTime[3] - curveVariable.startPointWithTime[3]) / dt_;
  Eigen::Vector4d diffV = curveVariable.goalPointWithTime - curveVariable.startPointWithTime;
  // cout << "number of points = " << numberOfPoints << endl;
  // cout << "diffV = " << diffV << endl;
  for (int i = 0; i < numberOfPoints; i++) {
    Eigen::Vector4d point = curveVariable.startPointWithTime + diffV / numberOfPoints * i;
    Eigen::Vector3d p = point.head(3);
    // cout << "p = " << p << endl;
    if (i != 0 and i != numberOfPoints_ - 1) {
      p.x() += d(gen);
      p.y() += d(gen);
      p.z() += d(gen);
    }
    initialTrajectory.push_back(p);
    trajectoryTime.push_back(point[3]);
  }
  chompResultTrajectory = chompFit(initialTrajectory);
  for (int i = 0; i < chompResultTrajectory.size(); i++) {
    Eigen::Vector4d p;
    p.head(3) = chompResultTrajectory[i];
    p[3] = trajectoryTime[i];
    resultTrajectory.push_back(p);
  }
  return resultTrajectory;
}

std::vector<Vector3d> SimpleChompPlanner::chompFit(std::vector<Vector3d>& trajectory){
  for (int cnt = 0; cnt < maxIteration_; cnt++){
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
  double d;
  esdfMap_->getDistanceAtPosition(x, &d);
  if( d < 0)
    return -d + obstacleCostThreshold_ / 2.0;
  else if( d <= obstacleCostThreshold_)
    return 1.0 / (2 * obstacleCostThreshold_) * (d - obstacleCostThreshold_) * (d - obstacleCostThreshold_);
  else
    return 0;
}

Vector3d SimpleChompPlanner::getObstacleCostGradient(Vector3d x){
  double d;
  Vector3d gradient;
  esdfMap_->getDistanceAndGradientAtPosition(x, &d, &gradient);
  if( d < 0)
    return -gradient;
  else if( d <= obstacleCostThreshold_)
    return 1.0 / obstacleCostThreshold_ * (d - obstacleCostThreshold_) * gradient;
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
    if (!(std::isnan(k.x()) or std::isnan(k.y()) or std::isnan(k.z())))
      return x_dt.norm() * (I - x_dt_normalized * x_dt_normalized.transpose()) * c_grad - c * k;
    else
      return Vector3d(0, 0, 0);
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
  return getObstacleFunctionGradient(trajectory, trajectoryIndex) + 
    objectFunctionLambda_ * getSmoothFunctionGradient(trajectory, trajectoryIndex);
}


double SimpleChompPlanner::curveEvaluation(const CurvePSQV3& curve){
  std::vector<Vector3d> trajectory;
  double startTime = curve.getMinTime();
  double endTime = curve.getMaxTime();
  CValueType value;
  for (double t = startTime; t < endTime; t+=dt_){
    curve.evaluate(value, t);
    Vector3d p(value[0], value[1], value[2]);
    trajectory.push_back(p);
  }
  if (trajectory.size() > 0)
    return trajectoryEvaluation(trajectory);
  else
    return 0;
}

bool SimpleChompPlanner::curveCollisionCheck(const CurvePSQV3& curve, std::vector<double>& collisionTimes){
  // cout << "collision check" << endl;
  collisionTimes.clear();
  CValueType value;
  double startTime = curve.getMinTime();
  double endTime = curve.getMaxTime();
  for (double t = startTime; t < endTime; t+=dt_){
    curve.evaluate(value, t);
    Eigen::Vector3d p(value[0], value[1], value[2]);
    double d;
    esdfMap_->getDistanceAtPosition(p, &d);
    if (d < collisionThreshold_ and (t - startTime) > startSphereTime_)
      collisionTimes.push_back(t);
  }
  if(collisionTimes.size() > 0)
    return true;
  else
    return false;
}

bool SimpleChompPlanner::curveCollisionCheck(const CurvePSQV3& curve) {
  // cout << "collision check" << endl;
  double startSphereTime_ = 1.0;
  CValueType value;
  double startTime = curve.getMinTime();
  double endTime = curve.getMaxTime();
  for (double t = startTime; t < endTime; t+=0.001){
    curve.evaluate(value, t);
    Eigen::Vector3d p(value[0], value[1], value[2]);
    double d;
    esdfMap_->getDistanceAtPosition(p, &d);
    // cout << "curve collision " << p << " = " << d << endl;
    if (d < collisionThreshold_ and (t - startTime) > startSphereTime_)
      return true;
  }
  return false;
}

double SimpleChompPlanner::fitCurveFromTrajectory(const PolynomialCurveVariable& curveVariable, 
    const std::vector<Vector3d>& trajectory, CurvePSQV3& fittedCurve){
  static int maxCurveFitIteration_ = 10;

  CDerivativeType CinitialVelocity(curveVariable.initialVelocity);
  CDerivativeType CinitialAcceleration(curveVariable.initialAcceleration);
  CDerivativeType CfinalVelocity(curveVariable.finalVelocity);
  CDerivativeType CfinalAcceleration(curveVariable.finalAcceleration);

  std::vector<Vector4d> curvePointsWithTime;
  curvePointsWithTime.push_back(curveVariable.startPointWithTime);
  curvePointsWithTime.push_back(curveVariable.goalPointWithTime);

  double endTime = curveVariable.goalPointWithTime[3];
  double startTime = curveVariable.startPointWithTime[3];
  double dt = (endTime - startTime) / trajectory.size();

  for (int i = 0; i < maxCurveFitIteration_; i++){
    PolynomialSplineQuinticVector3Curve curve;
    std::vector<curves::Time> times;
    std::vector<CValueType> values;
    for (Vector4d p : curvePointsWithTime){
      times.push_back(p[3]);
      values.push_back(CValueType(p.x(), p.y(), p.z()));
    }
    curve.fitCurve(times, values, CinitialVelocity, CinitialAcceleration, CfinalVelocity, CfinalAcceleration);
    std::vector<double> collisionTimes;
    if (curveCollisionCheck(curve, collisionTimes)){
      int prevIndex = 0;
      for (double t : collisionTimes){
        int collidedIndexInTrajectory = std::round((t - startTime) / dt);
        if (collidedIndexInTrajectory - prevIndex > 1){
          Vector4d pt;
          pt.head(3) = trajectory[collidedIndexInTrajectory];
          pt[3] = t;
          int j = 0;
          cout << "trajector added " << pt << endl;
          for(auto p : curvePointsWithTime) {
            if (t < p[3])
              curvePointsWithTime.insert(curvePointsWithTime.begin() + j, pt);
            j++;
          }
        }
        prevIndex = collidedIndexInTrajectory;
      }
    }
    else{
      fittedCurve = curve;
      return curveEvaluation(fittedCurve);
    }
  }
  return -1;
}

double SimpleChompPlanner::chompFitFromCurve(const PolynomialCurveVariable& curveVariable, 
    const CurvePSQV3& initialCurve, CurvePSQV3& resultCurve){
    CValueType value;
    double startTime = initialCurve.getMinTime();
    double endTime = initialCurve.getMaxTime();
    double minTrajectoryLength = std::numeric_limits<float>::max();
    CurvePSQV3 minCurve, chompResultCurve;
    for (int i = 0; i < chompTrialIteration_; i++) {
      std::vector<Vector3d> trajectory, resultTrajectory;
      std::random_device rnd;
      std::mt19937 gen(rnd());
      std::normal_distribution<> d(0, chompNoiseVariance_);
      for (double t = startTime; t < endTime; t+=dt_){
        initialCurve.evaluate(value, t);
        Eigen::Vector3d p(value[0], value[1], value[2]);
        if (t > startTime and t < endTime - dt_) {
          p.x() += d(gen);
          p.y() += d(gen);
          p.z() += d(gen);
        }
        trajectory.push_back(p);
      }
      resultTrajectory = chompFit(trajectory);
      // cout << "chomp fit done" << endl;
      double trajectoryLength = trajectoryEvaluation(resultTrajectory);
      if (trajectoryEvaluation(resultTrajectory) < 0) {
        // cout << "chomp fit has collision" << endl;
        // return -1;
      }
      else {
        trajectoryLength = fitCurveFromTrajectory(curveVariable, resultTrajectory, chompResultCurve);
        if (trajectoryLength > 0) {
          if (chompTrialBreak_) {
            resultCurve = chompResultCurve;
            return trajectoryLength;
          }
          if (trajectoryLength < minTrajectoryLength) {
            minTrajectoryLength = trajectoryLength;
            minCurve = chompResultCurve;
          }
        }
      }
    }
    if (minTrajectoryLength < std::numeric_limits<float>::max()){
      resultCurve = minCurve;
      return minTrajectoryLength;
    }
    else
      return -1;
}

int SimpleChompPlanner::fitCurve(const PolynomialCurveVariable curveVariable, CurvePSQV3& resultCurve) {
  CDerivativeType CinitialVelocity(curveVariable.initialVelocity);
  CDerivativeType CinitialAcceleration(curveVariable.initialAcceleration);
  CDerivativeType CfinalVelocity(curveVariable.finalVelocity);
  CDerivativeType CfinalAcceleration(curveVariable.finalAcceleration);

  std::vector<Vector4d> curvePointsWithTime;
  curvePointsWithTime.push_back(curveVariable.startPointWithTime);
  curvePointsWithTime.push_back(curveVariable.goalPointWithTime);
  std::vector<curves::Time> times;
  std::vector<CValueType> values;
  for (Vector4d p : curvePointsWithTime){
    times.push_back(p[3]);
    values.push_back(CValueType(p.x(), p.y(), p.z()));
  }
  resultCurve.fitCurve(times, values, CinitialVelocity, CinitialAcceleration, CfinalVelocity, CfinalAcceleration);
}


} /* namespace locomotion_planner */
