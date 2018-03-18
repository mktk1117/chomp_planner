/*
 * ChompPlannerBase.hpp
 *
 *  Created on: Aug 17, 2017
 *      Author: Takahiro Miki
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"

#include <Eigen/Core>

namespace locomotion_planner {

class ChompPlannerBase
{
  public:
    ChompPlannerBase();
    virtual ~ChompPlannerBase();

  protected:
    // virtual double getObstacleCost(grid_map_sdf::SignedDistanceField& sdf, Eigen::Vector3f x) = 0;
    // virtual Eigen::Vector3f getObstacleCostGradient(grid_map_sdf::SignedDistanceField& sdf, Eigen::Vector3f x) = 0;
    // virtual Eigen::VectorXf getSmoothFunctionGradient(std::vector<Eigen::VectorXf> trajectory, int trajectoryIndex) = 0;
    // virtual Eigen::VectorXf getObstacleFunctionGradient(grid_map_sdf::SignedDistanceField& sdf, std::vector<Eigen::VectorXf>& trajectory, int trajectoryIndex) = 0;
};

} /* namespace locomotion_planner */
