/*
 * ChompPlannerBase.hpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 *
 */

#pragma once

namespace chomp_planner {

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

} /* namespace chomp_planner */
