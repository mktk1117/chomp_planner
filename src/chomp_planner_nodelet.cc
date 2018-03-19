#include "chomp_planner/ChompPlannerNode.hpp"
#include <nodelet/nodelet.h>

namespace chomp_planner_nodelet {

class ChompPlannerNodelet : public nodelet::Nodelet
{
private:
  chomp_planner::ChompPlannerNode *chompPlannerNode;

public:
  ChompPlannerNodelet() : Nodelet(), chompPlannerNode(NULL)
  {
  }

  ~ChompPlannerNodelet()
  {
    if(chompPlannerNode)
    {
      delete chompPlannerNode;
    }
  }

  virtual void onInit()
  {
    chompPlannerNode = new chomp_planner::ChompPlannerNode(getNodeHandle(), getPrivateNodeHandle());
  }
};
}  // namespace chomp_planner_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(chomp_planner_nodelet::ChompPlannerNodelet, nodelet::Nodelet)

