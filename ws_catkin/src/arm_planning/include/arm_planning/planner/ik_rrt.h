#ifndef ARM_PLANNING_IKRRT_H
#define ARM_PLANNING_IKRRT_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class IKRRT: public PlannerBase
{
public:
    typedef std::shared_ptr<IKRRT> Ptr;

    IKRRT(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    void addRandomGoalIKNode(
        const Node::Ptr &goal_node,
        std::vector<Node::Ptr> &node_list_goal
    );

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "IKRRT";

    double new_goal_ik_probability_;



}; // end class


} // end namespace

#endif