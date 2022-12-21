#ifndef ARM_PLANNING_RRT_H
#define ARM_PLANNING_RRT_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class RRT: public PlannerBase
{
public:
    typedef std::shared_ptr<RRT> Ptr;

    RRT(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();


    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "RRT";

    // parameters
    double goal_bias_probability_;



}; // end class


} // end namespace

#endif