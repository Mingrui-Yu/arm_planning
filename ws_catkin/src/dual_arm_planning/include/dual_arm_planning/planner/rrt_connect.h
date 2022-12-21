#ifndef DUAL_ARM_PLANNING_RRTCONNECT_H
#define DUAL_ARM_PLANNING_RRTCONNECT_H

#include <arm_planning/common_include.h>

#include "dual_arm_planning/planning_request.h"
#include "dual_arm_planning/planning_response.h"
#include "dual_arm_planning/scene.h"
#include "dual_arm_planning/dual_arm.h"
#include "dual_arm_planning/visualize.h"
#include "dual_arm_planning/node.h"
#include "dual_arm_planning/planner/planner_base.h"


namespace dual_arm_planning{

class RRTConnect: public PlannerBase
{
public:
    typedef std::shared_ptr<RRTConnect> Ptr;

    RRTConnect(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "RRTConnect";



}; // end class


} // end namespace

#endif