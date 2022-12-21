#ifndef DUAL_ARM_PLANNING_PLANNER_INTERFACE_H
#define DUAL_ARM_PLANNING_PLANNER_INTERFACE_H

#include <arm_planning/common_include.h>

#include "dual_arm_planning/planning_request.h"
#include "dual_arm_planning/planning_response.h"

#include "dual_arm_planning/scene.h"
#include "dual_arm_planning/dual_arm.h"
#include "dual_arm_planning/visualize.h"




namespace dual_arm_planning{


class PlanningInterface
{
public:
    typedef std::shared_ptr<PlanningInterface> Ptr;

    PlanningInterface(
        const ros::NodeHandle& nh,
        const std::string &robot_description_name
    );

    bool solve(
        std::string algorithm,
        const PlanningRequest &req,
        PlanningResponse &res
    );




public: // protected

    ros::NodeHandle nh_;
    std::string robot_description_name_;

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

    Scene::Ptr scene_;
    DualArm::Ptr dual_arm_;
    Visualize::Ptr visualizer_;



}; // end class


} // end namespace

#endif