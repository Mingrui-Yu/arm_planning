#ifndef ARM_PLANNING_PLANNER_INTERFACE_H
#define ARM_PLANNING_PLANNER_INTERFACE_H

#include "arm_planning/common_include.h"

#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"

#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"




namespace arm_planning{


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
    Robot::Ptr robot_;
    Visualize::Ptr visualizer_;



}; // end class


} // end namespace

#endif