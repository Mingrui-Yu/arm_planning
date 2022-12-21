#ifndef ARM_PLANNING_PLANNING_REQUEST_H
#define ARM_PLANNING_PLANNING_REQUEST_H

#include "arm_planning/common_include.h"

namespace arm_planning{

class PlanningRequest
{
public:
    PlanningRequest(){}

public:
    std::string group_name_;

    std::vector<double> start_joint_pos_;

    std::string goal_type_;
    std::vector<double> goal_joint_pos_;
    geometry_msgs::Pose goal_pose_;

    bool b_visualize_start_goal_ = false;
    bool b_visualize_planning_process_ = false;
    bool b_visualize_res_path_ = false;
    

}; // end class

} // end namespace

#endif