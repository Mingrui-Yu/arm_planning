#ifndef ARM_PLANNING_PLANNING_REQUEST_H
#define ARM_PLANNING_PLANNING_REQUEST_H

#include <arm_planning/common_include.h>

namespace dual_arm_planning{

class PlanningRequest
{
public:
    PlanningRequest(){}

public:
    std::string arm_0_group_name_;
    std::string arm_1_group_name_;

    std::vector<double> arm_0_start_joint_pos_;
    std::vector<double> arm_1_start_joint_pos_;

    std::string goal_type_;
    std::vector<double> arm_0_goal_joint_pos_;
    std::vector<double> arm_1_goal_joint_pos_;
    geometry_msgs::Pose arm_0_goal_pose_;
    geometry_msgs::Pose arm_1_goal_pose_;

    geometry_msgs::Pose constraint_tcp_relative_pose_; // constraint of the arm_1 tcp pose in the arm_0 tcp frame

    bool b_visualize_start_goal_ = false;
    bool b_visualize_planning_process_ = false;
    bool b_visualize_res_path_ = false;
    

}; // end class

} // end namespace

#endif