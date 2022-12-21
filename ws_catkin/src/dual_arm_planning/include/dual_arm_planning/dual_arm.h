#ifndef DUAL_ARM_PLANNING_DUAL_ARM_H
#define DUAL_ARM_PLANNING_DUAL_ARM_H


#include <arm_planning/robot.h>
#include <arm_planning/common_include.h>



namespace dual_arm_planning{

class DualArm
{
public:
    typedef std::shared_ptr<DualArm> Ptr;

    DualArm(
        const ros::NodeHandle& nh,
        const std::string &robot_description_name,
        const std::string &arm_0_group_name,
        const std::string &arm_1_group_name
    );

    bool checkDualArmJointNum(
        double dual_arm_joint_value_size
    );

    void splitTwoArmJointPos(
        const Eigen::VectorXd &dual_arm_joint_pos,
        Eigen::VectorXd &arm_0_joint_pos,
        Eigen::VectorXd &arm_1_joint_pos
    );

public:
    ros::NodeHandle nh_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotStatePtr robot_state_;
    
    arm_planning::Robot::Ptr arm_0_, arm_1_;


}; // end class



} // end namespace

#endif