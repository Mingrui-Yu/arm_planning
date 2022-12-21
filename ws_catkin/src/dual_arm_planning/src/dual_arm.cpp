#include "dual_arm_planning/dual_arm.h"


namespace dual_arm_planning{

// -------------------------------------------------------
DualArm::DualArm(
    const ros::NodeHandle& nh,
    const std::string &robot_description_name,
    const std::string &arm_0_group_name,
    const std::string &arm_1_group_name
): nh_(nh)
{
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
        new robot_model_loader::RobotModelLoader(robot_description_name)); // The RobotModelLoader should be kept around. Reference: https://github.com/ros-planning/moveit/issues/2979#issuecomment-984440339

    robot_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_loader_->getModel()));
    robot_state_->setToDefaultValues();

    arm_0_ = std::make_shared<arm_planning::Robot>(nh_, robot_state_, arm_0_group_name);
    arm_1_ = std::make_shared<arm_planning::Robot>(nh_, robot_state_, arm_1_group_name);
}


// -------------------------------------------------------
bool DualArm::checkDualArmJointNum(
    double dual_arm_joint_value_size
){
    bool valid = (dual_arm_joint_value_size == arm_0_->joint_num_ + arm_1_->joint_num_);
    ROS_ERROR_COND(valid == false, "The number of joints doesn't match the group name.");
    return valid;
}


// -------------------------------------------------------
void DualArm::splitTwoArmJointPos(
    const Eigen::VectorXd &dual_arm_joint_pos,
    Eigen::VectorXd &arm_0_joint_pos,
    Eigen::VectorXd &arm_1_joint_pos
){
    checkDualArmJointNum(dual_arm_joint_pos.size());

    arm_0_joint_pos = dual_arm_joint_pos.block(0, 0, arm_0_->joint_num_, 1);
    arm_1_joint_pos = dual_arm_joint_pos.block(arm_0_->joint_num_, 0, arm_1_->joint_num_, 1);
}



} // end namespace