#include "dual_arm_planning/dual_arm.h"


namespace dual_arm_planning{

// -------------------------------------------------------
DualArm::DualArm(
    const ros::NodeHandle& nh,
    const std::string &robot_description_name,
    const std::string &arm_0_group_name,
    const std::string &arm_1_group_name,
    const std::string &dual_arm_group_name
): nh_(nh)
{
    dual_arm_group_name_ = dual_arm_group_name;

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


// ------------------------------------------------------------
bool DualArm::dualArmTcpRandomIK(
    const Eigen::Isometry3d &arm_0_target_pose,
    const Eigen::Isometry3d &arm_1_target_pose,
    Eigen::VectorXd &result_arm_0_joint_pos,
    Eigen::VectorXd &result_arm_1_joint_pos
){  
    return dualArmEndEffectorIK(arm_0_->randomJointPos(), 
                                arm_1_->randomJointPos(),
                                arm_0_target_pose, 
                                arm_1_target_pose, 
                                result_arm_0_joint_pos,
                                result_arm_1_joint_pos);
}


// ------------------------------------------------------------
bool DualArm::dualArmTcpIK(
    const Eigen::VectorXd &ref_arm_0_joint_pos,
    const Eigen::VectorXd &ref_arm_1_joint_pos,
    const Eigen::Isometry3d &arm_0_target_pose,
    const Eigen::Isometry3d &arm_1_target_pose,
    Eigen::VectorXd &result_arm_0_joint_pos,
    Eigen::VectorXd &result_arm_1_joint_pos
){  
    return dualArmEndEffectorIK(ref_arm_0_joint_pos, 
                                ref_arm_1_joint_pos,
                                arm_0_->tcpInBaseToEEInBase(arm_0_target_pose), 
                                arm_1_->tcpInBaseToEEInBase(arm_1_target_pose), 
                                result_arm_0_joint_pos,
                                result_arm_1_joint_pos);
}


// ------------------------------------------------------------
bool DualArm::dualArmEndEffectorIK(
    const Eigen::VectorXd &ref_arm_0_joint_pos,
    const Eigen::VectorXd &ref_arm_1_joint_pos,
    const Eigen::Isometry3d &arm_0_target_pose,
    const Eigen::Isometry3d &arm_1_target_pose,
    Eigen::VectorXd &result_arm_0_joint_pos,
    Eigen::VectorXd &result_arm_1_joint_pos
){  
    arm_0_->checkArmJointNum(ref_arm_0_joint_pos.size());
    arm_1_->checkArmJointNum(ref_arm_1_joint_pos.size());

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(dual_arm_group_name_);

    // initial value for IK solving
    Eigen::VectorXd ref_dual_arm_joint_pos(ref_arm_0_joint_pos.size() + ref_arm_1_joint_pos.size());
    ref_dual_arm_joint_pos << ref_arm_0_joint_pos, ref_arm_1_joint_pos;

    robot_state_->setJointGroupPositions(
        joint_model_group, Utils::eigenVectorXd2StdVector(ref_dual_arm_joint_pos));

    // solve the IK problem
    double timeout = 0.1;
    moveit::core::GroupStateValidityCallbackFn ik_check_callback = 
        boost::bind(&DualArm::validateIKSolution, this, _1, _2, _3);

    EigenSTL::vector_Isometry3d target_poses{arm_0_target_pose, arm_1_target_pose};
    std::vector<std::string> tips{arm_0_->arm_ee_link_name_, arm_1_->arm_ee_link_name_};

    bool found_ik = robot_state_->setFromIK(joint_model_group, target_poses, 
        tips, timeout, ik_check_callback);

    // get the solution
    if(found_ik){
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
        splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(joint_values), result_arm_0_joint_pos, result_arm_1_joint_pos);
        return true;
    }
    else{
        return false;
    }
}


// -------------------------------------------------------
bool DualArm::validateIKSolution( 
    robot_state::RobotState* robot_state, 
    const robot_state::JointModelGroup* joint_group, 
    const double* joint_group_variable_value
){
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_value);

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_group, joint_values);

    Eigen::VectorXd arm_0_joint_pos, arm_1_joint_pos;
    splitTwoArmJointPos(Utils::stdVector2EigenVectorXd(joint_values), arm_0_joint_pos, arm_1_joint_pos);

    return arm_0_->checkJointPositionSatisfyBound(arm_0_joint_pos) && 
           arm_1_->checkJointPositionSatisfyBound(arm_1_joint_pos);
}



} // end namespace