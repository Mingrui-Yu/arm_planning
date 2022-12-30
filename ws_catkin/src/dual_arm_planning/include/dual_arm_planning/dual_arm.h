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
        const std::string &arm_1_group_name,
        const std::string &dual_arm_group_name
    );

    bool checkDualArmJointNum(
        double dual_arm_joint_value_size
    );

    void splitTwoArmJointPos(
        const Eigen::VectorXd &dual_arm_joint_pos,
        Eigen::VectorXd &arm_0_joint_pos,
        Eigen::VectorXd &arm_1_joint_pos
    );

    /**
     * @brief not used.
    */
    bool dualArmTcpRandomIK(
        const Eigen::Isometry3d &arm_0_target_pose,
        const Eigen::Isometry3d &arm_1_target_pose,
        Eigen::VectorXd &result_arm_0_joint_pos,
        Eigen::VectorXd &result_arm_1_joint_pos
    );

    /**
     * @brief not used.
    */
    bool dualArmTcpIK(
        const Eigen::VectorXd &ref_arm_0_joint_pos,
        const Eigen::VectorXd &ref_arm_1_joint_pos,
        const Eigen::Isometry3d &arm_0_target_pose,
        const Eigen::Isometry3d &arm_1_target_pose,
        Eigen::VectorXd &result_arm_0_joint_pos,
        Eigen::VectorXd &result_arm_1_joint_pos
    );

    /**
     * @brief not used.
    */
    bool dualArmEndEffectorIK(
        const Eigen::VectorXd &ref_arm_0_joint_pos,
        const Eigen::VectorXd &ref_arm_1_joint_pos,
        const Eigen::Isometry3d &arm_0_target_pose,
        const Eigen::Isometry3d &arm_1_target_pose,
        Eigen::VectorXd &result_arm_0_joint_pos,
        Eigen::VectorXd &result_arm_1_joint_pos
    );

    /**
     * @brief not used.
    */
    bool validateIKSolution( 
        robot_state::RobotState* robot_state, 
        const robot_state::JointModelGroup* joint_group, 
        const double* joint_group_variable_value
    );

public:
    ros::NodeHandle nh_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotStatePtr robot_state_;
    
    std::string dual_arm_group_name_;
    arm_planning::Robot::Ptr arm_0_, arm_1_;


}; // end class



} // end namespace

#endif