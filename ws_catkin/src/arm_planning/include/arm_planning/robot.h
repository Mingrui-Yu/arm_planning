#ifndef ARM_PLANNING_ROBOT_H
#define ARM_PLANNING_ROBOT_H

#include "arm_planning/common_include.h"


namespace arm_planning{

/**
 * @brief Class for one single arm: arm information and kinematics
 */

class Robot
{
public:
    typedef std::shared_ptr<Robot> Ptr;

    Robot(
        const ros::NodeHandle& nh,
        const std::string &robot_description_name,
        const std::string &group_name
    );

    Robot(
        const ros::NodeHandle& nh,
        moveit::core::RobotStatePtr &robot_state,
        const std::string &group_name
    );

    void loadParams();

    void printRobotInfo();

    Eigen::VectorXd randomJointPos();

    bool validateIKSolution( 
        robot_state::RobotState* robot_state, 
        const robot_state::JointModelGroup* joint_group, 
        const double* joint_group_variable_value
    );

    bool armEndEffectorIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos
    );

    /**
     * @brief the returned IK solution may not be the closest solution to the ref_joint_pos
     * @param ref_joint_pos: initial guess
     */
    bool armTcpIK(
        const Eigen::VectorXd &ref_joint_pos,
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos
    );

    /**
     * @brief first randomly sample the initial guess, then call armTcpIK()
     */
    bool armTcpRandomIK(
        const Eigen::Isometry3d &target_pose,
        Eigen::VectorXd &result_joint_pos
    );

    bool checkArmJointNum(
        const Eigen::VectorXd &joint_value
    );

    bool checkArmJointNum(
        const double &num
    );

    Eigen::Isometry3d eeInBaseToTcpInBase(
        const Eigen::Isometry3d &pose_ee
    );

    Eigen::Isometry3d tcpInBaseToEEInBase(
        const Eigen::Isometry3d &pose_gripper
    );

    void getJointNames(
        std::vector<std::string> &joint_names
    );

    Eigen::Isometry3d getLinkPose(
        const std::string &link_name,
        const Eigen::VectorXd &joint_pos
    );

    Eigen::Isometry3d getTcpPose(
        const Eigen::VectorXd &joint_pos
    );

    VecEigenVec3 getLinksPos(
        const Eigen::VectorXd &joint_pos,
        const std::vector<std::string> &link_names 
    );

    Eigen::MatrixXd getJacobianMatrix(
        const std::string &link_name,
        const Eigen::VectorXd &joint_pos,
        Eigen::Vector3d reference_point_position = Eigen::Vector3d::Zero()
    );

    Eigen::MatrixXd getTcpJacobianMatrix(
        const Eigen::VectorXd &joint_pos
    );

    bool checkJointPositionSatisfyBound(
        const Eigen::VectorXd &joint_pos
    );

    void boundJointPos(
        Eigen::VectorXd &joint_pos
    );

    double weightedJointPosDistance(
        const Eigen::VectorXd &joint_pos_0,
        const Eigen::VectorXd &joint_pos_1
    );

    double linksPosDistance(
        const VecEigenVec3 &links_pos_0,
        const VecEigenVec3 &links_pos_1
    );

    // bool armEndEffectorClosestIK(
    //     const Eigen::VectorXd &ref_joint_pos,
    //     const Eigen::Isometry3d &target_pose,
    //     Eigen::VectorXd &result_joint_pos
    // );

    // bool armTcpClosestIK(
    //     const Eigen::VectorXd &ref_joint_pos,
    //     const Eigen::Isometry3d &target_pose,
    //     Eigen::VectorXd &result_joint_pos
    // );



public:
    ros::NodeHandle nh_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotStatePtr robot_state_;

    int joint_num_;

    std::string arm_group_name_;
    std::string arm_ee_link_name_;
    Eigen::Isometry3d tcp_in_ee_pose_;
    std::vector<double> joint_sample_lb_, joint_sample_ub_;
    std::vector<double> joint_pos_weight_;
    std::vector<std::string> critical_link_names_;
    

}; // end class


} // end namespace

#endif