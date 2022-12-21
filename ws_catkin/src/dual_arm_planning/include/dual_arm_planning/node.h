#ifndef DUAL_ARM_PLANNING_NODE_H
#define DUAL_ARM_PLANNING_NODE_H

#include <arm_planning/common_include.h>


namespace dual_arm_planning{

class Node{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Node> Ptr;

    Node(){}


public:
    Eigen::VectorXd arm_0_joint_pos_, arm_1_joint_pos_;

    Eigen::VectorXd dual_arm_joint_pos_;

    Eigen::Isometry3d arm_0_tcp_pose_, arm_1_tcp_pose_;

    VecEigenVec3 arm_0_links_pos_, arm_1_links_pos_;

    Ptr parent_ = nullptr;
    std::string note_ = ""; 
};


} // namespace

#endif