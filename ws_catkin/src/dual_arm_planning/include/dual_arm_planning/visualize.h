#ifndef DUAL_ARM_PLANNING_VISUALIZE_H
#define DUAL_ARM_PLANNING_VISUALIZE_H

#include <arm_planning/common_include.h>
#include "dual_arm_planning/scene.h"
#include "dual_arm_planning/dual_arm.h"
#include "dual_arm_planning/node.h"

namespace dual_arm_planning{


class Visualize
{
public:
    typedef std::shared_ptr<Visualize> Ptr;

    Visualize(
        const ros::NodeHandle& nh,
        const Scene::Ptr &scene
    );

    void loadParams();

    void initiate();

    void publishRobotState(
        const std::vector<double> &arm_0_joint_pos,
        const std::vector<double> &arm_1_joint_pos
    );

    void publishPlanningScene(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos
    );

    void publishText(
        const std::string &text
    );

    void publishNode(
        const Node::Ptr &node
    );

    void publishNodePath(
        const std::vector<Node::Ptr> &path_list,
        double ros_rate = 2.0
    );


public: // protected

    ros::NodeHandle nh_;
    Scene::Ptr scene_;
    DualArm::Ptr dual_arm_;

    std::string arm_base_link_name_;

    ros::Publisher planning_scene_pub_;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

}; // end class


} // end namespace

#endif