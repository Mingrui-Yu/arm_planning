#ifndef ARM_PLANNING_VISUALIZE_H
#define ARM_PLANNING_VISUALIZE_H

#include "arm_planning/common_include.h"
#include "arm_planning/node.h"
#include "arm_planning/robot.h"
#include "arm_planning/scene.h"

namespace arm_planning{

/**
 * @brief Class for visualization: publishing messages to rviz for visualization
 */

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
        const std::vector<double> &joint_pos
    );

    void publishPlanningScene(
        const Eigen::VectorXd &joint_pos
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


public:

    ros::NodeHandle nh_;
    Scene::Ptr scene_;
    Robot::Ptr robot_;

    std::string arm_base_link_name_;

    ros::Publisher planning_scene_pub_;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

}; // end class


} // end namespace

#endif