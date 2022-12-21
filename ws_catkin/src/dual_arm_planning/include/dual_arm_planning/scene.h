#ifndef DUAL_ARM_PLANNING_SCENE_H
#define DUAL_ARM_PLANNING_SCENE_H

#include <arm_planning/common_include.h>
#include "dual_arm_planning/dual_arm.h"


namespace dual_arm_planning{

class Scene
{
public:
    typedef std::shared_ptr<Scene> Ptr;

    Scene(
        const ros::NodeHandle& nh,
        const DualArm::Ptr &dual_arm,
        const planning_scene::PlanningSceneConstPtr& planning_scene
    );

    bool checkRobotCollision(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos
    );

    void getPlanningSceneMsg(
        const Eigen::VectorXd &arm_0_joint_pos,
        const Eigen::VectorXd &arm_1_joint_pos,
        moveit_msgs::PlanningScene &planning_scene_msg
    );

    

public:
    ros::NodeHandle nh_;

    DualArm::Ptr dual_arm_;

    planning_scene::PlanningScenePtr planning_scene_;
    const collision_detection::CollisionEnvHybrid* collision_env_hybrid_;



}; // end class


} // end namespace

#endif