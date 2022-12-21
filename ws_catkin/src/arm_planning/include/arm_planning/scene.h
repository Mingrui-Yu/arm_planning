#ifndef ARM_PLANNING_SCENE_H
#define ARM_PLANNING_SCENE_H

#include "arm_planning/common_include.h"
#include "arm_planning/robot.h"


namespace arm_planning{

/**
 * @brief Class for the planning scene
 * including the robot and the environment
 * mainly for collision checking
 */

class Scene
{
public:
    typedef std::shared_ptr<Scene> Ptr;

    Scene(
        const ros::NodeHandle& nh,
        const Robot::Ptr &robot,
        const planning_scene::PlanningSceneConstPtr& planning_scene
    );

    bool checkRobotCollision(
        const Eigen::VectorXd &joint_pos
    );

    void getPlanningSceneMsg(
        const Eigen::VectorXd &joint_pos,
        moveit_msgs::PlanningScene &planning_scene_msg
    );

    

public:
    ros::NodeHandle nh_;

    Robot::Ptr robot_;

    planning_scene::PlanningScenePtr planning_scene_;
    const collision_detection::CollisionEnvHybrid* collision_env_hybrid_;


    


}; // end class


} // end namespace

#endif