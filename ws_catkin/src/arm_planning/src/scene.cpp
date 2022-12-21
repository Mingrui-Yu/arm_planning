#include "arm_planning/scene.h"


namespace arm_planning{

// -------------------------------------------------------
Scene::Scene(
    const ros::NodeHandle& nh,
    const Robot::Ptr &robot,
    const planning_scene::PlanningSceneConstPtr& planning_scene
): nh_(nh)
{
    robot_ = robot;

    planning_scene_ = planning_scene->diff();
    planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);

    collision_env_hybrid_ = dynamic_cast<const collision_detection::CollisionEnvHybrid*>(
        planning_scene_->getCollisionEnv("HYBRID").get());
    if (!collision_env_hybrid_){
        ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
        return;
    }
}


// ------------------------------------------------------------
bool Scene::checkRobotCollision(
    const Eigen::VectorXd &joint_pos
){   
    std::string group_name = robot_->arm_group_name_;
    robot_->checkArmJointNum(joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* joint_model_group = 
        current_state.getJointModelGroup(group_name);

    current_state.setJointGroupPositions(joint_model_group, 
        Utils::eigenVectorXd2StdVector(joint_pos));
    current_state.update(); // update the transformation for collision checking

    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name;
    collision_detection::CollisionResult collision_result;
    
    // self collision & collision with world
    collision_env_hybrid_->checkCollision(
        collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    return collision_result.collision; 
    // true: collision; false: no collision
}


// ------------------------------------------------------------
void Scene::getPlanningSceneMsg(
    const Eigen::VectorXd &joint_pos,
    moveit_msgs::PlanningScene &planning_scene_msg
){
    robot_->checkArmJointNum(joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    current_state.setJointGroupPositions(robot_->arm_group_name_, Utils::eigenVectorXd2StdVector(joint_pos));
    current_state.update();

    planning_scene_->getPlanningSceneMsg(planning_scene_msg);
}





} // end namespace