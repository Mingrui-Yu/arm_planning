#include "dual_arm_planning/scene.h"


namespace dual_arm_planning{

// -------------------------------------------------------
Scene::Scene(
    const ros::NodeHandle& nh,
    const DualArm::Ptr &dual_arm,
    const planning_scene::PlanningSceneConstPtr& planning_scene
): nh_(nh)
{
    dual_arm_ = dual_arm;

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
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos
){   
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_0_->arm_group_name_);
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_1_->arm_group_name_);

    // 输入关节角
    current_state.setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
    current_state.setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

    current_state.update(); // 更新所有transformation，用于碰撞检测

    collision_detection::CollisionRequest collision_request;
    // collision_request.contacts = true;
    // collision_request.group_name = ""; // if empty, assume the complete robot
    collision_detection::CollisionResult collision_result;
    
    // self collision & collision with world
    collision_env_hybrid_->checkCollision(
        collision_request, collision_result, current_state, planning_scene_->getAllowedCollisionMatrix());

    // for(auto iter = collision_result.contacts.begin(); iter != collision_result.contacts.end(); iter++){
    //     std::cout << iter->first.first << " " << iter->first.second << std::endl;
    // }

    return collision_result.collision; 
    // true: 有碰撞; false: 无碰撞
}


// ------------------------------------------------------------
void Scene::getPlanningSceneMsg(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos,
    moveit_msgs::PlanningScene &planning_scene_msg
){
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos);
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos);

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_0_->arm_group_name_);
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        current_state.getJointModelGroup(dual_arm_->arm_1_->arm_group_name_);

    // 输入关节角
    current_state.setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
    current_state.setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

    current_state.update(); // 更新所有transformation，用于碰撞检测

    planning_scene_->getPlanningSceneMsg(planning_scene_msg);
}








} // end namespace