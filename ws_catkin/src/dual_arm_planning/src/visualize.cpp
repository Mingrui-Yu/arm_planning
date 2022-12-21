#include "dual_arm_planning/visualize.h"


namespace dual_arm_planning{

// -------------------------------------------------------
Visualize::Visualize(
    const ros::NodeHandle& nh,
    const Scene::Ptr &scene
): nh_(nh)
{
    scene_ = scene;
    dual_arm_ = scene_->dual_arm_;

    loadParams();

    initiate();
}


// -------------------------------------------------------
void Visualize::loadParams()
{
    std::string param_name;

    std::string group_name = dual_arm_->arm_0_->arm_group_name_;
    param_name = "robot_configs/" + group_name + "/base_link_name";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, arm_base_link_name_);
}


// -------------------------------------------------------
void Visualize::initiate()
{
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(arm_base_link_name_, 
        rviz_visual_tools::RVIZ_MARKER_TOPIC, dual_arm_->robot_model_loader_->getModel());

    planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning/planning_scene", 1);

    // ROS_INFO("Visualize: sleep 0.2 second to intialize the publishers.");
    ros::Duration(0.2).sleep();

    visual_tools_->setRobotStateTopic("/planning/robot_state");
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
}


// -------------------------------------------------------
void Visualize::publishText(
    const std::string &text
){
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.3;

    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
    visual_tools_->trigger();
}


// -------------------------------------------------------
void Visualize::publishRobotState(
    const std::vector<double> &arm_0_joint_pos,
    const std::vector<double> &arm_1_joint_pos
){
    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos.size());
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos.size());

    dual_arm_->robot_state_->setJointGroupPositions(dual_arm_->arm_0_->arm_group_name_, arm_0_joint_pos);
    dual_arm_->robot_state_->setJointGroupPositions(dual_arm_->arm_1_->arm_group_name_, arm_1_joint_pos);

    visual_tools_->publishRobotState(dual_arm_->robot_state_);
    visual_tools_->trigger();
}


// -------------------------------------------------------
void Visualize::publishPlanningScene(
    const Eigen::VectorXd &arm_0_joint_pos,
    const Eigen::VectorXd &arm_1_joint_pos
){
    ROS_ERROR_COND(scene_ == nullptr, "Visualize doesn't have the member scene_.");

    dual_arm_->arm_0_->checkArmJointNum(arm_0_joint_pos.size());
    dual_arm_->arm_1_->checkArmJointNum(arm_1_joint_pos.size());

    moveit_msgs::PlanningScene planning_scene_msg;
    scene_->getPlanningSceneMsg(arm_0_joint_pos, arm_1_joint_pos, planning_scene_msg);

    planning_scene_pub_.publish(planning_scene_msg);
}


// -------------------------------------------------------
void Visualize::publishNode(
    const Node::Ptr &node
){
    ROS_ERROR_COND(node == nullptr, "publishNode(): the input node is nullptr.");
    // publishRobotState(Utils::eigenVectorXd2StdVector(node->arm_0_joint_pos_),
    //                   Utils::eigenVectorXd2StdVector(node->arm_1_joint_pos_));
    publishPlanningScene(node->arm_0_joint_pos_, node->arm_1_joint_pos_);
}


// ------------------------------------------------------------
void Visualize::publishNodePath(
    const std::vector<Node::Ptr> &path_list,
    double ros_rate
){
    ros::Rate rate(ros_rate);

    for(size_t i = 0; ros::ok() && i < path_list.size(); i++){
        publishNode(path_list[i]);
        rate.sleep();
    }
}


} // end namespace