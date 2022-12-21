#include "arm_planning/visualize.h"


namespace arm_planning{

// -------------------------------------------------------
Visualize::Visualize(
    const ros::NodeHandle& nh,
    const Scene::Ptr &scene
): nh_(nh)
{
    scene_ = scene;
    robot_ = scene_->robot_;

    loadParams();

    initiate();
}


// -------------------------------------------------------
void Visualize::loadParams()
{
    std::string param_name;
    std::string group_name = robot_->arm_group_name_;

    param_name = "robot_configs/" + group_name + "/base_link_name";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, arm_base_link_name_);
}


// -------------------------------------------------------
void Visualize::initiate()
{
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(arm_base_link_name_, 
        rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_->robot_model_loader_->getModel());

    planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning/planning_scene", 1);

    // ROS_INFO("Visualize: sleep 0.2 second to intialize the publishers.");
    ros::Duration(0.2).sleep();

    visual_tools_->setRobotStateTopic("/planning/robot_state");
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
}



// -------------------------------------------------------
void Visualize::publishRobotState(
    const std::vector<double> &joint_pos
){
    ROS_ERROR_COND(joint_pos.size() != robot_->joint_num_, "publishRobotState(): the number of joints is wrong.");

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_->robot_state_->getJointModelGroup(robot_->arm_group_name_);

    visual_tools_->publishRobotState(joint_pos, joint_model_group);
    visual_tools_->trigger();
}


// -------------------------------------------------------
void Visualize::publishPlanningScene(
    const Eigen::VectorXd &joint_pos
){
    ROS_ERROR_COND(scene_ == nullptr, "Visualize doesn't have the member scene_.");

    robot_->checkArmJointNum(joint_pos.size());

    moveit_msgs::PlanningScene planning_scene_msg;
    scene_->getPlanningSceneMsg(joint_pos, planning_scene_msg);

    planning_scene_pub_.publish(planning_scene_msg);
}


// -------------------------------------------------------
void Visualize::publishText(
    const std::string &text
){
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
    visual_tools_->trigger();
}


// -------------------------------------------------------
void Visualize::publishNode(
    const Node::Ptr &node
){
    ROS_ERROR_COND(node == nullptr, "publishNode(): the input node is nullptr.");
    // publishRobotState(Utils::eigenVectorXd2StdVector(node->joint_pos_));
    publishPlanningScene(node->joint_pos_);
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