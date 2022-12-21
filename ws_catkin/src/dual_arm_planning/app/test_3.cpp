#include <arm_planning/common_include.h>

#include "dual_arm_planning/planning_interface.h"
#include "dual_arm_planning/planning_request.h"
#include "dual_arm_planning/planning_response.h"


using namespace dual_arm_planning;

/** @brief 
 * robot: dual_ur
 * goal: joint space
 * constraint: dual-arm grasp a box (closed-chain constraint)
 * algorithm: CBiRRT
 */


// -------------------------------------------------------
void addObstacle()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "box1";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.3;
        primitive.dimensions[primitive.BOX_Z] = 0.6;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.45;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.5;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep(); // required: leave some time for publishing the message to Move Group
}


// -------------------------------------------------------
void addAttachedObject(const Eigen::Vector3d &box_size)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "attached_box";
    collision_object.header.frame_id = "arm_0_tool0";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_size(0);
    primitive.dimensions[primitive.BOX_Y] = box_size(1);
    primitive.dimensions[primitive.BOX_Z] = box_size(2);
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.x = box_size(0) / 2.0;
    grab_pose.position.y = 0.0;
    grab_pose.position.z = 0.145;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(grab_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::AttachedCollisionObject attached_collision_object;
    attached_collision_object.link_name = "arm_0_tool0";
    attached_collision_object.object = collision_object;
    // attached_collision_object.touch_links = std::vector<std::string>{"arm_1_wrist_3_link", "arm_1_tool0"};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();
    bool success = planning_scene_interface.applyAttachedCollisionObject(attached_collision_object);
    ROS_INFO_STREAM("Add atached object ? : " << success);
    ros::Duration(1.0).sleep(); // required: leave some time for publishing the message to Move Group
}


// -------------------------------------------------------
int main(int argc, char** argv){

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // change the logger level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;


    addObstacle();


    Eigen::Vector3d attached_box_size(0.5, 0.06, 0.06);
    geometry_msgs::Pose constraint_tcp_relative_pose;
    constraint_tcp_relative_pose.position.x = 0.0;
    constraint_tcp_relative_pose.position.y = attached_box_size(0);
    constraint_tcp_relative_pose.position.z = 0.0;
    constraint_tcp_relative_pose.orientation.w = 1.0;

    addAttachedObject(attached_box_size);

    /**
     * pre-calculate the joint positions of the start and goal configuration using IK 
     * this is more convenient for users to define the start and goal configuration
     * but the start and goal configuration for the planner is still in the joint space
     */
    DualArm::Ptr dual_arm = std::make_shared<DualArm>(nh, "robot_description", "arm_0", "arm_1");
    std::vector<double> arm_0_default_joint_pos{M_PI, -3.0/4.0*M_PI, -1.0/2.0*M_PI, -3.0/4.0*M_PI, 0.0, 0.0};
    std::vector<double> arm_1_default_joint_pos{M_PI, -1.0/4.0*M_PI, 1.0/2.0*M_PI, -1.0/4.0*M_PI, 0.0, 0.0};

    Eigen::Isometry3d arm_0_start_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.7, -attached_box_size(0) / 2.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::Isometry3d arm_1_start_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.7, +attached_box_size(0) / 2.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::VectorXd arm_0_start_joint_pos, arm_1_start_joint_pos;
    dual_arm->arm_0_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), arm_0_start_pose, arm_0_start_joint_pos);
    dual_arm->arm_1_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), arm_1_start_pose, arm_1_start_joint_pos);
    Eigen::Isometry3d arm_0_goal_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.2, -attached_box_size(0) / 2.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::Isometry3d arm_1_goal_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.2, +attached_box_size(0) / 2.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::VectorXd arm_0_goal_joint_pos, arm_1_goal_joint_pos;
    dual_arm->arm_0_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), arm_0_goal_pose, arm_0_goal_joint_pos);
    dual_arm->arm_1_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), arm_1_goal_pose, arm_1_goal_joint_pos);


    // ---------------------- planning problem ---------------------------
    PlanningRequest req;
    PlanningResponse res;
    req.arm_0_group_name_ = "arm_0";
    req.arm_1_group_name_ = "arm_1";
    req.arm_0_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_start_joint_pos);
    req.arm_1_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_start_joint_pos);
    req.goal_type_ = "joint_pos";
    req.arm_0_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_goal_joint_pos);
    req.arm_1_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_goal_joint_pos);
    req.constraint_tcp_relative_pose_ = constraint_tcp_relative_pose;

    req.b_visualize_start_goal_ = true;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    
    // ---------------------- do planning ---------------------------
    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    std::vector<std::string> test_algorthms{"CBiRRT"};
    const int num_test = 50;
    double time_cost;
    int num_success;
    int iter;

    for(auto &algorithm_name: test_algorthms){
        time_cost = 0.0;
        num_success = 0;
        iter = 0;
        for(size_t i = 0; ros::ok() && i < num_test; i++){
            std::cout << "test " << i << ": " << std::endl;
            pi->solve(algorithm_name, req, res);
            if(res.success_){
                num_success++;
                time_cost += res.total_time_cost_;
                iter += res.total_iter_;
            }
        }
        std::cout << "Results: success rate: " << double(num_success) / double(num_test) << std::endl;
        std::cout << "Results: average time cost: " << time_cost / double(num_success) << std::endl;
        std::cout << "Results: average iteration: " << iter / double(num_success) << std::endl;
    }


    ros::Duration(0.2).sleep();
    return 0;
}