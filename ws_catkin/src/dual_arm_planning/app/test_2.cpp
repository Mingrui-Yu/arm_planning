#include <arm_planning/common_include.h>

#include "dual_arm_planning/planning_interface.h"
#include "dual_arm_planning/planning_request.h"
#include "dual_arm_planning/planning_response.h"


using namespace dual_arm_planning;

/** @brief 
 * robot: dual_ur
 * planning goal: joint-space
 * algorithm: RRTConnect
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
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.6;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.35;
        box_pose.position.y = -0.2;
        box_pose.position.z = -0.5;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "dual_base";
        // The id of the object is used to identify it.
        collision_object.id = "box2";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.6;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.35;
        box_pose.position.y = 0.2;
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

    /**
     * pre-calculate the joint positions of the start and goal configuration using IK 
     * this is more convenient for users to define the start and goal configuration
     * but the start and goal configuration for the planner is still in the joint space
     */
    DualArm::Ptr dual_arm = std::make_shared<DualArm>(nh, "robot_description", "arm_0", "arm_1", "dual_arm");
    std::vector<double> arm_0_default_joint_pos{M_PI, -3.0/4.0*M_PI, -1.0/2.0*M_PI, -3.0/4.0*M_PI, 0.0, 0.0};
    std::vector<double> arm_1_default_joint_pos{M_PI, -1.0/4.0*M_PI, 1.0/2.0*M_PI, -1.0/4.0*M_PI, 0.0, 0.0};

    Eigen::Isometry3d arm_0_start_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.5, 0.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::Isometry3d arm_1_start_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.2, 0.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::VectorXd arm_0_start_joint_pos, arm_1_start_joint_pos;
    dual_arm->arm_0_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), arm_0_start_pose, arm_0_start_joint_pos);
    dual_arm->arm_1_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), arm_1_start_pose, arm_1_start_joint_pos);
    Eigen::Isometry3d arm_0_goal_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.2, 0.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::Isometry3d arm_1_goal_pose = Utils::EigenPosQuatVec2Isometry3d(
                                                Eigen::Vector3d(0.5, 0.0, -0.4), 
                                                Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    Eigen::VectorXd arm_0_goal_joint_pos, arm_1_goal_joint_pos;
    dual_arm->arm_0_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_0_default_joint_pos), arm_0_goal_pose, arm_0_goal_joint_pos);
    dual_arm->arm_1_->armTcpIK(Utils::stdVector2EigenVectorXd(arm_1_default_joint_pos), arm_1_goal_pose, arm_1_goal_joint_pos);
    


    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    PlanningRequest req;
    PlanningResponse res;
    req.arm_0_group_name_ = "arm_0";
    req.arm_1_group_name_ = "arm_1";
    req.dual_arm_group_name_ = "dual_arm";
    req.arm_0_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_start_joint_pos);
    req.arm_1_start_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_start_joint_pos);
    req.goal_type_ = "joint_pos";
    req.arm_0_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_0_goal_joint_pos);
    req.arm_1_goal_joint_pos_ = Utils::eigenVectorXd2StdVector(arm_1_goal_joint_pos);

    req.b_visualize_start_goal_ = true;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    

    // ---------------------- do planning ---------------------------
    std::vector<std::string> test_algorthms{"RRTConnect"};
    const int num_test = 3;
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