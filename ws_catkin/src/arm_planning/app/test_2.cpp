#include "arm_planning/common_include.h"

#include "arm_planning/planning_interface.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"


using namespace arm_planning;

/** @brief 
 * robot: panda
 * planning goal: joint-space
 * algorithm: RRT, RRTConnect
 */


// -------------------------------------------------------
void addObstacle()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        // The id of the object is used to identify it.
        collision_object.id = "box1";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.8;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.3;
        box_pose.position.y = 0.3;
        box_pose.position.z = 0.4;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        // The id of the object is used to identify it.
        collision_object.id = "box2";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.8;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = -0.3;
        box_pose.position.y = -0.3;
        box_pose.position.z = 0.4;
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

    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    PlanningRequest req;
    PlanningResponse res;
    req.group_name_ = "panda_manipulator";
    req.start_joint_pos_ = std::vector<double>{
                            0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};
    req.goal_type_ = "joint_pos";
    req.goal_joint_pos_ = std::vector<double>{
                            2.12, 0.85, 0.0, -2.22, 0.0, 3.05, -0.72};

    req.b_visualize_start_goal_ = true;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    

    // ---------------------- do planning ---------------------------
    std::vector<std::string> test_algorthms{"RRT", "RRTConnect"};
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