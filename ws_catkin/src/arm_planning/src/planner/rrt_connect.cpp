#include "arm_planning/planner/rrt_connect.h"


namespace arm_planning{



// -------------------------------------------------------
RRTConnect::RRTConnect(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    loadParams();
}


// -------------------------------------------------------
void RRTConnect::loadParams()
{
    std::string param_name;

    param_name = "planner_configs/" + ALGORITHM_NAME + "/rrt_max_iter";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, rrt_max_iter_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/extend_max_steps";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, extend_max_steps_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/path_smooth_max_iter";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, path_smooth_max_iter_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/path_collision_check_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, path_collision_check_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/joint_steer_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, joint_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/dist_metric";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, dist_metric_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/connect_joint_max_dist";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, connect_joint_max_dist_);

    if(dist_metric_ == "links_pos"){
        b_node_links_pos_ = true;
    }
}


// -------------------------------------------------------
bool RRTConnect::solve(
    const PlanningRequest &req,
    PlanningResponse &res
){
    // check input
    ROS_ERROR_STREAM_COND(req.goal_type_ != "joint_pos", ALGORITHM_NAME << "can only deal with the goal type of 'joint_pos'.");
    ROS_ERROR_COND(req.start_joint_pos_.size() != robot_->joint_num_, "The number of the joints of start configuration is wrong.");
    ROS_ERROR_COND(req.goal_joint_pos_.size() != robot_->joint_num_, "The number of the joints of goal configuration is wrong.");

    loadParams();
    
    Node::Ptr start_node = std::make_shared<Node>();
    start_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.start_joint_pos_);
    start_node->note_ = "start";
    
    Node::Ptr goal_node = std::make_shared<Node>();
    goal_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.goal_joint_pos_);
    goal_node->note_ = "goal";

    ROS_ERROR_COND(checkNodeCollision(start_node), "The start node is in collision.");
    ROS_ERROR_COND(checkNodeCollision(goal_node), "The goal node is in collision.");

    if(b_node_links_pos_){
        updateNodeLinksPos(start_node);
        updateNodeLinksPos(goal_node);
    }

    if(req.b_visualize_start_goal_){
        visualizer_->publishText("start node");
        visualizer_->publishNode(start_node);
        ros::Duration(1).sleep();

        visualizer_->publishText("goal node");
        visualizer_->publishNode(goal_node);
        ros::Duration(1).sleep();
        visualizer_->publishText("planning ...");
    }
    
    std::vector<Node::Ptr> node_list_a{start_node};
    std::vector<Node::Ptr> node_list_b{goal_node};
    std::vector<Node::Ptr> res_path_list;

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;

    // rrt main loop
    t_begin = std::chrono::steady_clock::now();
    ros::Rate rate(1);
    int iter = 0;
    while(ros::ok() && iter < rrt_max_iter_){

        Node::Ptr rand_node = randomSampleNode();
        ROS_DEBUG_STREAM("solve(): tree A generate rand node.");

        Node::Ptr nearest_node_a = getNearestNode(node_list_a, rand_node, dist_metric_);
        ROS_DEBUG_STREAM("solve(): tree A generate nearest node.");

        Node::Ptr reached_node_a = extend(node_list_a, nearest_node_a, rand_node);
        ROS_DEBUG_STREAM("solve(): tree A generate reached node.");

        Node::Ptr nearest_node_b = getNearestNode(node_list_b, reached_node_a, dist_metric_);
        ROS_DEBUG_STREAM("solve(): tree B generate nearest node.");

        Node::Ptr reached_node_b = extend(node_list_b, nearest_node_b, reached_node_a);
        ROS_DEBUG_STREAM("solve(): tree B generate nearest node.");

        if(req.b_visualize_planning_process_){
            visualizer_->publishText("rand_node");
            visualizer_->publishNode(rand_node);
            rate.sleep();

            visualizer_->publishText("nearest_node_a");
            visualizer_->publishNode(nearest_node_a);
            rate.sleep();

            visualizer_->publishText("reached_node_a");
            visualizer_->publishNode(reached_node_a);
            rate.sleep();

            visualizer_->publishText("nearest_node_b");
            visualizer_->publishNode(nearest_node_b);
            rate.sleep();

            visualizer_->publishText("reached_node_b");
            visualizer_->publishNode(reached_node_b);
            rate.sleep();
        }

        if(!checkTwoNodeCanConnect(reached_node_a, reached_node_b)){
            swapTrees(node_list_a, node_list_b);
        }
        else{
            if(node_list_a[0]->note_ == "goal"){
                swapNodes(reached_node_a, reached_node_b);
            }
            res_path_list = pathExtract(reached_node_a, reached_node_b);
            
            t_end = std::chrono::steady_clock::now();
            time_used = std::chrono::duration_cast <std::chrono::duration<double>> (t_end - t_begin);
            ROS_INFO_STREAM(ALGORITHM_NAME + " find feasible path, path length: " << res_path_list.size() 
                << ", time cost: " << time_used.count() << "s" << ", iter: " << iter);

            pathSmooth(res_path_list);

            t_end = std::chrono::steady_clock::now();
            time_used = std::chrono::duration_cast <std::chrono::duration<double>> (t_end - t_begin);
            ROS_INFO_STREAM("Smoothed path length: " << res_path_list.size() 
                << ", total time cost: " << time_used.count() << "s");

            res.total_time_cost_ = time_used.count();
            res.total_iter_ = iter;

            pathNode2Vector(res_path_list, res.path_);
            
            if(req.b_visualize_res_path_){
                visualizer_->publishText("planned path");
                visualizer_->publishNodePath(pathInterpolation(res_path_list), /*ros_rate*/20);
            }
            
            res.success_ = true;
            return true;
        }

        ROS_DEBUG_STREAM("solve(): rrt iter " << iter << " done.");
        iter++;
    }

    res.success_ = false;
    ROS_WARN_STREAM("Failed to find feasible path.");
    return false;
}


} // end namespace
