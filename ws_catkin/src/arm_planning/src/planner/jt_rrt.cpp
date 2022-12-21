#include "arm_planning/planner/jt_rrt.h"


namespace arm_planning{



// -------------------------------------------------------
JTRRT::JTRRT(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    b_node_tcp_pose_ = true; // in JTRRT, b_node_tcp_pose_ must be true;

    loadParams();
}


// -------------------------------------------------------
void JTRRT::loadParams()
{
    std::string param_name;

    param_name = "planner_configs/" + ALGORITHM_NAME + "/goal_bias_probability";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, goal_bias_probability_);

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

    param_name = "planner_configs/" + ALGORITHM_NAME + "/jacobian_steer_type";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, jacobian_steer_type_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/pos_steer_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, pos_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/angle_steer_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, angle_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/dist_metric";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, dist_metric_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/connect_joint_max_dist";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, connect_joint_max_dist_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/connect_tcp_max_dist";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, connect_tcp_max_dist_);

    if(dist_metric_ == "links_pos"){
        b_node_links_pos_ = true;
    }
}


// -------------------------------------------------------
bool JTRRT::checkTwoNodeTcpPoseCanConnect(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    ROS_DEBUG_STREAM("checkTwoNodeTcpPoseCanConnect() begins.");

    bool close_enough = twoNodeDistance(from_node, to_node, "tcp_pose") < connect_tcp_max_dist_;

    ROS_DEBUG_STREAM("checkTwoNodeTcpPoseCanConnect() done.");

    return close_enough;
}


// -------------------------------------------------------
Node::Ptr JTRRT::oneStepSteerToTcpPose(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    Eigen::Vector3d pos_diff = to_node->tcp_pose_.translation() - from_node->tcp_pose_.translation();
    Eigen::AngleAxisd rotation_vector_diff(to_node->tcp_pose_.rotation() * from_node->tcp_pose_.rotation().transpose());
    Eigen::Vector3d rot_diff = rotation_vector_diff.angle() * rotation_vector_diff.axis();

    Eigen::VectorXd pose_diff(6);
    pose_diff << pos_diff(0), pos_diff(1), pos_diff(2), 
                  rot_diff(0), rot_diff(1), rot_diff(2);

    // choose the number of steer steps as the larger one between the required pos_steer_step and angle_steer_step
    double pos_steer_step = pos_diff.norm() / pos_steer_step_size_;
    double angle_steer_step = rot_diff.norm() / angle_steer_step_size_;
    int steer_step = int(std::max(1.0, std::round(std::max(pos_steer_step, angle_steer_step))));

    Eigen::VectorXd pose_steer = pose_diff / steer_step;

    Eigen::MatrixXd jacobian = robot_->getTcpJacobianMatrix(from_node->joint_pos_);

    Eigen::VectorXd joint_steer;
    if(jacobian_steer_type_ == "transpose"){
        joint_steer = jacobian.transpose() * pose_steer;
    }else if(jacobian_steer_type_ == "inverse"){
        joint_steer = Utils::pseudoInverse(jacobian) * pose_steer;
    }else{
        ROS_ERROR_STREAM("oneStepSteerToTcpPose(): invalid jacobian_steer_type: " << jacobian_steer_type_);
    }

    Node::Ptr new_node = std::make_shared<Node>();
    new_node->joint_pos_ = from_node->joint_pos_ + joint_steer;
    if(b_node_links_pos_) updateNodeLinksPos(new_node);
    if(b_node_tcp_pose_) updateNodeTcpPose(new_node);

    ROS_DEBUG_STREAM("oneStepSteerToTcpPose(): generate new node.");

    return new_node;
}


// -------------------------------------------------------
Node::Ptr JTRRT::extendToTcpPose(
    std::vector<Node::Ptr> &node_list,
    const Node::Ptr &from_node,
    const Node::Ptr &to_node,
    bool greedy
){
    int step = 0;
    Node::Ptr s_node = from_node;
    Node::Ptr s_node_old = from_node;

    while(ros::ok() && (greedy || step < extend_max_steps_)){
        bool b_connect_to_to_node = checkTwoNodeTcpPoseCanConnect(s_node, to_node);
        ROS_DEBUG_STREAM("extendToTcpPose(): s_node and to_node can be connected? : " << b_connect_to_to_node);
        if(b_connect_to_to_node) return s_node;

        bool b_far_awar_from_to_node = 
            twoNodeDistance(s_node, to_node, "tcp_pose") > twoNodeDistance(s_node_old, to_node, "tcp_pose");
        ROS_DEBUG_STREAM("extendToTcpPose(): s_node is far away from to_node than s_node_old ? :" << b_far_awar_from_to_node);
        if(b_far_awar_from_to_node) return s_node_old;

        s_node_old = s_node;

        // one step steer
        s_node = oneStepSteerToTcpPose(s_node, to_node);

        bool node_valid = (s_node != nullptr) 
            && !checkNodeCollision(s_node) && checkTwoNodeCanConnect(s_node_old, s_node);

        if(node_valid){
            s_node->parent_ = s_node_old;
            node_list.push_back(s_node);
        }
        else{
            ROS_DEBUG_STREAM("extendToTcpPose(): the node by oneStepSteerToTcpPose() is invalid.");
            return s_node_old;
        }
        
        step++;
    }

    ROS_DEBUG_STREAM("extendToTcpPose(): have done max steer steps.");
    return s_node;
}


// -------------------------------------------------------
bool JTRRT::solve(
    const PlanningRequest &req,
    PlanningResponse &res
){
    // check input
    ROS_ERROR_STREAM_COND(req.goal_type_ != "tcp_pose", ALGORITHM_NAME << "can only deal with the goal type of 'tcp_pose'.");
    ROS_ERROR_COND(req.start_joint_pos_.size() != robot_->joint_num_, "The number of the joints of start configuration is wrong.");
    ROS_ERROR_COND(!RosUtils::checkGeometryPoseInitialized(req.goal_pose_), "The goal pose of tcp hasn't been assigned.");

    // start and goal node
    Node::Ptr start_node = std::make_shared<Node>();
    start_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.start_joint_pos_);
    start_node->note_ = "start";
    
    Node::Ptr goal_node = std::make_shared<Node>();
    goal_node->tcp_pose_ = RosUtils::rosPose2EigenPose(req.goal_pose_);
    goal_node->note_ = "goal";
    
    ROS_ERROR_COND(checkNodeCollision(start_node), "The start node is in collision.");
    if(b_node_links_pos_) updateNodeLinksPos(start_node);
    if(b_node_tcp_pose_){
        updateNodeTcpPose(start_node);
        ROS_DEBUG_STREAM("solve(): start_node tcp pose: pos: " << start_node->tcp_pose_.translation().transpose() 
            << ", quat: " << Eigen::Quaterniond(start_node->tcp_pose_.rotation()).coeffs().transpose());
    }

    if(req.b_visualize_start_goal_){
        visualizer_->publishText("start node");
        visualizer_->publishNode(start_node);
        ros::Duration(1).sleep();

        visualizer_->visual_tools_->publishAxisLabeled(req.goal_pose_, "goal pose");
        visualizer_->visual_tools_->trigger();
        ros::Duration(1).sleep();
        visualizer_->publishText("planning ...");
    }
    
    std::vector<Node::Ptr> node_list{start_node};
    std::vector<Node::Ptr> res_path_list;

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;

    ros::Rate rate(1);

    // rrt main loop
    t_begin = std::chrono::steady_clock::now();
    int iter = 0;
    while(ros::ok() && iter < rrt_max_iter_){
        Node::Ptr rand_node, nearest_node, reached_node;
        bool b_goal_bias = Utils::getRandomDouble() < goal_bias_probability_;

        if(b_goal_bias){
            rand_node = goal_node;
            ROS_DEBUG_STREAM("solve(): choose goal node as rand node (goal_bias).");

            nearest_node = getNearestNode(node_list, rand_node, "tcp_pose");
            ROS_DEBUG_STREAM("solve(): generate nearest node (goal_bias).");

            reached_node = extendToTcpPose(node_list, nearest_node, rand_node, /*greedy*/true);
            ROS_DEBUG_STREAM("solve(): generate reached node (goal_bias).");
        }
        else{
            rand_node = randomSampleNode();
            ROS_DEBUG_STREAM("solve(): generate rand node.");
            
            nearest_node = getNearestNode(node_list, rand_node, dist_metric_);
            ROS_DEBUG_STREAM("solve(): generate nearest node.");

            reached_node = extend(node_list, nearest_node, rand_node);
            ROS_DEBUG_STREAM("solve(): generate reached node.");
        }

        if(req.b_visualize_planning_process_){
            ROS_DEBUG_STREAM("visualize_planning_process begins.");
            std::string b_goal_bias_text = b_goal_bias ? std::string(" (goal_pose_bias)") : std::string("");

            if(b_goal_bias == false){
                visualizer_->publishText("rand_node" + b_goal_bias_text);
                visualizer_->publishNode(rand_node);
                rate.sleep();
            }
            
            visualizer_->publishText("nearest_node" + b_goal_bias_text);
            visualizer_->publishNode(nearest_node);
            rate.sleep();

            visualizer_->publishText("reached_node" + b_goal_bias_text);
            visualizer_->publishNode(reached_node);
            rate.sleep();
            ROS_DEBUG_STREAM("visualize_planning_process done.");
        }


        if(checkTwoNodeTcpPoseCanConnect(reached_node, goal_node)){
            // use IK to calculate the joint pos of the goal node (using the joint pos of the reached node as the initial value)
            robot_->armTcpIK(reached_node->joint_pos_, goal_node->tcp_pose_, goal_node->joint_pos_);

            res_path_list = pathExtract(reached_node, goal_node);
            
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
