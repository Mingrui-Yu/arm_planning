#include "arm_planning/planner/cbirrt.h"


namespace arm_planning{



// -------------------------------------------------------
CBiRRT::CBiRRT(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    loadParams();

    ROS_WARN_STREAM("Please note that in the current version the constraints in " << ALGORITHM_NAME << 
        " should be specified in the corresponding source code (we haven't implemented the constraints as input args), and the start and goal must satifiy the constraints.");
}


// -------------------------------------------------------
void CBiRRT::loadParams()
{
    std::string param_name;

    param_name = "planner_configs/" + ALGORITHM_NAME + "/new_goal_ik_probability";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, new_goal_ik_probability_);

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

    param_name = "planner_configs/" + ALGORITHM_NAME + "/constraint_error_thres";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, constraint_error_thres_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/constraint_projection_max_iter";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, constraint_projection_max_iter_);

    if(dist_metric_ == "links_pos"){
        b_node_links_pos_ = true;
    }
}


// -------------------------------------------------------
void CBiRRT::addRandomGoalIKNode(
    const Node::Ptr &goal_node,
    std::vector<Node::Ptr> &node_list_goal
){
    Eigen::VectorXd goal_ik_joint_pos(robot_->joint_num_);;
    while(ros::ok()){
        robot_->armTcpRandomIK(goal_node->tcp_pose_, goal_ik_joint_pos);
        if(!scene_->checkRobotCollision(goal_ik_joint_pos)){
            break;
        }
    }

    Node::Ptr goal_ik_node = std::make_shared<Node>();
    goal_ik_node->joint_pos_ = goal_ik_joint_pos;
    goal_ik_node->note_ = "goal_ik";

    if(b_node_links_pos_) updateNodeLinksPos(goal_ik_node);
    if(b_node_tcp_pose_) updateNodeTcpPose(goal_ik_node);

    node_list_goal.push_back(goal_ik_node);
}


// -------------------------------------------------------
/**
 * @brief task-specific
 * The current constraint is that the z-axis should be down.
*/
Eigen::VectorXd CBiRRT::constraintError(
    const Eigen::VectorXd &pose  // pos + rpy angle (fixed-axis x-y-z angle)
){
    Eigen::VectorXd pose_vec = pose;
    Eigen::VectorXd constraint_error = Eigen::VectorXd::Zero(6);

    if(pose_vec(3) < 0) pose_vec(3) += 2 * M_PI; // from -pi~pi to 0~2pi

    constraint_error(3) = pose_vec(3) - M_PI;
    constraint_error(4) = pose_vec(4) - (0.0);
    
    return constraint_error;
}


// -------------------------------------------------------
bool CBiRRT::constrainConfig(
    Node::Ptr &node,
    const Node::Ptr &node_old
){
    int iter = 0;
    while(ros::ok() && iter < constraint_projection_max_iter_){
        updateNodeTcpPose(node);
        Eigen::VectorXd pose_vec = Utils::isometryToPosAndRPYAngle(node->tcp_pose_);

        Eigen::VectorXd constraint_error = constraintError(pose_vec);

        ROS_DEBUG_STREAM("constrainConfig(): updated pose_vec: " << pose_vec.transpose());
        ROS_DEBUG_STREAM("constrainConfig(): constraint_error: " << constraint_error.transpose());

        if(constraint_error.norm() < constraint_error_thres_){
            ROS_DEBUG_STREAM("constrainConfig(): satify the constraint_error_thres.");
            return true;
        }

        Eigen::MatrixXd jaco_transform_matrix = Eigen::MatrixXd::Zero(6, 6);
        jaco_transform_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jaco_transform_matrix.block<3, 3>(3, 3) = Utils::matrixRelateAngularVelToRPYVel(pose_vec.block<3,1>(3,0));

        Eigen::MatrixXd jacobian_euler = jaco_transform_matrix * robot_->getTcpJacobianMatrix(node->joint_pos_);

        node->joint_pos_ += Utils::pseudoInverse(jacobian_euler) * (-constraint_error);

        ROS_DEBUG_STREAM("constrainConfig(): jaco_transform_matrix: " << jaco_transform_matrix);
        ROS_DEBUG_STREAM("constrainConfig(): jacobian_euler: " << jacobian_euler);
        ROS_DEBUG_STREAM("constrainConfig(): joint_pos increment: " 
            << (Utils::pseudoInverse(jacobian_euler) * (-constraint_error)).transpose());

        if(!robot_->checkJointPositionSatisfyBound(node->joint_pos_)){
            ROS_DEBUG_STREAM("constrainConfig(): failed.");
            return false;
        }
        if(node_old != nullptr && (node->joint_pos_ - node_old->joint_pos_).norm() > 2*joint_steer_step_size_*std::sqrt(robot_->joint_num_)){
            ROS_DEBUG_STREAM("constrainConfig(): failed.");
            return false;
        }
        iter++;
    }
    return false;
}


// -------------------------------------------------------
Node::Ptr CBiRRT::oneStepSteer(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    Eigen::VectorXd diff = to_node->joint_pos_ - from_node->joint_pos_;

    Node::Ptr new_node = std::make_shared<Node>();

    double scale_factor = std::sqrt(robot_->joint_num_);

    new_node->joint_pos_ = from_node->joint_pos_ + 
        std::min(joint_steer_step_size_ * scale_factor, diff.norm()) * diff.normalized();

    if(!constrainConfig(new_node, from_node)){
        return nullptr;
    }
    
    if(b_node_links_pos_) updateNodeLinksPos(new_node);
    if(b_node_tcp_pose_) updateNodeTcpPose(new_node);
    
    ROS_DEBUG_STREAM("oneStepSteer(): generate new node.");

    return new_node;
}


// ------------------------------------------------------------
std::vector<Node::Ptr> CBiRRT::pathInterpolation(
    const std::vector<Node::Ptr> &path_list
){
    std::vector<Node::Ptr> interpolated_path;

    for(size_t i = 0; ros::ok() && i < path_list.size()-1; i++){

        Eigen::VectorXd joint_pos_from = path_list[i]->joint_pos_;
        Eigen::VectorXd joint_pos_to = path_list[i+1]->joint_pos_;

        Eigen::VectorXd diff = joint_pos_to - joint_pos_from;
        Eigen::VectorXd diff_normalized = diff.normalized();

        double scale_factor = std::sqrt(robot_->joint_num_);
        int num_step = int(diff.norm() / scale_factor / path_interpolation_step_size_);

        interpolated_path.push_back(path_list[i]);

        // linear interpolation between from_node and to_node (not including from_node and to_node）
        for(size_t i = 1; i <= num_step; i++){
            Eigen::VectorXd joint_pos = joint_pos_from + 
                i * path_interpolation_step_size_ * diff_normalized * scale_factor;

            Node::Ptr temp_node = std::make_shared<Node>();
            temp_node->joint_pos_ = joint_pos;

            // project the interpolated waypoint onto the constraint manifold
            if(!constrainConfig(temp_node, nullptr)){
                ROS_ERROR("constrainConfig() failed during the path interpolation.");
            }

            interpolated_path.push_back(temp_node);
        }
    }

    return interpolated_path;
}


// -------------------------------------------------------
bool CBiRRT::solve(
    const PlanningRequest &req,
    PlanningResponse &res
){
    // check input
    bool b_goal_is_tcp_pose = false;
    if(req.goal_type_ == "tcp_pose"){
        b_goal_is_tcp_pose = true;
        ROS_ERROR_COND(!RosUtils::checkGeometryPoseInitialized(req.goal_pose_), "The goal pose of tcp hasn't been assigned.");
    }else if(req.goal_type_ == "joint_pos"){
        b_goal_is_tcp_pose = false;
        ROS_ERROR_COND(req.goal_joint_pos_.size() != robot_->joint_num_, "The number of the joints of goal configuration is wrong.");
    }else{
        ROS_ERROR_STREAM_COND(req.goal_type_ != "joint_pos", ALGORITHM_NAME << "can only deal with the goal types of 'joint_pos' or 'tcp_pose'.");
    }
    ROS_ERROR_COND(req.start_joint_pos_.size() != robot_->joint_num_, "The number of the joints of start configuration is wrong.");
    
    // start and goal node
    Node::Ptr start_node = std::make_shared<Node>();
    start_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.start_joint_pos_);
    start_node->note_ = "start";
    ROS_ERROR_COND(checkNodeCollision(start_node), "The start node is in collision.");
    if(b_node_links_pos_) updateNodeLinksPos(start_node);
    
    Node::Ptr goal_node = std::make_shared<Node>();
    goal_node->note_ = "goal";
    if(b_goal_is_tcp_pose){
        goal_node->tcp_pose_ = RosUtils::rosPose2EigenPose(req.goal_pose_);
    }else{
        goal_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.goal_joint_pos_);
        ROS_ERROR_COND(checkNodeCollision(goal_node), "The goal node is in collision.");
        if(b_node_links_pos_) updateNodeLinksPos(goal_node);
    }

    if(req.b_visualize_start_goal_){
        visualizer_->publishText("start node");
        visualizer_->publishNode(start_node);
        ros::Duration(1).sleep();

        if(b_goal_is_tcp_pose){
            visualizer_->visual_tools_->publishAxisLabeled(req.goal_pose_, "goal pose");
            visualizer_->visual_tools_->trigger();
        }else{
            visualizer_->publishText("goal node");
            visualizer_->publishNode(goal_node);
        }
        ros::Duration(1).sleep();
        visualizer_->publishText("planning ...");
    }

    std::vector<Node::Ptr> node_list_a{start_node};
    std::vector<Node::Ptr> node_list_b;
    if(!b_goal_is_tcp_pose) node_list_b.push_back(goal_node);
    std::vector<Node::Ptr> res_path_list;

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;

    // rrt main loop
    t_begin = std::chrono::steady_clock::now();
    ros::Rate rate(1);
    int iter = 0;
    while(ros::ok() && iter < rrt_max_iter_){
        if(b_goal_is_tcp_pose){
            auto &node_list_goal = (node_list_a[0]->note_ == "start") ? node_list_b : node_list_a;
            bool b_new_goal_ik = (node_list_goal.size() == 0) || (Utils::getRandomDouble() < new_goal_ik_probability_);
            if(b_new_goal_ik){
                addRandomGoalIKNode(goal_node, node_list_goal);
            }
        }

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
            if(node_list_b[0]->note_ == "start"){
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
