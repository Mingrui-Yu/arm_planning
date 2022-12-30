#include "dual_arm_planning/planner/cbirrt.h"


namespace dual_arm_planning{



// -------------------------------------------------------
CBiRRT::CBiRRT(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    loadParams();

    ROS_WARN_STREAM("Please note that the current implementation of " << ALGORITHM_NAME << 
        " only support the closed-chain constraint (the two arms grasp a object, i.e., the arm_1 tcp pose in the arm_0 tcp frame is constrained).");
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



// // -------------------------------------------------------
// Node::Ptr CBiRRT::generateGoalIKNode(
//     const Node::Ptr &goal_node,
//     const Node::Ptr &ref_node
// ){
//     Node::Ptr goal_ik_node = std::make_shared<Node>();

//     int iter = 0;
//     while(ros::ok()){
//         bool success;
//         if(ref_node == nullptr){
//             success = dual_arm_->dualArmTcpRandomIK(
//                                 goal_node->arm_0_tcp_pose_, goal_node->arm_1_tcp_pose_, 
//                                 goal_ik_node->arm_0_joint_pos_, goal_ik_node->arm_1_joint_pos_);
//         }else{
//             success = dual_arm_->dualArmTcpIK(ref_node->arm_0_joint_pos_, ref_node->arm_1_joint_pos_,
//                                 goal_node->arm_0_tcp_pose_, goal_node->arm_1_tcp_pose_,
//                                 goal_ik_node->arm_0_joint_pos_, goal_ik_node->arm_1_joint_pos_);
//         }
//         ROS_DEBUG_STREAM("success: " << success);
//         if(success && !checkNodeCollision(goal_ik_node)){
//             break;
//         }
//         ROS_DEBUG_STREAM("generateGoalIKNode(): iter = " << iter++);
//     }

//     updateNode(goal_ik_node);
//     goal_ik_node->note_ = "goal_ik";
//     return goal_ik_node;
// }

// -------------------------------------------------------
Node::Ptr CBiRRT::generateGoalIKNode(
    const Node::Ptr &goal_node,
    const Node::Ptr &ref_node
){
    Node::Ptr goal_ik_node = std::make_shared<Node>();

    int iter = 0;
    while(ros::ok()){
        bool arm_0_success, arm_1_success;

        if(ref_node == nullptr){
            arm_0_success = dual_arm_->arm_0_->armTcpRandomIK(goal_node->arm_0_tcp_pose_, goal_ik_node->arm_0_joint_pos_);
            arm_1_success = dual_arm_->arm_1_->armTcpRandomIK(goal_node->arm_1_tcp_pose_, goal_ik_node->arm_1_joint_pos_);
        }else{
            arm_0_success = dual_arm_->arm_0_->armTcpIK(ref_node->arm_0_joint_pos_, goal_node->arm_0_tcp_pose_, goal_ik_node->arm_0_joint_pos_);
            arm_1_success = dual_arm_->arm_1_->armTcpIK(ref_node->arm_1_joint_pos_, goal_node->arm_1_tcp_pose_, goal_ik_node->arm_1_joint_pos_);
        }

        if(arm_0_success && arm_1_success && !checkNodeCollision(goal_ik_node)){
            break;
        }
        ROS_DEBUG_STREAM("generateGoalIKNode(): iter = " << iter++);
    }

    updateNode(goal_ik_node);
    goal_ik_node->note_ = "goal_ik";
    return goal_ik_node;
}


// -------------------------------------------------------
Node::Ptr CBiRRT::generateGoalClosestIKNode(
    const Node::Ptr &goal_node,
    const Node::Ptr &ref_node
){
    int attemp_num = 50;
    double min_dist = 1e10;
    Node::Ptr goal_closest_ik_node;

    int iter = 0;
    while(ros::ok() && iter < attemp_num){
        Node::Ptr new_node = generateGoalIKNode(goal_node, ref_node);
        
        double new_dist = twoNodeDistance(new_node, ref_node, "joint_pos");
        if(new_dist < min_dist){
            goal_closest_ik_node = new_node;
            min_dist = new_dist;
        }

        ROS_DEBUG_STREAM("generateGoalClosestIKNode(): iter = " << iter);
        iter++;
    }

    return goal_closest_ik_node;
}


// -------------------------------------------------------
Eigen::VectorXd CBiRRT::constraintError(
    const Eigen::VectorXd &pose,  // pos + rpy angle
    const Eigen::VectorXd &pose_lb,
    const Eigen::VectorXd &pose_ub
){
    Eigen::VectorXd pose_vec = pose;
    Eigen::VectorXd constraint_error = Eigen::VectorXd::Zero(6);

    for(size_t i = 0; i < pose.size(); i++){
        if(pose(i) < pose_lb(i)){
            constraint_error(i) = pose(i) - pose_lb(i);
        }else if(pose(i) > pose_ub(i)){
            constraint_error(i) = pose(i) - pose_ub(i);
        }
    }

    return constraint_error;
}


// -------------------------------------------------------
bool CBiRRT::constrainConfig(
    Node::Ptr &node,
    const Node::Ptr &node_old
){
    // arm_1
    int iter = 0;
    // double last_constraint_error_norm = 1e10;
    while(ros::ok() && iter < constraint_projection_max_iter_){
        updateNodeTcpPose(node);

        Eigen::Isometry3d constraint_frame = node->arm_0_tcp_pose_;
        Eigen::Isometry3d pose_in_constraint_frame = constraint_frame.inverse() * node->arm_1_tcp_pose_;
        Eigen::VectorXd pose_vec = Utils::isometryToPosAndRPYAngle(pose_in_constraint_frame);

        Eigen::VectorXd pose_lb = Utils::isometryToPosAndRPYAngle(RosUtils::rosPose2EigenPose(req_.constraint_tcp_relative_pose_));
        Eigen::VectorXd pose_ub = pose_lb;

        Eigen::VectorXd constraint_error = constraintError(pose_vec, pose_lb, pose_ub);

        // ROS_DEBUG_STREAM("constrainConfig(): updated pose_vec: " << pose_vec.transpose());
        // ROS_DEBUG_STREAM("constrainConfig(): constraint_error: " << constraint_error.transpose());

        if(constraint_error.norm() < constraint_error_thres_){
            ROS_DEBUG_STREAM("constrainConfig(): arm_1 satify the constraint_error_thres.");
            return true;
        }
        // else if(constraint_error.norm() >= last_constraint_error_norm){
        //     ROS_DEBUG_STREAM("constrainConfig(): failed, constraint_error_thres increased.");
        //     return false;
        // }
        // last_constraint_error_norm = constraint_error.norm();

        Eigen::MatrixXd avel2eulervel_transform = Eigen::MatrixXd::Zero(6, 6);
        avel2eulervel_transform.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        avel2eulervel_transform.block<3, 3>(3, 3) = Utils::matrixRelateAngularVelToRPYVel(pose_vec.block<3,1>(3,0));

        Eigen::MatrixXd base2constraintframe_transform = Eigen::MatrixXd::Zero(6, 6);
        base2constraintframe_transform.block<3, 3>(0, 0) = constraint_frame.rotation().transpose();
        base2constraintframe_transform.block<3, 3>(3, 3) = constraint_frame.rotation().transpose();

        Eigen::MatrixXd transformed_jacobian = avel2eulervel_transform * base2constraintframe_transform
             * dual_arm_->arm_1_->getTcpJacobianMatrix(node->arm_1_joint_pos_);

        node->arm_1_joint_pos_ += Utils::pseudoInverse(transformed_jacobian) * (-constraint_error);

        // ROS_DEBUG_STREAM("constrainConfig(): avel2eulervel_transform: " << avel2eulervel_transform);
        // ROS_DEBUG_STREAM("constrainConfig(): transformed_jacobian: " << transformed_jacobian);
        // ROS_DEBUG_STREAM("constrainConfig(): joint_pos increment: " 
        //     << (Utils::pseudoInverse(transformed_jacobian) * (-constraint_error)).transpose());

        if(!dual_arm_->arm_1_->checkJointPositionSatisfyBound(node->arm_1_joint_pos_)){
            ROS_DEBUG_STREAM("constrainConfig(): failed.");
            return false;
        }
        if(node_old != nullptr && (node->arm_1_joint_pos_ - node_old->arm_1_joint_pos_).norm() 
            > 2*joint_steer_step_size_*std::sqrt(node->arm_1_joint_pos_.size()))
        {
            ROS_DEBUG_STREAM("constrainConfig(): failed.");
            return false;
        }
        iter++;
    }

    ROS_INFO_STREAM("constrainConfig(): failed: max iter " << iter);
    return false;
}


// -------------------------------------------------------
Node::Ptr CBiRRT::oneStepSteer(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    Eigen::VectorXd diff = to_node->dual_arm_joint_pos_ - from_node->dual_arm_joint_pos_;

    double scale_factor = std::sqrt(from_node->dual_arm_joint_pos_.size());

    Eigen::VectorXd new_dual_arm_joint_pos = from_node->dual_arm_joint_pos_ + 
        std::min(joint_steer_step_size_ * scale_factor, diff.norm()) * diff.normalized();

    Node::Ptr new_node = std::make_shared<Node>();
    dual_arm_->splitTwoArmJointPos(new_dual_arm_joint_pos, new_node->arm_0_joint_pos_, new_node->arm_1_joint_pos_);

    if(!constrainConfig(new_node, from_node)){
        return nullptr;
    }
    updateNode(new_node);
    
    ROS_DEBUG_STREAM("oneStepSteer(): generate new node.");

    return new_node;
}


// ------------------------------------------------------------
std::vector<Node::Ptr> CBiRRT::pathInterpolation(
    const std::vector<Node::Ptr> &path_list
){
    std::vector<Node::Ptr> interpolated_path;

    for(size_t i = 0; ros::ok() && i < path_list.size()-1; i++){

        Eigen::VectorXd joint_pos_from = path_list[i]->dual_arm_joint_pos_;
        Eigen::VectorXd joint_pos_to = path_list[i+1]->dual_arm_joint_pos_;

        Eigen::VectorXd diff = joint_pos_to - joint_pos_from;
        Eigen::VectorXd diff_normalized = diff.normalized();

        double scale_factor = std::sqrt(joint_pos_from.size());
        int num_step = int(diff.norm() / scale_factor / path_interpolation_step_size_);

        interpolated_path.push_back(path_list[i]);

        // linear interpolation between from_node and to_node (not including from_node and to_node）
        for(size_t i = 1; i <= num_step; i++){
            Eigen::VectorXd joint_pos = joint_pos_from + 
                i * path_interpolation_step_size_ * diff_normalized * scale_factor;

            Node::Ptr temp_node = std::make_shared<Node>();
            dual_arm_->splitTwoArmJointPos(joint_pos, temp_node->arm_0_joint_pos_, temp_node->arm_1_joint_pos_);

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
        ROS_ERROR_COND(!RosUtils::checkGeometryPoseInitialized(req.arm_0_goal_pose_), "The goal pose of tcp hasn't been assigned.");
        ROS_ERROR_COND(!RosUtils::checkGeometryPoseInitialized(req.arm_1_goal_pose_), "The goal pose of tcp hasn't been assigned.");
    }else if(req.goal_type_ == "joint_pos"){
        b_goal_is_tcp_pose = false;
        ROS_ERROR_COND(req.arm_0_goal_joint_pos_.size() != dual_arm_->arm_0_->joint_num_, "The number of the joints of goal configuration is wrong.");
        ROS_ERROR_COND(req.arm_1_goal_joint_pos_.size() != dual_arm_->arm_1_->joint_num_, "The number of the joints of goal configuration is wrong.");
    }else{
        ROS_ERROR_STREAM_COND(req.goal_type_ != "joint_pos", ALGORITHM_NAME << "can only deal with the goal types of 'joint_pos' or 'tcp_pose'.");
    }
    ROS_ERROR_COND(req.arm_0_start_joint_pos_.size() != dual_arm_->arm_0_->joint_num_, "The number of the joints of start configuration is wrong.");
    ROS_ERROR_COND(req.arm_1_start_joint_pos_.size() != dual_arm_->arm_1_->joint_num_, "The number of the joints of start configuration is wrong.");

    req_ = req;
    
    // start and goal node
    Node::Ptr start_node = std::make_shared<Node>();
    start_node->arm_0_joint_pos_ = Utils::stdVector2EigenVectorXd(req.arm_0_start_joint_pos_);
    start_node->arm_1_joint_pos_ = Utils::stdVector2EigenVectorXd(req.arm_1_start_joint_pos_);
    start_node->note_ = "start";
    updateNode(start_node);
    ROS_ERROR_COND(checkNodeCollision(start_node), "The start node is in collision.");
    
    Node::Ptr goal_node = std::make_shared<Node>();
    goal_node->note_ = "goal";
    if(b_goal_is_tcp_pose){
        goal_node->arm_0_tcp_pose_ = RosUtils::rosPose2EigenPose(req.arm_0_goal_pose_);
        goal_node->arm_1_tcp_pose_ = RosUtils::rosPose2EigenPose(req.arm_1_goal_pose_);
    }else{
        goal_node->arm_0_joint_pos_ = Utils::stdVector2EigenVectorXd(req.arm_0_goal_joint_pos_);
        goal_node->arm_1_joint_pos_ = Utils::stdVector2EigenVectorXd(req.arm_1_goal_joint_pos_);
        ROS_ERROR_COND(checkNodeCollision(goal_node), "The goal node is in collision.");
        updateNode(goal_node);
    }

    if(req.b_visualize_start_goal_){
        visualizer_->publishText("start node");
        visualizer_->publishNode(start_node);
        ros::Duration(1).sleep();

        if(b_goal_is_tcp_pose){
            visualizer_->visual_tools_->publishAxisLabeled(req.arm_0_goal_pose_, "arm_0 goal pose");
            visualizer_->visual_tools_->publishAxisLabeled(req.arm_1_goal_pose_, "arm_1 goal pose");
            visualizer_->visual_tools_->trigger();
        }else{
            visualizer_->publishText("goal node");
            visualizer_->publishNode(goal_node);
        }
        ros::Duration(1).sleep();
        visualizer_->publishText("planning ...");
    }

    // setup the two trees
    std::vector<Node::Ptr> node_list_a{start_node};
    std::vector<Node::Ptr> node_list_b;
    if(!b_goal_is_tcp_pose){
        node_list_b.push_back(goal_node);
    }
    else{
        Node::Ptr goal_ik_node = generateGoalClosestIKNode(goal_node, /*ref_node*/start_node);
        node_list_b.push_back(goal_ik_node);

        if(req.b_visualize_planning_process_){
            visualizer_->publishText("goal_ik_node");
            visualizer_->publishNode(goal_ik_node);
            ros::Duration(2).sleep();
        }
    }
    std::vector<Node::Ptr> res_path_list;

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;
    ros::Rate rate(1);
    
    // rrt main loop
    t_begin = std::chrono::steady_clock::now();
    int iter = 0;
    while(ros::ok() && iter < rrt_max_iter_){
        // add new goal_ik_node with some probability
        if(b_goal_is_tcp_pose){
            auto &node_list_goal = (node_list_a[0]->note_ == "start") ? node_list_b : node_list_a;
            if(Utils::getRandomDouble() < new_goal_ik_probability_){
                Node::Ptr goal_ik_node = generateGoalIKNode(goal_node, /*ref_node*/nullptr);
                // add the new node to the goal tree only if the new node cannot be connected to the existing nodes
                Node::Ptr nearest_node = getNearestNode(node_list_goal, goal_ik_node, "joint_pos");
                if(!checkTwoNodeCanConnect(goal_ik_node, nearest_node)){
                    node_list_goal.push_back(goal_ik_node);
                }
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

            pathNode2Vector(res_path_list, res.arm_0_path_, res.arm_1_path_);
            
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
