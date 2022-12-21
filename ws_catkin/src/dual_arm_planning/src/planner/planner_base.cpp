#include "dual_arm_planning/planner/planner_base.h"


namespace dual_arm_planning{



// -------------------------------------------------------
PlannerBase::PlannerBase(
    const ros::NodeHandle& nh, 
    const Scene::Ptr &scene
): nh_(nh)
{
    scene_ = scene;
    dual_arm_ = scene_->dual_arm_;

    loadCommonParams();
}

// -------------------------------------------------------
void PlannerBase::setVisualizer(Visualize::Ptr &visualizer)
{
    visualizer_ = visualizer;
}


// -------------------------------------------------------
void PlannerBase::loadCommonParams()
{
    std::string param_name;

    param_name = "planner_configs/common/pose_distance/pos_weight";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, pose_dist_pos_weight_);

    param_name = "planner_configs/common/pose_distance/rot_weight";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, pose_dist_rot_weight_);

    param_name = "planner_configs/common/path_interpolation_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, path_interpolation_step_size_);
}


// -------------------------------------------------------
void PlannerBase::randomSampleJointPos(
    Eigen::VectorXd &arm_0_joint_pos,
    Eigen::VectorXd &arm_1_joint_pos
){
    while(ros::ok()){
        arm_0_joint_pos = dual_arm_->arm_0_->randomJointPos();
        arm_1_joint_pos = dual_arm_->arm_1_->randomJointPos();

        if(!scene_->checkRobotCollision(arm_0_joint_pos, arm_1_joint_pos)){
            break;
        }
    }
}


// -------------------------------------------------------
void PlannerBase::updateNode(
    const Node::Ptr &node
){
    updateNodeDualArmJointPos(node);
    if(b_node_links_pos_) updateNodeLinksPos(node);
    if(b_node_tcp_pose_) updateNodeTcpPose(node);
}


// -------------------------------------------------------
void PlannerBase::updateNodeDualArmJointPos(
    const Node::Ptr &node
){
    Eigen::VectorXd dual_arm_joint_pos(node->arm_0_joint_pos_.size() + node->arm_1_joint_pos_.size());

    dual_arm_joint_pos.block(0, 0, node->arm_0_joint_pos_.size(), 1) = node->arm_0_joint_pos_;
    dual_arm_joint_pos.block(node->arm_0_joint_pos_.size(), 0, node->arm_1_joint_pos_.size(), 1) = node->arm_1_joint_pos_;

    node->dual_arm_joint_pos_ = dual_arm_joint_pos;
}



// -------------------------------------------------------
void PlannerBase::updateNodeTcpPose(
    const Node::Ptr &node
){
    node->arm_0_tcp_pose_ = dual_arm_->arm_0_->getTcpPose(node->arm_0_joint_pos_);
    node->arm_1_tcp_pose_ = dual_arm_->arm_1_->getTcpPose(node->arm_1_joint_pos_);
}


// -------------------------------------------------------
void PlannerBase::updateNodeLinksPos(
    const Node::Ptr &node
){
    node->arm_0_links_pos_ = dual_arm_->arm_0_->getLinksPos(node->arm_0_joint_pos_, dual_arm_->arm_0_->critical_link_names_);
    node->arm_1_links_pos_ = dual_arm_->arm_1_->getLinksPos(node->arm_1_joint_pos_, dual_arm_->arm_1_->critical_link_names_);
}


// -------------------------------------------------------
Node::Ptr PlannerBase::randomSampleNode()
{
    Node::Ptr rand_node = std::make_shared<Node>();

    Eigen::VectorXd arm_0_random_joint_pos, arm_1_random_joint_pos;
    randomSampleJointPos(rand_node->arm_0_joint_pos_, rand_node->arm_1_joint_pos_);
    updateNode(rand_node);

    rand_node->note_ = "rand";
    
    ROS_DEBUG_STREAM("rand node arm_0 joint pos: " << rand_node->arm_0_joint_pos_.transpose() << 
        ", rand node arm_1 joint pos: " << rand_node->arm_1_joint_pos_.transpose());

    return rand_node;
}


// -------------------------------------------------------
double PlannerBase::twoNodeDistance(
    const Node::Ptr &node_0,
    const Node::Ptr &node_1,
    std::string dist_metric
){
    if(dist_metric == "joint_pos"){
        return (node_0->arm_0_joint_pos_ - node_1->arm_0_joint_pos_).norm() 
            + (node_0->arm_1_joint_pos_ - node_1->arm_1_joint_pos_).norm();
    }
    else if(dist_metric == "weighted_joint_pos"){
        double arm_0_dist = dual_arm_->arm_0_->weightedJointPosDistance(node_0->arm_0_joint_pos_, node_1->arm_0_joint_pos_);
        double arm_1_dist = dual_arm_->arm_1_->weightedJointPosDistance(node_0->arm_1_joint_pos_, node_1->arm_1_joint_pos_);
        return arm_0_dist + arm_1_dist;
    }
    else if(dist_metric == "links_pos"){
        double arm_0_dist = dual_arm_->arm_0_->linksPosDistance(node_0->arm_0_links_pos_, node_1->arm_0_links_pos_);
        double arm_1_dist = dual_arm_->arm_1_->linksPosDistance(node_0->arm_1_links_pos_, node_1->arm_1_links_pos_);
        return arm_0_dist + arm_1_dist;
    }
    else if(dist_metric == "tcp_pose"){
        double arm_0_dist = Utils::distanceBetweenTwoPose(
            node_0->arm_0_tcp_pose_, node_1->arm_0_tcp_pose_, pose_dist_pos_weight_, pose_dist_rot_weight_);
        double arm_1_dist = Utils::distanceBetweenTwoPose(
            node_0->arm_1_tcp_pose_, node_1->arm_1_tcp_pose_, pose_dist_pos_weight_, pose_dist_rot_weight_);
        return arm_0_dist + arm_1_dist;
    }
    else{
        ROS_ERROR_STREAM("twoNodeDistance(): invalid distance metric: " << dist_metric);
        throw "Shut down the program.";
    }
}


// -------------------------------------------------------
Node::Ptr PlannerBase::getNearestNode(
    const std::vector<Node::Ptr> &node_list,
    const Node::Ptr &target_node,
    const std::string &dist_metric
){
    // linear search
    double min_dist = 1e10;
    int min_idx = 0;
    for(size_t i = 0; i < node_list.size(); i++){
        double dist = twoNodeDistance(target_node, node_list[i], dist_metric);
        if(dist < min_dist){
            min_dist = dist;
            min_idx = i;
        }
    }
    return node_list[min_idx];
}


// -------------------------------------------------------
bool PlannerBase::checkNodeCollision(const Node::Ptr &node)
{
    // ROS_DEBUG_STREAM("checkNodeCollision(): collision ? : " 
    //     << scene_->checkRobotCollision(node->arm_0_joint_pos_, node->arm_1_joint_pos_));
    // ROS_DEBUG_STREAM("checkNodeCollision(): arm_0 out of joint bound ? : " 
    //     << !dual_arm_->arm_0_->checkJointPositionSatisfyBound(node->arm_0_joint_pos_));
    // ROS_DEBUG_STREAM("checkNodeCollision(): arm_1 out of joint bound ? : " 
    //     << !dual_arm_->arm_1_->checkJointPositionSatisfyBound(node->arm_1_joint_pos_));

    return (scene_->checkRobotCollision(node->arm_0_joint_pos_, node->arm_1_joint_pos_)) 
        || (!dual_arm_->arm_0_->checkJointPositionSatisfyBound(node->arm_0_joint_pos_))
        || (!dual_arm_->arm_1_->checkJointPositionSatisfyBound(node->arm_1_joint_pos_));
}



// -------------------------------------------------------
bool PlannerBase::checkTwoNodePathCollision(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    Eigen::VectorXd joint_pos_from = from_node->dual_arm_joint_pos_;
    Eigen::VectorXd joint_pos_to = to_node->dual_arm_joint_pos_;

    Eigen::VectorXd diff = joint_pos_to - joint_pos_from;
    Eigen::VectorXd diff_normalized = diff.normalized();

    double scale_factor = std::sqrt(diff.size());
    int num_step = int(diff.norm() / scale_factor / path_collision_check_step_size_);

    // linear interpolation between from_node and to_node (not including from_node and to_node）
    for(size_t i = 1; i <= num_step; i++){
        Eigen::VectorXd joint_pos = joint_pos_from + 
            i * path_collision_check_step_size_ * diff_normalized * scale_factor;

        Node::Ptr temp_node = std::make_shared<Node>();
        dual_arm_->splitTwoArmJointPos(joint_pos, temp_node->arm_0_joint_pos_, temp_node->arm_1_joint_pos_);

        if(checkNodeCollision(temp_node)){
            ROS_DEBUG_STREAM("checkTwoNodePathCollision(): collision ? : " << true);
            return true;
        }
    }

    ROS_DEBUG_STREAM("checkTwoNodePathCollision(): collision ? : " << false);
    return false;
}


// -------------------------------------------------------
bool PlannerBase::checkTwoNodeCanConnect(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    bool no_collision = !checkTwoNodePathCollision(from_node, to_node);

    double scale_factor = std::sqrt(from_node->dual_arm_joint_pos_.size());
    bool close_enough = (twoNodeDistance(from_node, to_node, "joint_pos") / scale_factor) < connect_joint_max_dist_;

    return no_collision && close_enough;
}


// -------------------------------------------------------
Node::Ptr PlannerBase::oneStepSteer(
    const Node::Ptr &from_node,
    const Node::Ptr &to_node
){
    Eigen::VectorXd diff = to_node->dual_arm_joint_pos_ - from_node->dual_arm_joint_pos_;

    double scale_factor = std::sqrt(from_node->dual_arm_joint_pos_.size());

    Eigen::VectorXd new_dual_arm_joint_pos = from_node->dual_arm_joint_pos_ + 
        std::min(joint_steer_step_size_ * scale_factor, diff.norm()) * diff.normalized();

    Node::Ptr new_node = std::make_shared<Node>();
    dual_arm_->splitTwoArmJointPos(new_dual_arm_joint_pos, new_node->arm_0_joint_pos_, new_node->arm_1_joint_pos_);
    updateNode(new_node);
    
    ROS_DEBUG_STREAM("oneStepSteer(): generate new node.");

    return new_node;
}


// -------------------------------------------------------
Node::Ptr PlannerBase::extend(
    std::vector<Node::Ptr> &node_list,
    const Node::Ptr &from_node,
    const Node::Ptr &to_node,
    bool greedy
){
    int step = 0;
    Node::Ptr s_node = from_node;
    Node::Ptr s_node_old = from_node;

    while(ros::ok() && (greedy || step < extend_max_steps_)){
        if(checkTwoNodeCanConnect(s_node, to_node)){
            return s_node;
        }

        if(twoNodeDistance(s_node, to_node, "joint_pos") > twoNodeDistance(s_node_old, to_node, "joint_pos")){
            return s_node_old;
        }

        s_node_old = s_node;

        // one step steer
        s_node = oneStepSteer(s_node, to_node);

        bool node_valid = (s_node != nullptr) 
            && !checkNodeCollision(s_node) && checkTwoNodeCanConnect(s_node_old, s_node);

        if(node_valid){
            s_node->parent_ = s_node_old;
            node_list.push_back(s_node);
        }
        else{
            ROS_DEBUG_STREAM("extend(): the node by one step steer is invalid.");
            return s_node_old;
        }
        
        step++;
    }

    ROS_DEBUG_STREAM("extend(): have done max steer steps.");
    return s_node;
}


// ------------------------------------------------------------
std::vector<Node::Ptr> PlannerBase::pathExtract(
    const Node::Ptr &node_forward_end, 
    const Node::Ptr &node_backward_end
){
    std::vector<Node::Ptr> path_list;

    // tree from the start
    Node::Ptr node = node_forward_end;
    while(node != nullptr){
        path_list.push_back(node);
        node = node->parent_;
    }
    std::reverse(path_list.begin(), path_list.end());

    // tree from the goal
    node = node_backward_end;
    while(node != nullptr){
        path_list.push_back(node);
        node = node->parent_;
    }

    return path_list;
}


// ------------------------------------------------------------
void PlannerBase::pathSmooth(
    std::vector<Node::Ptr> &path_list
){
    int iter = 0;
    while(ros::ok() && iter < path_smooth_max_iter_)
    {
        std::vector<Node::Ptr> smooth_path_list;
        std::vector<Node::Ptr> path_shortcut;

        if (path_list.size() < 3){
            return;
        }

        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_int_distribution<int> rand_i(0, path_list.size() - 3); // [a, b]
        int i = rand_i(eng);
        std::uniform_int_distribution<int> rand_j(i + 2, path_list.size() - 1);
        int j = rand_j(eng);

        Node::Ptr node_reached = extend(path_shortcut, path_list[i], path_list[j], /*greedy*/true);
        
        if(checkTwoNodeCanConnect(node_reached, path_list[j]) &&
                path_shortcut.size() < j-i-1){
                    
            for(int idx = 0; idx <= i; idx++){
                smooth_path_list.push_back(path_list[idx]);
            }
            for(int idx = 0; idx < path_shortcut.size(); idx++){
                smooth_path_list.push_back(path_shortcut[idx]);
            }
            for(int idx = j; idx < path_list.size(); idx++){
                smooth_path_list.push_back(path_list[idx]);
            }

            path_list = smooth_path_list;
            ROS_DEBUG_STREAM("pathSmooth(): iter = " << iter << ", path_length = " << path_list.size());
        }
        iter++;
    }
}


// ------------------------------------------------------------
std::vector<Node::Ptr> PlannerBase::pathInterpolation(
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

            interpolated_path.push_back(temp_node);
        }
    }

    return interpolated_path;
}


// -------------------------------------------------------
void PlannerBase::pathNode2Vector(
    const std::vector<Node::Ptr> &path_list,
    std::vector<std::vector<double> > &path_arm_0_joint_pos,
    std::vector<std::vector<double> > &path_arm_1_joint_pos
){
    path_arm_0_joint_pos.clear();
    path_arm_1_joint_pos.clear();

    for(auto node: path_list){
        path_arm_0_joint_pos.push_back(Utils::eigenVectorXd2StdVector(node->arm_0_joint_pos_));
        path_arm_1_joint_pos.push_back(Utils::eigenVectorXd2StdVector(node->arm_1_joint_pos_));
    }
}


// ------------------------------------------------------------
void PlannerBase::swapTrees(
    std::vector<Node::Ptr> &node_list_a,
    std::vector<Node::Ptr> &node_list_b
){
    auto tmp_list = node_list_a;
    node_list_a = node_list_b;
    node_list_b = tmp_list;
}


// ------------------------------------------------------------
void PlannerBase::swapNodes(
    Node::Ptr &node_a,
    Node::Ptr &node_b
){
    auto tmp_node = node_a;
    node_a = node_b;
    node_b = tmp_node;
}


} // end namespace
