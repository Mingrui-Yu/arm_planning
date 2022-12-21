#include "arm_planning/robot.h"


namespace arm_planning{

// -------------------------------------------------------
Robot::Robot(
    const ros::NodeHandle& nh,
    const std::string &robot_description_name,
    const std::string &group_name
): nh_(nh)
{
    arm_group_name_ = group_name;

    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
        new robot_model_loader::RobotModelLoader(robot_description_name)); // The RobotModelLoader should be kept around. Reference: https://github.com/ros-planning/moveit/issues/2979#issuecomment-984440339

    robot_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_loader_->getModel()));
    robot_state_->setToDefaultValues();

    const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);
    joint_num_ = joint_model_group->getActiveVariableCount();

    loadParams();
}


// -------------------------------------------------------
Robot::Robot(
    const ros::NodeHandle& nh,
    moveit::core::RobotStatePtr &robot_state,
    const std::string &group_name
): nh_(nh)
{
    arm_group_name_ = group_name;
    robot_state_ = robot_state;

    const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);
    joint_num_ = joint_model_group->getActiveVariableCount();

    loadParams();
}


// -------------------------------------------------------
void Robot::loadParams(
){
    std::string param_name;
    std::string group_name = arm_group_name_;

    param_name = "robot_configs/" + group_name + "/ee_link_name";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, arm_ee_link_name_);

    std::vector<double> position, quaternion;
    param_name = "robot_configs/" + group_name + "/tcp_in_ee_pose/position";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, position);

    param_name = "robot_configs/" + group_name + "/tcp_in_ee_pose/orientation";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, quaternion);

    tcp_in_ee_pose_ = Utils::stdPosQuatVec2Isometry3d(position, quaternion);

    param_name = "robot_configs/" + group_name + "/joint_sample_lb";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
    nh_.getParam(param_name, joint_sample_lb_);
    ROS_ERROR_STREAM_COND(joint_sample_lb_.size() != joint_num_, "The dimension of joint_sample_lb doesn't match the group name");

    param_name = "robot_configs/" + group_name + "/joint_sample_ub";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
    nh_.getParam(param_name, joint_sample_ub_);
    ROS_ERROR_STREAM_COND(joint_sample_ub_.size() != joint_num_, "The dimension of " << param_name <<  " doesn't match the group name");

    param_name = "robot_configs/" + group_name + "/joint_pos_weight";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
    nh_.getParam(param_name, joint_pos_weight_);
    ROS_ERROR_STREAM_COND(joint_pos_weight_.size() != joint_num_, "The dimension of " << param_name <<  " doesn't match the group name");

    param_name = "robot_configs/" + group_name + "/critical_link_names";
    ROS_WARN_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << "doesn't exist.");
    nh_.getParam(param_name, critical_link_names_);
}


// -------------------------------------------------------
void Robot::printRobotInfo()
{
    std::cout << "--------------- Robot Info ----------------- " << std::endl;

    std::cout << "group name: " << arm_group_name_ << std::endl;

    const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(arm_group_name_);

    std::cout << "getVariableCount(): " << joint_model_group->getVariableCount() << std::endl;

    std::cout << "getActiveVariableCount(): " << joint_model_group->getActiveVariableCount() << std::endl;

    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    std::cout << "getVariableNames(): ";
    Utils::coutStdVector(joint_names);

    std::cout << "------------------------------------------- " << std::endl;
}



// ------------------------------------------------------------
bool Robot::checkArmJointNum(
    const Eigen::VectorXd &joint_value
){
    bool valid = (joint_value.size() == joint_num_);
    ROS_ERROR_COND(valid == false, "The number of joints doesn't match the group name.");
    return valid;
}

// ------------------------------------------------------------
bool Robot::checkArmJointNum(
    const double &num
){
    bool valid = (num == joint_num_);
    ROS_ERROR_COND(valid == false, "The number of joints doesn't match the group name.");
    return valid;
}


// ------------------------------------------------------------
Eigen::VectorXd Robot::randomJointPos()
{
    Eigen::VectorXd random_joint_pos(joint_num_);
    for(size_t j = 0; j < random_joint_pos.size(); j++){
        random_joint_pos[j] = Utils::getRandomDouble(joint_sample_lb_[j], joint_sample_ub_[j]);
    }
    return random_joint_pos;
}


// ------------------------------------------------------------
bool Robot::armTcpRandomIK(
    const Eigen::Isometry3d &target_pose,
    Eigen::VectorXd &result_joint_pos
){  
    return armTcpIK(randomJointPos(), target_pose, result_joint_pos);
}


// ------------------------------------------------------------
bool Robot::armTcpIK(
    const Eigen::VectorXd &ref_joint_pos,
    const Eigen::Isometry3d &target_pose,
    Eigen::VectorXd &result_joint_pos
){  
    return armEndEffectorIK(ref_joint_pos, tcpInBaseToEEInBase(target_pose), result_joint_pos);
}


// ------------------------------------------------------------
bool Robot::armEndEffectorIK(
    const Eigen::VectorXd &ref_joint_pos,
    const Eigen::Isometry3d &target_pose,
    Eigen::VectorXd &result_joint_pos
){  
    checkArmJointNum(ref_joint_pos);

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(arm_group_name_);

    // initial value for IK solving
    robot_state_->setJointGroupPositions(
        joint_model_group, Utils::eigenVectorXd2StdVector(ref_joint_pos));

    // solve the IK problem
    double timeout = 0.1;
    moveit::core::GroupStateValidityCallbackFn ik_check_callback = 
        boost::bind(&Robot::validateIKSolution, this, _1, _2, _3);

    bool found_ik = robot_state_->setFromIK(joint_model_group, target_pose, 
        arm_ee_link_name_, timeout, ik_check_callback);

    // get the solution
    if(found_ik){
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
        result_joint_pos = Utils::stdVector2EigenVectorXd(joint_values);

        // ROS_INFO_STREAM("armEndEffectorIK(): ik solution: " << result_joint_pos.transpose());
        // ROS_INFO_STREAM("armEndEffectorIK(): ik solution satisfies the joint bound ? : " << checkJointPositionSatisfyBound(result_joint_pos));

        return true;
    }
    else{
        // ROS_INFO_STREAM("armEndEffectorIK(): failed");
        return false;
    }
}


// -------------------------------------------------------
bool Robot::validateIKSolution( 
    robot_state::RobotState* robot_state, 
    const robot_state::JointModelGroup* joint_group, 
    const double* joint_group_variable_value
){
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_value);

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_group, joint_values);

    return checkJointPositionSatisfyBound(Utils::stdVector2EigenVectorXd(joint_values));
}


// ------------------------------------------------------------
Eigen::Isometry3d Robot::eeInBaseToTcpInBase(
    const Eigen::Isometry3d &pose_ee
){
    return pose_ee * tcp_in_ee_pose_;
}


// ------------------------------------------------------------
Eigen::Isometry3d Robot::tcpInBaseToEEInBase(
    const Eigen::Isometry3d &pose_tcp
){
    return pose_tcp * tcp_in_ee_pose_.inverse();
}


// ------------------------------------------------------------
void Robot::getJointNames(
    std::vector<std::string> &joint_names
){
    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(arm_group_name_);
    joint_names = joint_model_group->getVariableNames();
}


// ------------------------------------------------------------
Eigen::Isometry3d Robot::getLinkPose(
    const std::string &link_name,
    const Eigen::VectorXd &joint_pos
){
    checkArmJointNum(joint_pos);

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(arm_group_name_);

    robot_state_->setJointGroupPositions(
        joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

    Eigen::Isometry3d link_pose = robot_state_->getGlobalLinkTransform(link_name);

    return link_pose;
}


// ------------------------------------------------------------
Eigen::Isometry3d Robot::getTcpPose(
    const Eigen::VectorXd &joint_pos
){
    return eeInBaseToTcpInBase(getLinkPose(arm_ee_link_name_, joint_pos));
}


// ------------------------------------------------------------
VecEigenVec3 Robot::getLinksPos(
    const Eigen::VectorXd &joint_pos,
    const std::vector<std::string> &link_names 
){
    checkArmJointNum(joint_pos);

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(arm_group_name_);

    robot_state_->setJointGroupPositions(
        joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

    VecEigenVec3 links_pos(link_names.size());
    for(size_t i = 0; i < link_names.size(); i++){
        links_pos[i] = robot_state_->getGlobalLinkTransform(link_names[i]).translation();
    }

    return links_pos;
}


// ------------------------------------------------------------
Eigen::MatrixXd Robot::getJacobianMatrix(
    const std::string &link_name,
    const Eigen::VectorXd &joint_pos,
    Eigen::Vector3d reference_point_position
){
    checkArmJointNum(joint_pos);

    const moveit::core::JointModelGroup* joint_model_group = 
        robot_state_->getJointModelGroup(arm_group_name_);
    
    robot_state_->setJointGroupPositions(
        joint_model_group, Utils::eigenVectorXd2StdVector(joint_pos));

    // return the jacobian matrix of the link in the base frame
    Eigen::MatrixXd jacobian;
    robot_state_->getJacobian(joint_model_group,
        robot_state_->getLinkModel(link_name),
        reference_point_position, jacobian);

    return jacobian;
}


// ------------------------------------------------------------
Eigen::MatrixXd Robot::getTcpJacobianMatrix(
    const Eigen::VectorXd &joint_pos
){
    return getJacobianMatrix(arm_ee_link_name_, joint_pos, tcp_in_ee_pose_.translation());
}


// ------------------------------------------------------------
bool Robot::checkJointPositionSatisfyBound(
    const Eigen::VectorXd &joint_pos
){
    checkArmJointNum(joint_pos);

    for(size_t i = 0; i < joint_pos.size(); i++){
        if(joint_pos[i] > joint_sample_ub_[i] || joint_pos[i] < joint_sample_lb_[i]){
            return false;
        }
    }

    return true;
}


// ------------------------------------------------------------
void Robot::boundJointPos(
    Eigen::VectorXd &joint_pos
){
    checkArmJointNum(joint_pos);

    for(size_t i = 0; i < joint_pos.size(); i++){
        joint_pos[i] = std::max(joint_sample_lb_[i], std::min(joint_pos[i], joint_sample_ub_[i]));
    }
}


// ------------------------------------------------------------
double Robot::weightedJointPosDistance(
    const Eigen::VectorXd &joint_pos_0,
    const Eigen::VectorXd &joint_pos_1
){
    Eigen::VectorXd diff = joint_pos_1 - joint_pos_0;

    // weight the differences of the joint angles
    for(size_t j = 0; j < diff.size(); j++){
        diff[j] *= joint_pos_weight_[j];
    }

    return diff.norm();
}


// ------------------------------------------------------------
double Robot::linksPosDistance(
    const VecEigenVec3 &links_pos_0,
    const VecEigenVec3 &links_pos_1
){
    return (Utils::stdVecEigenVec3ToEigenVectorXd(links_pos_0) -
            Utils::stdVecEigenVec3ToEigenVectorXd(links_pos_1)).norm();
}


// // ------------------------------------------------------------
// bool Robot::armTcpClosestIK(
//     const Eigen::VectorXd &ref_joint_pos,
//     const Eigen::Isometry3d &target_pose,
//     Eigen::VectorXd &result_joint_pos
// ){  
//     return armEndEffectorClosestIK(ref_joint_pos, tcpInBaseToEEInBase(target_pose), result_joint_pos);
// }


// // ------------------------------------------------------------
// bool Robot::armEndEffectorClosestIK(
//     const Eigen::VectorXd &ref_joint_pos,
//     const Eigen::Isometry3d &target_pose,
//     Eigen::VectorXd &result_joint_pos
// ){
//     const moveit::core::JointModelGroup* jmg = 
//         robot_state_->getJointModelGroup(arm_group_name_);

//     const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
//     if (!solver){
//         ROS_ERROR_NAMED("No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
//         return false;
//     }

//     Eigen::Isometry3d pose = target_pose;
//     // bring the pose to the frame of the IK solver
//     if (!robot_state_->setToIKSolverFrame(pose, solver))
//       return false;
//     // Convert Eigen pose to geometry_msgs pose
//     geometry_msgs::Pose ik_query = RosUtils::eigenPose2RosPose(target_pose);

//     // Bijection
//     const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();
//     std::vector<double> seed(bij.size());
//     for (std::size_t i = 0; i < bij.size(); ++i)
//         seed[i] = ref_joint_pos[bij[i]];

//     std::vector<double> ik_solution;
//     moveit_msgs::MoveItErrorCodes error_code;
//     if(!solver->getPositionIK(ik_query, seed, ik_solution, error_code)){
//         ROS_DEBUG("solver->getPositionIK() failed.");
//         return false;
//     }
//     result_joint_pos = Utils::stdVector2EigenVectorXd(ik_solution);
//     return true;
// }

} // end namespace


