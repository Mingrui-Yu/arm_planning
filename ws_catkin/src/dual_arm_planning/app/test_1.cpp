#include <arm_planning/common_include.h>

#include "dual_arm_planning/dual_arm.h"
#include "dual_arm_planning/scene.h"
#include "dual_arm_planning/visualize.h"


using namespace dual_arm_planning;

/** @brief 
 * dual_ur
 * 测试 DualArm, Scene, Visualize
 */


int main(int argc, char** argv){

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // change the logger level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;


    std::string ROBOT_DESCRIPTION = "robot_description";

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    // planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();


    DualArm::Ptr dual_arm = std::make_shared<DualArm>(nh, ROBOT_DESCRIPTION, "arm_0", "arm_1", "dual_arm");
    dual_arm->arm_0_->printRobotInfo();
    dual_arm->arm_1_->printRobotInfo();

    Scene::Ptr scene = std::make_shared<Scene>(nh, dual_arm, planning_scene_monitor::LockedPlanningSceneRO(psm));

    ROS_DEBUG("Create scene.");

    Visualize::Ptr visualizer = std::make_shared<Visualize>(nh, scene);

    ROS_DEBUG("Create visualizer.");


    const moveit::core::JointModelGroup* arm_0_joint_model_group = 
        dual_arm->robot_state_->getJointModelGroup("arm_0");
    const moveit::core::JointModelGroup* arm_1_joint_model_group = 
        dual_arm->robot_state_->getJointModelGroup("arm_1");


    ros::Rate rate(1);
    while(ros::ok()){
        Eigen::VectorXd arm_0_joint_pos = dual_arm->arm_0_->randomJointPos();
        Eigen::VectorXd arm_1_joint_pos = dual_arm->arm_1_->randomJointPos();

        dual_arm->arm_0_->robot_state_->setJointGroupPositions(arm_0_joint_model_group, Utils::eigenVectorXd2StdVector(arm_0_joint_pos));
        dual_arm->arm_1_->robot_state_->setJointGroupPositions(arm_1_joint_model_group, Utils::eigenVectorXd2StdVector(arm_1_joint_pos));

        bool collision = scene->checkRobotCollision(arm_0_joint_pos, arm_1_joint_pos);

        visualizer->publishPlanningScene(arm_0_joint_pos, arm_1_joint_pos);
        visualizer->publishText(collision ? "collision" : "no collision");
        rate.sleep();

        // test IK
        const Eigen::Isometry3d& arm_0_ee_pose = 
            dual_arm->arm_0_->robot_state_->getGlobalLinkTransform(dual_arm->arm_0_->arm_ee_link_name_);
        const Eigen::Isometry3d& arm_1_ee_pose = 
            dual_arm->arm_1_->robot_state_->getGlobalLinkTransform(dual_arm->arm_1_->arm_ee_link_name_);

        Eigen::VectorXd arm_0_ik_joint_pos, arm_1_ik_joint_pos;

        // bool success = dual_arm->dualArmEndEffectorIK(
        //                             arm_0_joint_pos, arm_1_joint_pos,
        //                             arm_0_ee_pose, arm_1_ee_pose,
        //                             arm_0_ik_joint_pos, arm_1_ik_joint_pos);
        // std::cout << "success: " << success <<  std::endl;

        bool arm_0_success = dual_arm->arm_0_->armEndEffectorIK(arm_0_joint_pos, arm_0_ee_pose, arm_0_ik_joint_pos);
        bool arm_1_success = dual_arm->arm_1_->armEndEffectorIK(arm_1_joint_pos, arm_1_ee_pose, arm_1_ik_joint_pos);
        std::cout << "arm_0_success: " << arm_0_success << ", arm_1_success: " << arm_1_success << std::endl;
        
        visualizer->publishPlanningScene(arm_0_ik_joint_pos, arm_1_ik_joint_pos);
        visualizer->publishText("IK solution");
        rate.sleep();
    }
    






    ros::Duration(0.2).sleep();

    return 0;
}