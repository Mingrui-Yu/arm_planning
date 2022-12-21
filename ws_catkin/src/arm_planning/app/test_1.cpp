#include "arm_planning/common_include.h"
#include "arm_planning/robot.h"
#include "arm_planning/scene.h"
#include "arm_planning/visualize.h"

using namespace arm_planning;

/** @brief 
 * robot: panda
 * aim: test Class: Robot, Scene, Visualize
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
    std::string GROUP_NAME = "panda_manipulator";


    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    // planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();


    Robot::Ptr robot = std::make_shared<Robot>(nh, ROBOT_DESCRIPTION, GROUP_NAME);
    robot->printRobotInfo();

    Scene::Ptr scene = std::make_shared<Scene>(nh, robot, planning_scene_monitor::LockedPlanningSceneRO(psm));
    Visualize::Ptr visualizer = std::make_shared<Visualize>(nh, scene);


    const moveit::core::JointModelGroup* joint_model_group = 
        robot->robot_state_->getJointModelGroup(GROUP_NAME);

    ros::Rate rate(1);
    while(ros::ok()){
        robot->robot_state_->setToRandomPositions(joint_model_group);

        std::vector<double> joint_values;
        robot->robot_state_->copyJointGroupPositions(joint_model_group, joint_values);

        bool collision = scene->checkRobotCollision(Utils::stdVector2EigenVectorXd(joint_values));

        visualizer->publishPlanningScene(Utils::stdVector2EigenVectorXd(joint_values));
        visualizer->publishText(collision ? "collision" : "no collision");
        rate.sleep();

        // test IK
        const Eigen::Isometry3d& end_effector_state = robot->robot_state_->getGlobalLinkTransform(robot->arm_ee_link_name_);

        Eigen::VectorXd ik_joint_pos;
        bool success = robot->armEndEffectorIK(Utils::stdVector2EigenVectorXd(joint_values), 
            end_effector_state, ik_joint_pos);
        // std::cout << "ik success: " << success << std::endl;
        // std::cout << "ik_joint_pos: " << ik_joint_pos.transpose() << std::endl;

        visualizer->publishPlanningScene(ik_joint_pos);
        visualizer->publishText("IK solution");
        rate.sleep();
    }
    

    ros::Duration(0.2).sleep();

    return 0;
}