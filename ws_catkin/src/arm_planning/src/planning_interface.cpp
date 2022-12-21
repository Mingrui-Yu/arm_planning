#include "arm_planning/planning_interface.h"

#include "arm_planning/planner/rrt.h"
#include "arm_planning/planner/rrt_connect.h"
#include "arm_planning/planner/jt_rrt.h"
#include "arm_planning/planner/ik_rrt.h"
#include "arm_planning/planner/atace.h"
#include "arm_planning/planner/cbirrt.h"

namespace arm_planning{

// -------------------------------------------------------
PlanningInterface::PlanningInterface(
    const ros::NodeHandle& nh,
    const std::string &robot_description_name
): nh_(nh)
{
    robot_description_name_ = robot_description_name;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader(robot_description_name_));

    // planning scene monitor
    psm_ = planning_scene_monitor::PlanningSceneMonitorPtr(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    psm_->startSceneMonitor("/move_group/monitored_planning_scene");
}


// -------------------------------------------------------
bool PlanningInterface::solve(
    std::string algorithm,
    const PlanningRequest &req,
    PlanningResponse &res
){
    robot_ = std::make_shared<Robot>(nh_, robot_description_name_, req.group_name_);
    // robot_->printRobotInfo();

    psm_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
    scene_ = std::make_shared<Scene>(nh_, robot_, lscene);

    visualizer_ = std::make_shared<Visualize>(nh_, scene_);

    if(algorithm == "RRT"){
        RRT::Ptr planner = std::make_shared<RRT>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else if(algorithm == "RRTConnect"){
        RRTConnect::Ptr planner = std::make_shared<RRTConnect>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else if(algorithm == "JTRRT"){
        JTRRT::Ptr planner = std::make_shared<JTRRT>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else if(algorithm == "IKRRT"){
        IKRRT::Ptr planner = std::make_shared<IKRRT>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else if(algorithm == "ATACE"){
        ATACE::Ptr planner = std::make_shared<ATACE>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else if(algorithm == "CBiRRT"){
        CBiRRT::Ptr planner = std::make_shared<CBiRRT>(nh_, scene_);
        planner->setVisualizer(visualizer_);
        planner->solve(req, res);
    }
    else{
        ROS_ERROR_STREAM("Invalid planning algorithm: " << algorithm);
    }
    

    return false;
}










} // end namespace