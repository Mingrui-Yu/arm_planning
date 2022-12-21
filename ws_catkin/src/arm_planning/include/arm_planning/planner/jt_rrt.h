#ifndef ARM_PLANNING_JTRRT_H
#define ARM_PLANNING_JTRRT_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class JTRRT: public PlannerBase
{
public:
    typedef std::shared_ptr<JTRRT> Ptr;

    JTRRT(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    bool checkTwoNodeTcpPoseCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    Node::Ptr extendToTcpPose(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        bool greedy = false
    );

    Node::Ptr oneStepSteerToTcpPose(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "JTRRT";

    // parameters
    double goal_bias_probability_;
    double pos_steer_step_size_;
    double angle_steer_step_size_;
    std::string jacobian_steer_type_;



}; // end class


} // end namespace

#endif