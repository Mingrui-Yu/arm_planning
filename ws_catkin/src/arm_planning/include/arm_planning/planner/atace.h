#ifndef ARM_PLANNING_ATACE_H
#define ARM_PLANNING_ATACE_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class ATACE: public PlannerBase
{
public:
    typedef std::shared_ptr<ATACE> Ptr;

    ATACE(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    std::string extendToTcpPose(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        Node::Ptr &reached_node,
        bool greedy
    );

    Node::Ptr oneStepSteerToTcpPose(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    Eigen::VectorXd constraintProjection(
        const Eigen::VectorXd &movement
    );

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "ATACE";

    // parameters
    double pos_steer_step_size_;
    double angle_steer_step_size_;



}; // end class


} // end namespace

#endif