#ifndef ARM_PLANNING_CBIRRT_H
#define ARM_PLANNING_CBIRRT_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class CBiRRT: public PlannerBase
{
public:
    typedef std::shared_ptr<CBiRRT> Ptr;

    CBiRRT(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    void addRandomGoalIKNode(
        const Node::Ptr &goal_node,
        std::vector<Node::Ptr> &node_list_goal
    );

    Eigen::VectorXd constraintError(
        const Eigen::VectorXd &pose_vec  // pos + rpy angle
    );

    bool constrainConfig(
        Node::Ptr &node,
        const Node::Ptr &node_old
    );

    Node::Ptr oneStepSteer(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    std::vector<Node::Ptr> pathInterpolation(
        const std::vector<Node::Ptr> &path_list
    );

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "CBiRRT";

    double new_goal_ik_probability_;
    double constraint_error_thres_;
    int constraint_projection_max_iter_;



}; // end class


} // end namespace

#endif