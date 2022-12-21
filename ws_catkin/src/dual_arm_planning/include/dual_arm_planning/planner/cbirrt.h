#ifndef DUAL_ARM_PLANNING_CBIRRT_H
#define DUAL_ARM_PLANNING_CBIRRT_H

#include <arm_planning/common_include.h>

#include "dual_arm_planning/planning_request.h"
#include "dual_arm_planning/planning_response.h"
#include "dual_arm_planning/scene.h"
#include "dual_arm_planning/dual_arm.h"
#include "dual_arm_planning/visualize.h"
#include "dual_arm_planning/node.h"
#include "dual_arm_planning/planner/planner_base.h"


namespace dual_arm_planning{

class CBiRRT: public PlannerBase
{
public:
    typedef std::shared_ptr<CBiRRT> Ptr;

    CBiRRT(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    Node::Ptr generateGoalIKNode(
        const Node::Ptr &goal_node,
        const Node::Ptr &ref_node
    );

    Node::Ptr generateGoalClosestIKNode(
        const Node::Ptr &goal_node,
        const Node::Ptr &ref_node
    );

    Eigen::VectorXd constraintError(
        const Eigen::VectorXd &pose,  // pos + rpy angle
        const Eigen::VectorXd &pose_lb,
        const Eigen::VectorXd &pose_ub
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

    PlanningRequest req_;

    double new_goal_ik_probability_;
    double constraint_error_thres_;
    int constraint_projection_max_iter_;


}; // end class


} // end namespace

#endif