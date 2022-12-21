#ifndef ARM_PLANNING_PLANNER_BASE_H
#define ARM_PLANNING_PLANNER_BASE_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"


namespace arm_planning{


class PlannerBase
{
public:
    typedef std::shared_ptr<PlannerBase> Ptr;

    PlannerBase(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void setVisualizer(Visualize::Ptr &visualizer);

    void loadCommonParams();

    virtual void loadParams() = 0;

    virtual Eigen::VectorXd randomSampleJointPos();

    virtual Node::Ptr randomSampleNode();

    virtual double twoNodeDistance(
        const Node::Ptr &node_0,
        const Node::Ptr &node_1,
        std::string dist_metric
    );

    virtual Node::Ptr getNearestNode(
        const std::vector<Node::Ptr> &node_list,
        const Node::Ptr &target_node,
        const std::string &dist_metric
    );

    virtual bool checkNodeCollision(const Node::Ptr &node);

    virtual bool checkTwoNodePathCollision(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    virtual bool checkTwoNodeCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    virtual Node::Ptr oneStepSteer(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    virtual Node::Ptr extend(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        bool greedy = false
    );

    virtual std::vector<Node::Ptr> pathExtract(
        const Node::Ptr &node_forward_end, 
        const Node::Ptr &node_backward_end
    );

    virtual void pathSmooth(
        std::vector<Node::Ptr> &path_list
    );

    /**
     * @brief linear interpolation in the joint space
    */
    virtual std::vector<Node::Ptr> pathInterpolation(
        const std::vector<Node::Ptr> &path_list
    );

    virtual void pathNode2Vector(
        const std::vector<Node::Ptr> &path_list,
        std::vector<std::vector<double> > &path_joint_pos
    );

    virtual bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    ) = 0;

    virtual void updateNodeTcpPose(
        const Node::Ptr &node
    );

    virtual void updateNodeLinksPos(
        const Node::Ptr &node
    );

    virtual void swapTrees(
        std::vector<Node::Ptr> &node_list_a,
        std::vector<Node::Ptr> &node_list_b
    );

    virtual void swapNodes(
        Node::Ptr &node_a,
        Node::Ptr &node_b
    );

    virtual bool checkTwoNodeTcpPoseCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    
public:
    ros::NodeHandle nh_;

    Scene::Ptr scene_;
    Robot::Ptr robot_;
    Visualize::Ptr visualizer_;

    // parameters
    int rrt_max_iter_;
    int extend_max_steps_;
    int path_smooth_max_iter_;
    double path_collision_check_step_size_;
    double joint_steer_step_size_;
    std::string dist_metric_;
    double connect_joint_max_dist_;
    double connect_link_max_dist_;
    double connect_tcp_max_dist_;
    double pose_dist_pos_weight_, pose_dist_rot_weight_;

    bool b_node_tcp_pose_ = false;
    bool b_node_links_pos_ = false;

    double path_interpolation_step_size_;
    
    


}; // end class


} // end namespace

#endif