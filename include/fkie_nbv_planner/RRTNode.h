#ifndef RRTNODE_H
#define RRTNODE_H

#include <sstream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "fkie_nbv_planner/NBVParameters.hpp"

namespace fkie_nbv_planner
{
  class RRTNode : std::enable_shared_from_this<RRTNode>
  {
  public:
    int node_id_;                                       // Unique node identity
    std::weak_ptr<RRTNode> parent_Wp_;                  // parent node in the tree
    std::vector<std::shared_ptr<RRTNode>> children_Sp_; // children nodes
    double cubature_best_yaw = 0.0;                     // save the best yaw computed from gain cubature (TODO: move somewhere else?)

  public:
    RRTNode();

    ~RRTNode() = default;

    /**
     * @brief Returns the pose of the node gain
     */
    geometry_msgs::Pose getPose() const;

    /**
     * @brief Set the pose of the node gain
     */
    bool setPose(const geometry_msgs::Pose pose);
    bool setPose(const Eigen::Vector4d pose);
    bool setPose(const Eigen::Vector3d pose);

    /**
     * @brief Set cost till parent and cost till root
     */
    void setTreeCosts();
    /**
     * @brief Returns the node gain, which depend on free space and measurement values
     */
    double getGain() const;

    /**
     * @brief Returns the node gain depending upon the free space
     */
    double getGainFreeSpace() const;

    /**
     * @brief Returns the node best yaw depending upon the free space
     */
    double getCubatureBestYaw() const;

    /**
     * @brief Returns the node cost, which depend on its "distance" to parten/root
     */
    double getCost() const;
    double getCostTillRoot() const;
    double getCostToParent() const;
    /**
     * @brief Get the computed score for the node, proportional to cost and gain.
     */
    double getScore() const;

    /**
     * @brief Set the node's gain based on [free_space_gain] and [measurement_gain]
     */
    void setGain(const double free_space_gain, const double measurement_gain, const double visited_gain);

    /**
     * @brief Set the node's orientation
     */
    void setOrientation(const geometry_msgs::Quaternion orientation);

    /**
     * @brief Compute and update the score of the node
     */
    void computeScore();

    /**
     * @brief Create a string representation of an RRTNode
     */
    std::string toString(const bool show_costs = true, const bool show_utility = true) const;

    /**
     * @brief Convert the node into an [geometry_msgs::Pose] object
     */
    geometry_msgs::Pose toPose() const;

    /**
     * @brief Convert the node into an [geometry_msgs::PoseStamped] object
     */
    geometry_msgs::PoseStamped toPoseStamped() const;

    /**
     * @brief Copy current object into a new RRTNode object
     */
    RRTNode copyToRRTNode() const;

  private:
    NBVParameters *params = NULL;

    geometry_msgs::Pose pose_; // Pose of the node

    double gain_ = 0.0;              // total gain of the node, proportional to [gain_free_space_] and [gain_measurement_]
    double gain_free_space_ = 0.0;   // gain proportional to free space
    double gain_measurement_ = 0.0;  // gain proportional to the measurement mean value
    double gain_visited_cell_ = 0.0; // gain assigned to a previously visited node (possible negative)

    double cost_ = 0.0;           // Total cost computed based on [cost_till_root_] and [cost_to_parent_]
    double cost_till_root_ = 0.0; // cost for moving until root node
    double cost_to_parent_ = 0.0; // cost for moving from parent to node

    double score_ = 0.0; // total score

  private:
    /**
     * @brief Compute the score based on original [aeplanner] paper
     */
    void compute_score_aeplanner();

    /**
     * @brief Compute the score based on linear equation considering only free space and lambda
     */
    void compute_score_linear();

    /**
     * @brief Compute the score based on the weighted-sum optimization of free space and measurement estimation
     */
    void compute_score_weighted_sum();

    /**
     * @brief Compute euclidean distance from the node to [parent] node
     */
    double euclidean_dist(std::shared_ptr<RRTNode> parent);

    template <typename T>
    std::string to_string(const T a_value, const int precision = 5) const;
  };

} // namespace fkie_nbv_planner
#endif // RRTNODE_H