#include "fkie_nbv_planner/RRTNode.h"

namespace fkie_nbv_planner
{
  RRTNode::RRTNode()
  {
    params = &NBVParameters::getInstance(); // get params singletone
  }

  geometry_msgs::Pose RRTNode::getPose() const
  {
    return pose_;
  }

  bool RRTNode::setPose(const geometry_msgs::Pose pose)
  {
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.w))
    {
      pose_ = geometry_msgs::Pose();
      return false;
    }

    pose_ = pose;
    return true;
  }

  bool RRTNode::setPose(const Eigen::Vector4d pose)
  {
    pose_ = geometry_msgs::Pose();
    if (!std::isfinite(pose[0]) || !std::isfinite(pose[1]) || !std::isfinite(pose[2]))
      return false;

    pose_.position.x = pose[0];
    pose_.position.y = pose[1];
    pose_.position.z = pose[2];

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, pose[3]);

    pose_.orientation.x = quat.x();
    pose_.orientation.y = quat.y();
    pose_.orientation.z = quat.z();
    pose_.orientation.w = quat.w();

    return true;
  }

  bool RRTNode::setPose(const Eigen::Vector3d pose)
  {
    pose_ = geometry_msgs::Pose();
    if (!std::isfinite(pose[0]) || !std::isfinite(pose[1]) || !std::isfinite(pose[2]))
      return false;

    pose_.position.x = pose[0];
    pose_.position.y = pose[1];
    pose_.position.z = pose[2];

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);

    pose_.orientation.x = quat.x();
    pose_.orientation.y = quat.y();
    pose_.orientation.z = quat.z();
    pose_.orientation.w = quat.w();

    return true;
  }

  double RRTNode::getGain() const
  {
    return this->gain_;
  }

  double RRTNode::getGainFreeSpace() const
  {
    return this->gain_free_space_;
  }

  double RRTNode::getCubatureBestYaw() const
  {
    return this->cubature_best_yaw;
  }

  double RRTNode::getCost() const
  {
    return this->cost_;
  }

  double RRTNode::getCostTillRoot() const
  {
    return this->cost_till_root_;
  }

  double RRTNode::getCostToParent() const
  {
    return this->cost_to_parent_;
  }

  double RRTNode::getScore() const
  {
    return this->score_;
  }

  void RRTNode::setTreeCosts()
  {
    this->cost_to_parent_ = 0.0;
    this->cost_till_root_ = 0.0;

    std::shared_ptr<RRTNode> parent = this->parent_Wp_.lock();
    if (parent)
    {
      this->cost_to_parent_ = this->euclidean_dist(parent);
      this->cost_till_root_ = this->cost_to_parent_ + parent->cost_till_root_;
    }

    ROS_DEBUG_STREAM("Set cost parent to new node:" << this->cost_to_parent_);
    ROS_DEBUG_STREAM("Set cost root to new node:" << this->cost_till_root_);
  }

  void RRTNode::setGain(const double free_space_gain, const double measurement_gain, const double visited_gain)
  {
    if (std::abs(free_space_gain) > 1.0)
      ROS_WARN_STREAM("RRTNode: [free_space_gain] > 1.0: free_space_gain: " << free_space_gain << " - " << this->toString());

    if (std::abs(measurement_gain) > 1.0)
      ROS_WARN_STREAM("RRTNode: [measurement_gain] > 1.0: measurement_gain: " << measurement_gain << " - " << this->toString());

    if (std::abs(visited_gain) > 1.0)
      ROS_WARN_STREAM("RRTNode: [visited_gain] > 1.0: visited_gain: " << visited_gain << " - " << this->toString());

    this->gain_free_space_ = free_space_gain;
    this->gain_measurement_ = measurement_gain;
    this->gain_visited_cell_ = visited_gain;

    double gain_m = (params->utility_weight_measurement * this->gain_measurement_);
    double gain_fs = (params->utility_weight_free_space * this->gain_free_space_);
    double gain_v = (params->utility_weight_visited_cell * this->gain_visited_cell_);
    // ROS_INFO("*********");
    // ROS_INFO_STREAM("params->utility_weight_visited_cell" << params->utility_weight_visited_cell);
    // sum up all gain components
    this->gain_ = 0.0;
    this->gain_ = gain_m + gain_fs + gain_v;

    // ROS_INFO_STREAM("Freespace gain: " << gain_fs << ", Measurement gain: " << gain_m << ", Visited gain: " << gain_v << ", Total gain:" << this->gain_);
  }

  void RRTNode::setOrientation(const geometry_msgs::Quaternion orientation)
  {
    this->pose_.orientation = orientation;
  }

  void RRTNode::computeScore()
  {
    this->score_ = (0.0);

    //compute_score_aeplanner();
    //compute_score_linear();
    compute_score_weighted_sum();
  }

  std::string RRTNode::toString(const bool show_costs, const bool show_utility) const
  {
    std::string t;
    t += "id:[" + std::to_string(node_id_) + "] ";
    t += "(" + to_string(pose_.position.x, 2) + "," + to_string(pose_.position.y, 2) + "," + to_string(pose_.position.z, 2) + ") ";
    if (show_utility)
    {
      t += "gain: [" + to_string(gain_free_space_, 3) + "] ";
      t += "score: [" + to_string(score_, 3) + "] ";
    }

    else if (show_costs)
    {
      t += "cost_till_root: [" + to_string(cost_till_root_, 3) + "] ";
    }
    return t;
  }

  geometry_msgs::Pose RRTNode::toPose() const
  {
    return pose_;
  }

  geometry_msgs::PoseStamped RRTNode::toPoseStamped() const
  {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = params->world_frame;
    goal.pose = getPose();
    return goal;
  }

  RRTNode RRTNode::copyToRRTNode() const
  {
    RRTNode n;
    n.node_id_ = node_id_;
    n.pose_ = pose_;
    n.gain_ = getGain();
    n.score_ = getScore();
    return n;
  }

  void RRTNode::compute_score_aeplanner()
  {
    // Score from the aeplanner planner
    // ROS_INFO_STREAM("0. Current Node: (" << this->pose_[0] << "," << this->pose_[1] << "," << this->pose_[2] << ")");
    std::shared_ptr<RRTNode> parent = this->parent_Wp_.lock();
    if (parent)
    {
      this->cost_ = this->euclidean_dist(parent);
      this->score_ = parent->score_ + getGain() * exp(-params->utility_lambda * this->euclidean_dist(parent));
    }
    else
    {
      this->cost_ = 0.0;
      this->score_ = getGain();
    }
  }

  void RRTNode::compute_score_linear()
  {
    std::shared_ptr<RRTNode> parent = this->parent_Wp_.lock();
    if (parent)
    {
      this->cost_to_parent_ = this->euclidean_dist(parent);
      this->cost_till_root_ = this->cost_to_parent_ + parent->cost_till_root_;
      this->cost_ = this->cost_to_parent_;
      this->score_ = (getGain() - params->utility_lambda * this->cost_to_parent_) + parent->score_;
    }
    else
    {
      this->cost_ = 0.0;
      this->score_ = getGain();
    }
  }

  void RRTNode::compute_score_weighted_sum()
  {
    std::shared_ptr<RRTNode> parent = this->parent_Wp_.lock();

    this->score_ = 0.0;

    if (parent)
    {
      // branch node
      this->cost_to_parent_ = this->euclidean_dist(parent);
      this->cost_till_root_ = this->cost_to_parent_ + parent->cost_till_root_;

      this->cost_ = this->cost_till_root_;
      this->score_ = getGain() - params->utility_weight_euclidean_cost * getCost();
      // this->score_ += parent->score_; // Accumulate the score of the parent
    }
    else
    {
      // Root node
      this->cost_ = 0.0;
      this->score_ += params->utility_weight_measurement * this->gain_measurement_; // Add contribution of the measurement
      this->score_ += params->utility_weight_free_space * this->gain_free_space_;   // Add contribution of free space cells
    }
  }

  double RRTNode::euclidean_dist(std::shared_ptr<RRTNode> parent)
  {
    Eigen::Vector3d diff(
        getPose().position.x - parent->getPose().position.x,
        getPose().position.y - parent->getPose().position.y,
        getPose().position.z - parent->getPose().position.z);
    return diff.norm();
  }

  template <typename T>
  std::string RRTNode::to_string(const T a_value, const int precision) const
  {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << a_value;
    return out.str();
  }

}