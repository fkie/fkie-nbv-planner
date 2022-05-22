#include <fkie_nbv_planner/NBVPlanner.h>

namespace fkie_nbv_planner
{
  NBVPlanner::NBVPlanner(const ros::NodeHandle &nh)
      : nh_(nh),
        tfListener(tfBuffer),
        planning_as_(nh_, "nbv_rrt", boost::bind(&NBVPlanner::executePlan, this, _1), false)
  {
    params = &NBVParameters::getInstance();

    std::string ns = ros::this_node::getNamespace();

    current_camera_pose_initialized_ = false;
    root_initialized_ = false;
    boundary_initialized_ = false;

    current_utility_max_value_free_space = params->utility_max_value_free_space;
    current_utility_max_value_measurement = params->utility_max_value_measurement;
    current_utility_max_value_visited_cell = params->utility_max_value_visited_cell;

    visited_grid.setGridSize(params->visited_cells_grid_size);

    // TODO: Lookup the poses through TF instead of subscribing to them
    camera_pose_sub_ = nh_.subscribe("camera_pose", 1, &NBVPlanner::cameraPoseCallback, this);
    octomap_sub_ = nh_.subscribe("octomap", 1, &NBVPlanner::octomapCallback, this);

    robot_footprint_sub_ = nh_.subscribe("robot_bbx", 1, &NBVPlanner::robotFootprintCallback, this);

    exploration_bbx_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("exploration_bbx_coords", 10, true);
    planning_poly_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("exploration_planning_poly_coords", 10, true);

    node_count = 0;
    planning_iteration = 0;

    planning_as_.start();
    ROS_INFO("Initialised NBVPlanner...");
  }

  void NBVPlanner::resetTree()
  {
    tree_adapter_.clear();
    kdtree_ = std::unique_ptr<kd_tree_nano>(new kd_tree_nano(3, tree_adapter_));
    node_count = 0;
  }

  void NBVPlanner::cameraPoseCallback(const geometry_msgs::PoseStamped &msg)
  {
    curr_camera_pose_ = msg.pose;
    current_camera_pose_initialized_ = true;
    visited_grid.addValue(PositionGrid(curr_camera_pose_), VisitedValue(true));
  }

  void NBVPlanner::octomapCallback(const octomap_msgs::Octomap &msg)
  {
    octomap::AbstractOcTree *aot = octomap_msgs::msgToMap(msg);
    {
      std::lock_guard<std::mutex> lockGuard(otMutex_);
      octomap::OcTree *ot = (octomap::OcTree *)aot;
      ot_ = std::make_shared<octomap::OcTree>(*ot);

      if (ot)
      {
        delete ot;
      }
    }
  }

  void NBVPlanner::robotFootprintCallback(const geometry_msgs::PolygonStamped &msg)
  {
    curr_robot_footprint = msg.polygon;
  }

  void NBVPlanner::updateGainCachedNodes()
  {
    ROS_INFO("Updating gain for cached nodes...");
    for (auto &node : cached_nodes_)
    {
      updateGain(node);
    }
  }

  void NBVPlanner::removeLowGainCacheNodes()
  {

    if (!cached_nodes_.empty())
    {
      size_t previous_size = cached_nodes_.size();

      // sort vector of nodes depending on its gain
      // ROS_INFO_STREAM("Sorting " << cached_nodes_.size() << " cached nodes in descending order..");
      std::sort(std::begin(cached_nodes_),
                std::end(cached_nodes_),
                [](auto const lhs, auto const rhs)
                { return lhs.getGain() > rhs.getGain(); });

      // keep only [max_num_cached_points_to_keep]
      if (cached_nodes_.size() > (size_t)params->max_num_cached_points_to_keep)
        cached_nodes_.erase(cached_nodes_.begin() + params->max_num_cached_points_to_keep, cached_nodes_.end());
      ROS_INFO_STREAM("Cached nodes previous size: " << previous_size << " - new size: " << cached_nodes_.size());

      double cached_node_worst_gain = cached_nodes_.back().getGain();
      min_gain_ = cached_node_worst_gain;
      // ROS_INFO_STREAM("Found cached node with worst gain value:" << cached_node_worst_gain);
      // ROS_INFO_STREAM("Found cached node with BEST gain value:" << cached_nodes_[0].getGain());
    }
  }

  // Find all the samples within a given radius E.g., https://github.com/jlblancoc/nanoflann/blob/master/examples/dynamic_pointcloud_example.cpp
  std::vector<std::shared_ptr<RRTNode>> NBVPlanner::findSamplesWithinRadius(const double &radius, const Eigen::Vector3d &sample)
  {
    ROS_INFO_STREAM("Step 4.0: Finding samples within the radius");
    std::vector<std::shared_ptr<RRTNode>> nodes;
    // Turn the sample into a point to query
    double query_pt[3] = {sample[0], sample[1], sample[2]};

    std::vector<std::pair<size_t, double>> indices_dists;
    nanoflann::RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

    // Find all the neighbors within the radius
    if (kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10)))
    {
      ROS_INFO_STREAM("[resultSet.size()] Points within radius:" << resultSet.size() << ", [indices_dists.size()] Points within radius:" << indices_dists.size());

      // ROS_INFO_STREAM("[Nanoflann] Current Nodes in RRT: " << tree_adapter_.kdtree_get_point_count() << " nodes");
      if (indices_dists.size() > 0)
      {
        for (size_t i = 0; i < indices_dists.size(); ++i)
        {
          nodes.push_back(tree_adapter_.nodes[indices_dists[i].first]);
          // ROS_INFO_STREAM("Node within radius: " << tree_adapter_.nodes[indices_dists[i].first]->toString());
        }
      }
    }
    else
    {
      ROS_INFO("No neighbors found within the radius");
    }

    return nodes;
  }

  std::vector<std::shared_ptr<RRTNode>> NBVPlanner::findKNearestNeighbors(const int &k)
  {
    std::vector<std::shared_ptr<RRTNode>> nodes;
    return nodes;
  }

  void NBVPlanner::executePlan(const fkie_nbv_planner::NbvPlannerGoalConstPtr &goal)
  {
    ROS_INFO_STREAM("******* [NBV Planner] Received a new request to plan *******");

    planning_iteration += 1;
    // ROS_INFO_STREAM("1) Current planning iteration: " << planning_iteration);
    fkie_nbv_planner::NbvPlannerResult result;
    result.request_base_pose = false;
    result.complete_exploration = false;

    if (!octomapReceived())
    {
      ROS_ERROR("Octomap has been received yet");
      planning_as_.setSucceeded(result);
      return;
    }

    if (!current_camera_pose_initialized_)
    {
      ROS_ERROR("Robot's camera pose not yet received. Make sure it is being published and correctly mapped");
      planning_as_.setSucceeded(result);
      return;
    }

    if (!initializeBoundary(goal->current_boundary))
    {
      ROS_ERROR("Could not initialize boundary");
      planning_as_.setSucceeded(result);
      return;
    }
    else
    {
      publishBoundaryBBx();
    }

    // initialize measurement grid
    measurement_grid.setGridSize(goal->estimations.grid_size);
    for (auto v : goal->estimations.values)
    {
      PositionGrid p(v.x, v.y, v.z);
      MeasurementValue ev(v.mean, v.variance);
      measurement_grid.addValue(p, ev);
    }
    measurement_grid.publishToRVIZ();

    // publish current visited grid
    visited_grid.publishToRVIZ();

    // ROS_INFO("===== [1] START: NBV planning =====");
    // Do not accept any new octomap updates while planning is in progress..
    std::lock_guard<std::mutex> lockGuard(otMutex_);

    // Calculate the time taken to answer a request
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    max_node_score = 0.0; // reset max score

    // Update gain for all cached nodes
    updateGainCachedNodes();

    // remove cached nodes with lower gain
    removeLowGainCacheNodes();

    // Get the current frontiers
    std::vector<RRTNode> curr_frontiers;
    RRTNode temp_node;

    // Frontiers are cached nodes either with high combined gain (free-space + measurement) or only free-space
    if (cached_nodes_.size() > 0)
    {
      if ((int)cached_nodes_.size() >= params->num_frontiers)
      {
        for (int i = 0; i < params->num_frontiers; i++)
        {
          if (params->free_space_frontier)
          {
            // Choose frontiers only if they are above minimum frontier gain
            if (cached_nodes_[i].getGainFreeSpace() > params->min_gain_frontier)
            {
              temp_node = cached_nodes_[i];

              // Make sure, the yaw is free space yaw and not measurement yaw
              double yaw = cached_nodes_[i].getCubatureBestYaw();
              geometry_msgs::Quaternion node_orientation;
              tf2::Quaternion quat;
              quat.setRPY(0.0, 0.0, yaw);
              node_orientation.x = quat.x();
              node_orientation.y = quat.y();
              node_orientation.z = quat.z();
              node_orientation.w = quat.w();
              temp_node.setOrientation(node_orientation);

              curr_frontiers.push_back(temp_node);
            }
          }
          else
          {
            // Choose frontiers only if they are above minimum frontier gain
            if (cached_nodes_[i].getGain() > params->min_gain_frontier)
            {
              curr_frontiers.push_back(cached_nodes_[i]);
            }
          }
        }
        std::reverse(curr_frontiers.begin(), curr_frontiers.end());
      }
      else
      {
        for (size_t i = 0; i < cached_nodes_.size(); i++)
        {
          if (params->free_space_frontier)
          {
            // Choose frontiers only if they are above minimum frontier gain
            if (cached_nodes_[i].getGainFreeSpace() > params->min_gain_frontier)
            {
              temp_node = cached_nodes_[i];
              // Make sure, the yaw is free space yaw and not measurement yaw
              double yaw = cached_nodes_[i].getCubatureBestYaw();
              geometry_msgs::Quaternion node_orientation;
              tf2::Quaternion quat;
              quat.setRPY(0.0, 0.0, yaw);
              node_orientation.x = quat.x();
              node_orientation.y = quat.y();
              node_orientation.z = quat.z();
              node_orientation.w = quat.w();
              temp_node.setOrientation(node_orientation);
              curr_frontiers.push_back(temp_node);
            }
          }
          else
          {
            // Choose frontiers only if they are above minimum frontier gain
            if (cached_nodes_[i].getGain() > params->min_gain_frontier)
            {
              curr_frontiers.push_back(cached_nodes_[i]);
            }
          }
        }
        std::reverse(curr_frontiers.begin(), curr_frontiers.end());
      }

      if (curr_frontiers.size() > 0)
      {
        // for (size_t i = 0; i < curr_frontiers.size(); i++)
        // {
        //     ROS_INFO_STREAM("Current frontier " << i << " :" << curr_frontiers[i].toString());
        // }

        // Find the best frontier based on the current robot pose
        found_best_frontier_ = false;
        found_best_frontier_ = findHighUtilityFrontier(curr_frontiers, best_frontier_);

        if (!found_best_frontier_)
        {
          ROS_INFO_STREAM("Found best frontier: " << best_frontier_.toString());
        }
        else
        {
          ROS_WARN_STREAM("No best frontiers found but there are frontiers still available.");
        }
      }
      else
      {
        ROS_ERROR_STREAM("curr_frontiers.size() > 0");
      }
    }
    else
    {
      ROS_WARN("No cached nodes, No frontiers..");
    }

    // Grow the tree and compute the best branch
    best_branch_ = getNbvBranch(curr_frontiers);

    // If best branch has been computed
    if (best_branch_.size() != 0)
    {
      result.goals = best_branch_;
      result.request_base_pose = false;
    }
    // [No best branch] If best branch does not exist, then check if there are frontiers
    else
    {
      ROS_WARN("No best branch found");

      // No best node from RRT and no current frontiers then exploration is completed
      if (curr_frontiers.size() == 0)
      {
        ROS_ERROR("No frontiers found. Exploration completed..");
        result.request_base_pose = false;
        result.complete_exploration = true;
      }

      // If no frontiers and best branch does not exist
      else
      {
        if (found_best_frontier_)
        {
          result.goal_pose_3d = best_frontier_.toPoseStamped();
          // ROS_INFO_STREAM("[2] Assigned new best frontier: " << best_frontier_.toString());
          ROS_WARN_STREAM("No best branch found. Giving 3D Pose/best frontier: (" << result.goal_pose_3d.pose.position.x << "," << result.goal_pose_3d.pose.position.y << "," << result.goal_pose_3d.pose.position.z << ")");
          result.request_base_pose = true;

          auto compare = [&result](const fkie_nbv_planner::RRTNode &element)
          {
            return result.goal_pose_3d.pose.position.x == element.getPose().position.x && result.goal_pose_3d.pose.position.y == element.getPose().position.y && result.goal_pose_3d.pose.position.z == element.getPose().position.z;
          };

          std::remove_if(cached_nodes_.begin(), cached_nodes_.end(), compare);
        }
        else
        {
          ROS_ERROR("No best frontier found. Exploration completed.. :/");
          result.request_base_pose = false;
          result.complete_exploration = true;
        }
      }
    }

    // remove cached nodes with lower gain
    removeLowGainCacheNodes();

    if (result.request_base_pose)
      visited_grid.addValue(PositionGrid(result.goal_pose_3d.pose), VisitedValue(true));

    // publish cached nodes to RVIZ
    nbv_publisher.pubCachedNodes(cached_nodes_, max_node_score);

    // Optimize the path if required (enabled via parameters)
    result.goals = p_optimizer.optimizePath(result.goals, ot_);
    nbv_publisher.pubPath(result.goals, "optimized_path", 0.4, 0.05, 1.0, 0.1);

    // ROS_INFO("===== [1] END: NBV planning =====");
    ROS_INFO("====================");
    ROS_INFO_STREAM("[ARM] Total goals sent:" << result.goals.size());
    ROS_INFO_STREAM("[BASE] Move the robot base: " << std::boolalpha << bool(result.request_base_pose));
    ROS_INFO("====================");
    // ROS_INFO_STREAM("REMAINING GOALS best_branch:" << best_branch_.size());
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    planning_as_.setSucceeded(result);
    double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    ROS_INFO_STREAM("******** Request completed in " << time_ms << "[ms], " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s] **************");
  }

  bool NBVPlanner::findHighUtilityFrontier(const std::vector<RRTNode> &frontiers, RRTNode &best_frontier)
  {
    bool found = false;
    // ROS_INFO("Finding high utility frontier...");
    double closest_distance = inf_double;
    visualization_msgs::MarkerArray marker_array;

    // get current pose of the robot
    geometry_msgs::PoseStamped p_input;
    p_input.header.frame_id = params->robot_sampling_frame;
    geometry_msgs::PoseStamped robot_position = transformPose(p_input, params->world_frame, params->robot_sampling_frame);

    for (size_t i = 0; i < frontiers.size(); i++)
    {
      geometry_msgs::PoseStamped p_frontier;
      p_frontier.pose = frontiers[i].getPose();
      double curr_distance = distancePoseStamped(robot_position, p_frontier);

      // ROS_INFO_STREAM("Robot: " << PositionGrid(robot_position.pose).toString() << " - frontier: " << frontiers[i].toString() << ", Distance:" << curr_distance);

      // Best frontier has the shortest distance
      if (curr_distance < closest_distance && curr_distance > params->arm_goal_range)
      {
        closest_distance = curr_distance;
        best_frontier = frontiers[i];
        found = true;
      }

      // Create frontier with utility markers
      nbv_publisher.addFrontierMarkers(frontiers[i], i, curr_distance, marker_array);
    }

    // ROS_WARN_STREAM("Best frontier: " << best_frontier.toString() << " closest_distance " << closest_distance);

    // Publish the best frontier for visualisation in blue and all the other frontiers with their utility in light blue
    nbv_publisher.curr_frontiers_rviz_pub_.publish(marker_array);
    nbv_publisher.pubBestFrontier(best_frontier.getPose());
    return found;
  }

  std::vector<geometry_msgs::PoseStamped> NBVPlanner::extractNBVPoses()
  {
    // Get the best branch and send the result
    std::vector<std::shared_ptr<RRTNode>> sub_tree = getBestNodeBranch(best_node_);

    std::vector<geometry_msgs::PoseStamped> goals;
    ROS_INFO_STREAM("RRT complete best branch has " << sub_tree.size() << " nodes. Filtering goals with gain below:" << min_gain_);
    visualization_msgs::MarkerArray marker_array_BN_branch;
    for (size_t i = 0; i < sub_tree.size(); i++)
    {
      // Trim NBV branch and remove nodes from the best branch below min gain
      if (sub_tree[i]->getGain() > min_gain_)
      {
        geometry_msgs::PoseStamped goal = sub_tree[i]->toPoseStamped();
        Eigen::Vector3d sampling_frame(0, 0, 0);
        geometry_msgs::PoseStamped p_tf = transformPose(goal, params->robot_sampling_frame, params->world_frame);
        Eigen::Vector3d nbv_pose(p_tf.pose.position.x, p_tf.pose.position.y, p_tf.pose.position.z);

        // Check if the pose is within the reachable same of the arm
        Eigen::Vector3d diff((sampling_frame - nbv_pose));

        // Check if the pose is within arms reach
        if (diff.norm() < params->arm_goal_range)
        {
          goals.push_back(goal);
          ROS_INFO_STREAM("Found NBV Goal: " << i << ": " << sub_tree[i]->toString());

          // Color the best node pink
          if (sub_tree[i]->node_id_ == best_node_->node_id_)
          {
            nbv_publisher.addNodeMarkers(sub_tree[i], sub_tree[i]->node_id_, false, true, true, marker_array_BN_branch, true);
          }
          else
          {
            nbv_publisher.addNodeMarkers(sub_tree[i], sub_tree[i]->node_id_, false, false, true, marker_array_BN_branch, true);
          }

          // Color the best branch edges as green
          nbv_publisher.addEdgeMarkers(sub_tree[i], sub_tree[i]->node_id_, true, marker_array_BN_branch);
        }
      }
    }
    // ROS_DEBUG_STREAM("NBV Goals above min gain within arm reach:" << goals.size());

    nbv_publisher.best_branch_rviz_pub_.publish(marker_array_BN_branch);
    marker_array_BN_branch.markers.clear();
    return goals;
  }

  /*BSD 3-Clause License

  Copyright (c) 2019, Magnus Selin
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */
  std::pair<double, double> NBVPlanner::gainCubature(const geometry_msgs::Pose pot_pose) const
  {
    double gain = 0.0;

    // Camera parameters
    double camera_hfov = params->camera_hfov;
    double camera_vfov = params->camera_vfov;
    double raycast_dr = params->raycast_dr, raycast_dphi = params->raycast_dphi, raycast_dtheta = params->raycast_dtheta;
    double dphi_rad = M_PI * raycast_dphi / 180.0f, dtheta_rad = M_PI * raycast_dtheta / 180.0f;
    double r;
    int phi, theta;
    double phi_rad, theta_rad;

    std::map<int, double> gain_per_yaw;

    Eigen::Vector3d origin(pot_pose.position.x, pot_pose.position.y, pot_pose.position.z);
    Eigen::Vector3d vec, dir;

    for (theta = -180; theta < 180; theta += raycast_dtheta)
    {
      theta_rad = M_PI * theta / 180.0f;

      // Azimuth sample along the lower and upper angle of the vertical FOV
      for (phi = 90 - camera_vfov / 2; phi < 90 + camera_vfov / 2; phi += raycast_dphi)
      {
        phi_rad = M_PI * phi / 180.0f;

        double g = 0;
        for (r = params->gain_r_min; r < params->gain_r_max; r += raycast_dr)
        {
          vec[0] = pot_pose.position.x + r * cos(theta_rad) * sin(phi_rad);
          vec[1] = pot_pose.position.y + r * sin(theta_rad) * sin(phi_rad);
          vec[2] = pot_pose.position.z + r * cos(phi_rad);
          dir = vec - origin;

          Eigen::Vector3d v(vec[0], vec[1], vec[2]);

          // If query point not in the polygon, then do not count it as gain, if the unmapped area is within the robot do not count it as the gain
          if (!isSampleInPolygon(v, curr_boundary_.polygon.polygon, params->arm_height_max) or isSampleInPolygon(v, curr_robot_footprint, params->arm_height_max))
            break;

          if (!std::isfinite(vec[0]) || !std::isfinite(vec[1]) || !std::isfinite(vec[2]))
          {
            ROS_WARN_STREAM("gainCubature: vec is NULL");
            break;
          }

          if (!ot_)
            break;

          octomap::point3d query(vec[0], vec[1], vec[2]);
          octomap::OcTreeNode *result = ot_->search(query);

          // If result, then the node is known to be free or occupied
          if (result)
          {
            // Break if occupied so we don't count any information gain behind a wall.
            if (result->getLogOdds() > 0)
              break;
          }
          else
          {
            g += (2 * r * r * raycast_dr + 1 / 6 * raycast_dr * raycast_dr * raycast_dr) * dtheta_rad * sin(phi_rad) *
                 sin(dphi_rad / 2);
          }
        }

        gain += g;
        // Add the volume gain for each yaw
        gain_per_yaw[theta] += g;
      }
    }

    int best_yaw = 0;
    double best_yaw_score = 0;
    for (int yaw = -180; yaw < 180; yaw++)
    {

      double yaw_score = 0;
      for (int hfov_ = -camera_hfov / 2; hfov_ < camera_hfov / 2; hfov_++)
      {
        int theta = yaw + hfov_;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;

        yaw_score += gain_per_yaw[theta];
      }

      if (best_yaw_score < yaw_score)
      {
        best_yaw_score = yaw_score;
        best_yaw = yaw;
      }
    }

    gain = best_yaw_score;

    double yaw = M_PI * best_yaw / 180.f;
    // ROS_INFO_STREAM("Best yaw: " << yaw);

    return std::make_pair(gain, yaw);
  }

  std::vector<std::shared_ptr<RRTNode>> NBVPlanner::getBestNodeBranch(std::shared_ptr<RRTNode> node)
  {
    std::vector<std::shared_ptr<RRTNode>> sub_tree;
    std::shared_ptr<RRTNode> current_node(new RRTNode);
    current_node = node;
    bool root_reached = false;

    while (!root_reached)
    {
      // Check if the current node has a parent, if yes, add node to branch and set the parent as the current node
      if (current_node->parent_Wp_.lock())
      {
        sub_tree.push_back(current_node);
        current_node = current_node->parent_Wp_.lock();
      }
      else
      {
        root_reached = true;
      }
    }
    // Reverse the tree
    std::reverse(sub_tree.begin(), sub_tree.end());
    return sub_tree;
  }

  fkie_nbv_planner::NbvPlannerResult NBVPlanner::generateGoalsToSubFrontier(fkie_nbv_planner::NbvPlannerResult result)
  {
    ROS_INFO("Extracting reachable goals and subfrontier");
    int erase_upto = 0;
    for (size_t i = 0; i < best_branch_.size(); i++)
    {
      erase_upto = i;
      // Transform the nbv_pose from world to robot sampling frame
      Eigen::Vector3d sampling_frame(0, 0, 0);
      geometry_msgs::PoseStamped p_tf = transformPose(best_branch_[i], params->robot_sampling_frame, params->world_frame);
      Eigen::Vector3d nbv_pose(p_tf.pose.position.x, p_tf.pose.position.y, p_tf.pose.position.z);

      // Check if the pose is within the reachable same of the arm
      Eigen::Vector3d diff((sampling_frame - nbv_pose));
      // ROS_INFO_STREAM("[Best branch: " << i << " ]: (" << best_branch_[i].pose.position.x << "," << best_branch_[i].pose.position.y << "," << best_branch_[i].pose.position.z << ")");

      if (diff.norm() < params->arm_goal_range)
      {
        // ROS_INFO_STREAM("[Within ARM goal range:" << params->arm_goal_range << ", goal id:" << i << " ]: (" << best_branch_[i].pose.position.x << "," << best_branch_[i].pose.position.y << "," << best_branch_[i].pose.position.z << ")");
        // ROS_INFO_STREAM("******************");
        result.goals.push_back(best_branch_[i]);
      }
      else
      {
        // The pose is out of arm's reach, move the base
        result.request_base_pose = false;
        // result.goal_pose_3d = best_branch_[i];
        ROS_INFO_STREAM("Found a sub-frontier.");
        // ROS_INFO_STREAM("[Sub-frontier]:" << i << "(" << best_branch_[i].pose.position.x << "," << best_branch_[i].pose.position.y << "," << best_branch_[i].pose.position.z << ")");
        // ROS_INFO("******************");
        // nbv_publisher.pubSubFrontier(result.goal_pose_3d.pose);
        break;
      }
    }

    // Mark original path points as visited (assumes that arm execution is correct)
    for (const geometry_msgs::PoseStamped &ps : result.goals)
      visited_grid.addValue(PositionGrid(ps.pose), VisitedValue(true));

    // Optimize the path if required (enabled via parameters)
    // nbv_publisher.pubPath(result.goals, "original_path", 1.0, 0.0, 0.0, 0.06);
    // result.goals = p_optimizer.optimizePath(result.goals, ot_);
    // nbv_publisher.pubPath(result.goals, "optimized_path", 0.4, 0.05, 1.0, 0.1);

    // Mark Optimized path points as visited (assumes that arm execution is correct)
    for (const geometry_msgs::PoseStamped &ps : result.goals)
      visited_grid.addValue(PositionGrid(ps.pose), VisitedValue(true));

    best_branch_.erase(best_branch_.begin(), best_branch_.begin() + erase_upto + 1);
    // ROS_INFO_STREAM("Erased elements:" << erase_upto << ", Current size of best branch frontier is:" << best_branch_.size());
    ROS_INFO_STREAM("Found " << result.goals.size() << " goals within the arm range.");
    ROS_INFO_STREAM("Sent request to move base:" << bool(result.request_base_pose));
    return result;
  }

  bool NBVPlanner::getFrontiers(std::vector<RRTNode> &frontiers)
  {
    // This method assumes that the [cached_nodes] vector is sorted
    frontiers.clear();

    // Push back output frontiers
    for (size_t i = 0; i < cached_nodes_.size() && i < (size_t)params->num_frontiers; i++)
      frontiers.push_back(cached_nodes_[i]);

    // ROS_INFO_STREAM("Sorted RRT vector (nodes) ---------------------------------------------------");
    // for (auto a : cached_nodes_)
    //     ROS_INFO_STREAM(a->toString());

    // if (frontiers.size() > 0)
    // {
    //   ROS_INFO_STREAM("Current Frontiers ---------------------------------------------------");
    //   for (auto a : frontiers)
    //   {
    //     ROS_INFO_STREAM(a.toString());
    //   }
    // }

    return frontiers.size() > 0;
  }

  void NBVPlanner::updateGain(std::shared_ptr<RRTNode> node) const
  {
    // compute gain cubature
    std::pair<double, double> ret = gainCubature(node->getPose());
    node->cubature_best_yaw = ret.second;

    // get measurement gain
    MeasurementValue mv = measurement_grid.getValue(PositionGrid(node->getPose()));

    // get visited cell gain
    VisitedValue vv = visited_grid.getValue(PositionGrid(node->getPose()));

    // add max-normalized gain
    if (ret.first > current_utility_max_value_free_space)
      current_utility_max_value_free_space = ret.first;

    if (mv.getValue() > current_utility_max_value_measurement)
      current_utility_max_value_measurement = mv.getValue();

    if (vv.getValue() > current_utility_max_value_visited_cell)
      current_utility_max_value_visited_cell = vv.getValue();

    // ROS_INFO_STREAM("Before NORM: freespace gain: " << ret.first << ", MAX: " << current_utility_max_value_free_space << ", measurement gain: " << mv.getValue() << ", MAX: " << current_utility_max_value_measurement << ", visited gain: " << vv.getValue() << ", MAX:" << current_utility_max_value_visited_cell);

    node->setGain(ret.first / current_utility_max_value_free_space,
                  mv.getValue() / current_utility_max_value_measurement,
                  (vv.getValue() / current_utility_max_value_visited_cell) * -1.0);
  }

  // TODO: Combine two updateGain methods
  void NBVPlanner::updateGain(RRTNode &node) const
  {
    // compute gain cubature
    std::pair<double, double> ret = gainCubature(node.getPose());
    node.cubature_best_yaw = ret.second;

    // get measurement gain
    MeasurementValue mv = measurement_grid.getValue(PositionGrid(node.getPose()));

    // get visited cell gain
    VisitedValue vv = visited_grid.getValue(PositionGrid(node.getPose()));

    // add max-normalized gain
    if (ret.first > current_utility_max_value_free_space)
      current_utility_max_value_free_space = ret.first;

    if (mv.getValue() > current_utility_max_value_measurement)
      current_utility_max_value_measurement = mv.getValue();

    if (vv.getValue() > current_utility_max_value_visited_cell)
      current_utility_max_value_visited_cell = vv.getValue();

    // add max-normalized gain
    // ROS_INFO_STREAM("Before NORM Gain and normalisation values: freespace gain: " << ret.first << ", MAX: " << current_utility_max_value_free_space << ", measurement gain: " << mv.getValue() << ", MAX: " << current_utility_max_value_measurement << ", visited gain: " << vv.getValue() << ", MAX:" << current_utility_max_value_visited_cell);
    node.setGain(ret.first / current_utility_max_value_free_space,
                 mv.getValue() / current_utility_max_value_measurement,
                 (vv.getValue() / current_utility_max_value_visited_cell) * -1.0);
  }

  void NBVPlanner::computeYaw(std::shared_ptr<RRTNode> node)
  {
    // update yaw
    double yaw = 0.0;

    if (params->compute_yaw_from_free_space || measurement_grid.size() == 0)
    {
      yaw = node->cubature_best_yaw; // yaw computed as best-yaw from gain cubature
    }

    if (params->compute_yaw_from_measurement)
    {
      // yaw computed based on measurement gradient of neighbors
      std::vector<IndexGrid> indices;

      // get the indices for the neighbor voxels of the current node
      PositionGrid node_position(node->getPose());
      measurement_grid.getIndexNeighbors(node_position, 1, indices);

      // search the index with the highest gradient
      MeasurementValue reference_value = measurement_grid.getValue(PositionGrid(node->getPose()));
      double max_difference = 0.0;
      IndexGrid max_index;
      for (const IndexGrid &i : indices)
      {
        MeasurementValue m = measurement_grid.getValue(i);
        double difference = (m - reference_value).mean;

        if (difference > max_difference)
        {
          max_difference = difference;
          max_index = i;
        }
      }

      // if an index was found, compute the yaw based on their positions
      if (max_index.isValid())
      {
        PositionGrid position_max = measurement_grid.getPosition(max_index);
        yaw = std::atan2(position_max.y - node_position.y, position_max.x - node_position.x);
      }
    }

    geometry_msgs::Quaternion node_orientation;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    node_orientation.x = quat.x();
    node_orientation.y = quat.y();
    node_orientation.z = quat.z();
    node_orientation.w = quat.w();
    node->setOrientation(node_orientation);
  }

  void NBVPlanner::publishBoundaryBBx()
  {
    std::vector<double> x;
    std::vector<double> y;
    // ROS_INFO_STREAM("Received " << curr_boundary_.polygon.polygon.points.size() << " points for the boundary.. Publishing to Rviz");
    for (std::size_t i = 0; i < curr_boundary_.polygon.polygon.points.size(); ++i)
    {
      x.push_back(curr_boundary_.polygon.polygon.points[i].x);
      y.push_back(curr_boundary_.polygon.polygon.points[i].y);
      ROS_INFO_STREAM("Polygon coordinates: " << i << " (" << curr_boundary_.polygon.polygon.points[i].x << " ," << curr_boundary_.polygon.polygon.points[i].y << " ," << curr_boundary_.polygon.polygon.points[i].z);
    }

    double poly_min[3] = {*std::min_element(x.begin(), x.end()), *std::min_element(y.begin(), y.end()), params->arm_height_min};
    double poly_max[3] = {*std::max_element(x.begin(), x.end()), *std::max_element(y.begin(), y.end()), params->arm_height_max};

    // ROS_INFO_STREAM("Bbx min: " << poly_min[0] << "," << poly_min[1] << "," << poly_min[2]);
    // ROS_INFO_STREAM("Bbx max: " << poly_max[0] << "," << poly_max[1] << "," << poly_max[2]);

    std_msgs::Float64MultiArray bbx;
    bbx.data.push_back(poly_min[0]);
    bbx.data.push_back(poly_min[1]);
    bbx.data.push_back(poly_min[2]);
    bbx.data.push_back(poly_max[0]);
    bbx.data.push_back(poly_max[1]);
    bbx.data.push_back(poly_max[2]);

    //[eval node] Publish the min max bbx
    exploration_bbx_pub_.publish(bbx);
    //[Rviz] Pulish min, max bounding box around actual planning polygon for visualisation
    // nbv_publisher.pubPlanningBBx(poly_min, poly_max);
    //[eval node] Publish actual planning polygon
    planning_poly_pub_.publish(curr_boundary_.polygon);
    // [RViz] Publish actual planning polygon to Rviz for visualisation
    nbv_publisher.pubPlanningPolygon(curr_boundary_);
    // [RViz] Publish ROI for the end-user visualisation 8m or 6m

    // 8m
    double roi_min[3] = {-8.95, -9.05, params->arm_height_min};
    double roi_max[3] = {-0.95, -1.05, params->arm_height_max};

    // 6m
    // double roi_min[3] = {-7.95, -8.05, params->arm_height_min};
    // double roi_max[3] = {-1.95, -2.05, params->arm_height_max};
    nbv_publisher.pubPlanningBBx(roi_min, roi_max);
  }

  void NBVPlanner::cleanCurrentRRT()
  {
    // ROS_INFO_STREAM("Cleaning the RRT");

    // Initialize the best node and best frontier
    best_node_ = nullptr;
    // Reset tree and initialise the current camera position as the root, expand the tree, find the best branch
    resetTree();

    nbv_publisher.sendClearAllMarker(nbv_publisher.rrt_rviz_pub_);
    nbv_publisher.sendClearAllMarker(nbv_publisher.best_branch_rviz_pub_);
    marker_array_rrt_.markers.clear();
  }

  bool NBVPlanner::initializeBoundary(fkie_measurement_msgs::BoundaryPolygon boundary_coords)
  {
    if (!boundary_initialized_)
    {
      // If the boundary is not initalised then use values from the client or the yaml file
      if (boundary_coords.polygon.polygon.points.size() == 0)
      {
        return initializeDefaultBoundary();
      }

      else
      {
        if (boundary_coords.polygon.polygon.points.size() < 3)
        {
          ROS_ERROR_STREAM("A boundary should have 3 or more points, received " << boundary_coords.polygon.polygon.points.size() << "instead");
          return false;
        }
        curr_boundary_ = boundary_coords;
        boundary_initialized_ = true;
      }
    }
    return true;
  }

  bool NBVPlanner::initializeDefaultBoundary()
  {
    ROS_WARN("Boundaries for exploration not known.. Using boundary params from the yaml file");
    if ((params->planner_boundary_x.size() >= 3) && (params->planner_boundary_y.size() >= 3))
    {
      for (auto const &i : boost::combine(params->planner_boundary_x, params->planner_boundary_y))
      {
        double x, y;
        boost::tie(x, y) = i;
        geometry_msgs::Point32 p;
        p.x = x;
        p.y = y;
        p.z = 0;
        curr_boundary_.polygon.polygon.points.push_back(p);
      }
      boundary_initialized_ = true;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("A boundary should have 3 or more points, received " << params->planner_boundary_y.size() << "y points instead");
      return false;
    }
  }

  std::vector<geometry_msgs::PoseStamped> NBVPlanner::getNbvBranch(const std::vector<RRTNode> &curr_frontiers)
  {
    // ROS_INFO_STREAM("=== [2] START: getNbvBranch ===");
    cleanCurrentRRT();

    std::vector<geometry_msgs::PoseStamped> nbv_branch;

    // Calculate the time taken to expand the tree without nodelets and estimation
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    bool is_tree_grown = expandRRT(curr_frontiers);
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("Current RRT size: " << tree_adapter_.kdtree_get_point_count() << " nodes");
    // std::cout << "[Expand rrt] Time taken: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;
    // std::cout << "[Expand rrt] Time taken:" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms], "
    //           << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;

    // If RRT cannot be grown, set best node to Null and return
    if (!is_tree_grown)
    {
      ROS_ERROR("RRT could not be grown.. best_node_ = NULL :(");
      best_node_ = NULL;
      // ROS_DEBUG("=== [2] END: getNbvBranch ===");
      return std::vector<geometry_msgs::PoseStamped>();
    }

    // If there was a best node found... then return the best branch as the next set of goals
    if (best_node_)
    {
      ROS_INFO_STREAM("Found best node: " << best_node_->toString());
      if (best_node_->getGain() > min_gain_)
      {
        // Extract the branch, filter nodes with low gain and that are reachable
        nbv_branch = extractNBVPoses();
      }

      else if (best_node_->getGain() < min_gain_)
      {
        ROS_WARN("best_node_->getGain() < min_gain_, no best branch, no best node");
        best_node_ = NULL;
      }
    }
    // ROS_DEBUG("=== [2] END: getNbvBranch ===");
    return nbv_branch;
  }

  bool NBVPlanner::octomapReceived()
  {
    // Check if octomap was received
    if (!ot_)
    {
      ROS_WARN("No octomap received. Cannot proceed :(");
      ROS_WARN("Please check if you need  to remap to subscribe to octomap_full topic.. ");
      ROS_WARN("If simulating, check if gazebo camera data is published. Maybe you are missing the gazebo_realsense_plugin?");
      return false;
    }
    else
    {
      // ROS_INFO_STREAM("Octomap received");
      return true;
    }
  }

  bool NBVPlanner::expandRRT(const std::vector<RRTNode> &curr_frontiers)
  {
    // Initialise the current camera position as the root
    std::shared_ptr<RRTNode> root = initializeRoot();

    if (!root_initialized_)
    {
      ROS_ERROR("The arm either got stuck outside of the polygon trying to reach an unreachable goal or the robot got confused and drove outside.. Drive the robot inside the polygon and try again");
      return false;
    }
    // ROS_INFO_STREAM("=== [3] START:[Expanding RRT] ====");
    // ROS_INFO_STREAM("=== [Expanding RRT] ====");
    visualization_msgs::MarkerArray marker_array;

    // If RRT is close to atleast one frontier, then the value is set to true
    std::vector<bool> f_vec(curr_frontiers.size(), false);
    int frontier_try_count = 0;

    // Obtain the robot center for sampling in world coordinates
    std::string from_frame = params->robot_sampling_frame;
    geometry_msgs::PoseStamped center;
    center.header.frame_id = from_frame;
    geometry_msgs::PoseStamped sampling_center = transformPose(center, params->world_frame, from_frame);
    // ROS_INFO_STREAM("Robot center in world:(" << sampling_center.pose.position.x << "," << sampling_center.pose.position.y << "," << sampling_center.pose.position.z << ")");

    for (int i = 0; i < params->rrt_tree_length; i++)
    {
      // ROS_INFO_STREAM("Current RRT iteration: " << i);

      octomap::OcTreeNode *ot_result = NULL;
      std::shared_ptr<RRTNode> new_node(new RRTNode);
      std::shared_ptr<RRTNode> nearest_node;
      Eigen::Vector3d q_new;
      std::vector<std::shared_ptr<RRTNode>> near_nodes;
      bool invalid_q_new = false;
      bool is_collision = true;
      int do_while_count = 0;
      bool known = true;

      // Loop if the sample is not known, colliding and not within polygon and exit timeout has not yet been called
      do
      {
        // ROS_INFO_STREAM("do-while count: " << do_while_count);
        if (do_while_count == params->rrt_N_max_tries)
        {
          // ROS_INFO_STREAM("Max count reached:" << do_while_count << ", breaking the loop..");
          invalid_q_new = true;
          break;
        }

        do_while_count += 1;

        // Generate a random sample
        Eigen::Vector3d q_rand;
        // q_rand << sampleNewPoint(params->rrt_sampling_radius);
        q_rand << generateRandomSample(params->rrt_sampling_radius);
        q_rand[0] += sampling_center.pose.position.x;
        q_rand[1] += sampling_center.pose.position.y;
        // ROS_INFO_STREAM("Step 1: q_rand: (" << q_rand[0] << "," << q_rand[1] << "," << q_rand[2] << ")");

        // Find the closest neighbor
        Eigen::Vector3d q_near;
        if (!findClosestNeighbor(q_rand, nearest_node))
        {
          // ROS_INFO_STREAM("findClosestNeighbor could not find closest neighbor");
          continue;
        }

        q_near = poseToEigenVector3d(nearest_node->getPose());
        // ROS_INFO_STREAM("Step 2: q_near (distance): (" << q_near[0] << "," << q_near[1] << "," << q_near[2] << ")");

        // Progress q_near by params->rrt_step_size along the straight line in Q between q_near and q_rand
        findNewRRTNode(q_near, q_rand, q_new);
        // ROS_INFO_STREAM("Step 3: q_new: (" << q_new[0] << "," << q_new[1] << "," << q_new[2] << ")");

        // Search the sample in octomap to check if it known
        if (!std::isfinite(q_new[0]) || !std::isfinite(q_new[1]) || !std::isfinite(q_new[2]))
        {
          ROS_WARN_STREAM("expandRRT: q_new is NULL");
          continue;
        }

        ot_result = ot_->search(octomap::point3d(q_new[0], q_new[1], q_new[2]));

        // Grow RRT either unknown space or in known space
        if (params->sample_in_unknown)
        {
          // Set to true to avoid the do while loop from repeating when the sample is unknown
          known = true;
          // If sample is known and obstacle, repeat do loop from the top
          if (ot_result != NULL && ot_result->getLogOdds() > 0)
          {
            continue;
          }
        }
        else
        {
          // If sample is known, repeat the loop
          if (ot_result == NULL)
          {
            // ROS_INFO_STREAM("q_new: (" << q_new[0] << "," << q_new[1] << "," << q_new[2] << ") has [ot_result = NULL] Continuing the loop..");
            known = false;
            continue;
          }
          // else
          // {
          //   ROS_DEBUG_STREAM("Octree sample search result found:" << ot_result);
          // }
        }

        // From here, things change for RRT star
        if (params->do_rrt_star)
        {
          ROS_INFO("Doing RRT star...");
          // Find all node within neighborhood radius
          near_nodes = findSamplesWithinRadius(params->rrt_step_size, q_new);
          if (near_nodes.size() > 0)
          {
            ROS_INFO_STREAM("Found " << near_nodes.size() << " near_nodes around new_node..");
            auto n = std::min_element(near_nodes.begin(), near_nodes.end(),
                                      [](const std::shared_ptr<RRTNode> &a, const std::shared_ptr<RRTNode> &b)
                                      {
                                        return a->getCostTillRoot() < b->getCostTillRoot();
                                      });
            nearest_node = *n;
            ROS_INFO_STREAM("Step 4.1: Nearest node based on cost: (" << nearest_node->getPose().position.x << "," << nearest_node->getPose().position.y << "," << nearest_node->getPose().position.z << "), cost:" << nearest_node->getCostTillRoot());
          }
        }

        // Check if q_new collides with q_near
        is_collision = collisionLine(q_near, q_new, params->rrt_collision_cyl_radius);

        if (is_collision)
        {
          // ROS_WARN_STREAM("Parent node collides with the child node, try changing the cylinder radius...");
          ROS_DEBUG_STREAM("Step 4.2: q_near->q_new is NOT collision free");
        }
        else
        {
          ROS_DEBUG_STREAM("Step 4.2: q_near->q_new is collision free");
        }

      } while (!known or is_collision or !isSampleInPolygon(q_new, curr_boundary_.polygon.polygon, params->arm_height_max) or (isSampleInPolygon(q_new, curr_robot_footprint, params->self_collision_height)) or (isSampleCloseToRRTNode(q_new, params->rrt_sample_threshold_distance)));

      // ROS_INFO("Out of do while");
      if (invalid_q_new)
      {
        ROS_DEBUG_STREAM("Current RRT iteration: " << i << ": Failed to get a collision free sample q_new in " << do_while_count << " for current iteration: " << i << ". ");
      }
      else
      {

        ROS_DEBUG_STREAM("Step 4.3: Found q_new");
        // Add q_new as the new node since it is collision free and initialise yaw
        new_node->setPose(q_new);

        // Add the node id
        new_node->node_id_ = node_count;
        // ROS_INFO_STREAM("Node id:" << node_count);

        // Compute the gain and yaw for the new node using the position of the node
        updateGain(new_node);

        // compute the yaw based on gain cubature or measurement estimations
        computeYaw(new_node);

        // Add parent info to the node
        new_node->parent_Wp_ = nearest_node;
        // ROS_INFO_STREAM("New node parent: " << new_node->parent_Wp_.lock()->toString());

        // Add node as the nearest node's child
        nearest_node->children_Sp_.push_back(new_node);

        // If RRT star, re-wire the tree before computing the score
        if (params->do_rrt_star)
        {
          ROS_INFO("Doing RRT star rewiring...");
          if (near_nodes.size() > 0)
          {
            ROS_INFO_STREAM("Found near nodes for re-wiring:" << near_nodes.size());
            rewire(near_nodes, new_node);
          }
        }

        // Compute score for the node based on the gain
        new_node->computeScore();

        if (new_node->getScore() > max_node_score)
          max_node_score = new_node->getScore();

        ROS_DEBUG_STREAM("Current iteration:" << i << ", New node: " << new_node->toString());

        if (!best_node_ or new_node->getScore() > best_node_->getScore())
        {
          best_node_ = new_node;
          ROS_DEBUG_STREAM("Current Best node: " << best_node_->toString());
        }

        // Add the node to the kd tree
        tree_adapter_.addNode(new_node);
        kdtree_->addPoints(0, tree_adapter_.kdtree_get_point_count() - 1);

        // Cache the node
        cached_nodes_.push_back(new_node->copyToRRTNode());

        // Show the new node in the tree
        nbv_publisher.addNodeMarkers(new_node, new_node->node_id_, false, false, false, marker_array_rrt_, true);
        nbv_publisher.addEdgeMarkers(new_node, new_node->node_id_, false, marker_array_rrt_);

        // Give the node a uid so that it is visible
        node_count = node_count + 1;

        if (frontier_try_count > params->frontiers_N_max_tries)
        {
          allow_decrementing_i = false;
          ROS_DEBUG("i shall not be decremented any more");
        }

        /** Extend the RRT until it touches the frontier, if there are frontiers and within the frontier count,
         * and if best node is null or has gain lower than frontier or min gain **/
        if (curr_frontiers.size() > 0)
        {
          if (best_node_ == NULL)
          {
            frontier_try_count += 1;
            ROS_DEBUG_STREAM("Frontier try count: " << frontier_try_count << ", [Expanding RRT] best_node_ == NULL");
            std::pair<std::vector<bool>, int> res = expandTreeTowardsFrontiers(f_vec, i, curr_frontiers);
            f_vec = res.first;
            i = res.second;
            ROS_DEBUG_STREAM(" i:" << i);
          }
          else
          {
            // Caution: Assumes frontiers are sorted in descending order of gains
            if (curr_frontiers[0].getGain() > best_node_->getGain() or best_node_->getGain() < min_gain_)
            {
              frontier_try_count += 1;
              if (curr_frontiers[0].getGain() > best_node_->getGain())
              {
                ROS_DEBUG_STREAM("Frontier try count: " << frontier_try_count << ", [Expanding RRT] curr_frontiers[0].getGain() > best_node_->getGain(), curr_frontiers[0].getGain() :" << curr_frontiers[0].getGain() << ", best_node_->getGain()" << best_node_->getGain());
              }
              else
                ROS_DEBUG_STREAM("Frontier try count: " << frontier_try_count << ", [Expanding RRT] best_node_->getGain() < min_gain, best_node_->getGain():" << best_node_->getGain() << ", min_gain" << min_gain_);
              std::pair<std::vector<bool>, int> res = expandTreeTowardsFrontiers(f_vec, i, curr_frontiers);
              f_vec = res.first;
              i = res.second;
            }
          }
        }
      }
    }

    nbv_publisher.sendClearAllMarker(nbv_publisher.rrt_rviz_pub_);
    nbv_publisher.rrt_rviz_pub_.publish(marker_array_rrt_);
    ROS_DEBUG("=== [3] END:[Expanding RRT] ====");

    if (node_count == 1)
    {
      ROS_WARN_STREAM("Tree could not be expanded further. The robot base needs to be moved.");
      return false;
    }
    return true;
  }

  void NBVPlanner::rewire(std::vector<std::shared_ptr<RRTNode>> near_nodes, std::shared_ptr<RRTNode> new_node)
  {
    ROS_INFO_STREAM("---Step 5.0: Rewiring the RRT*---");
    Eigen::Vector3d q_new = poseToEigenVector3d(new_node->getPose());

    // Rewire if no collision and low cost
    for (size_t i = 0; i < near_nodes.size(); i++)
    {
      Eigen::Vector3d q_near = poseToEigenVector3d(near_nodes[i]->getPose());

      ROS_INFO("-----");
      ROS_INFO_STREAM("Current near node: " << i << near_nodes[i]->toString());
      ROS_INFO_STREAM("Current new node:" << i << new_node->toString());

      // Test for collision between new_node and near_nodes[i]
      if (collisionLine(q_near, q_new, params->rrt_collision_cyl_radius))
      {
        ROS_ERROR("Q new collides with Near node, skipping...");
        continue;
      }

      // if near_nodes[i]->cost_till_root > new_node->cost_till_root_ + dist(new_node, near_nodes[i]), then re-wire
      if (!near_nodes[i]->getCostTillRoot())
      {

        ROS_ERROR_STREAM("Found node without cost_till_root_. Check if all new nodes are given their costs.." << near_nodes[i]->toString());
      }
      else
      {
        Eigen::Vector3d eu_dist(q_new - q_near);
        ROS_INFO_STREAM("Distance from q new to q_near: " << eu_dist.norm());
        ROS_INFO_STREAM("near_nodes[i]->getCostTillRoot(): " << near_nodes[i]->getCostTillRoot());
        ROS_INFO_STREAM("new_node->getCostTillRoot() + eu_dist.norm(): " << new_node->getCostTillRoot() + eu_dist.norm());
        if (near_nodes[i]->getCostTillRoot() > new_node->getCostTillRoot() + eu_dist.norm())
        {
          ROS_INFO_STREAM("Step 5.1: near_nodes[i]->cost_till_root_ > new_node->cost_till_root_ + eu_dist.norm(), rewiring parent and child for node:" << i);

          // Re-assign the parent of near_nodes[i] as new_node
          near_nodes[i]->parent_Wp_ = new_node;
          ROS_INFO("near_nodes[i]->parent_Wp_ = new_node");

          // Re-assign the child of new_node as near_nodes[i]
          new_node->children_Sp_.push_back(near_nodes[i]);
          ROS_INFO("new_node->children_Sp_.push_back(near_nodes[i])");
        }
      }
    }
  }

  std::pair<std::vector<bool>, int> NBVPlanner::expandTreeTowardsFrontiers(std::vector<bool> f_vec, int i, const std::vector<RRTNode> &curr_frontiers)
  {
    if (!std::any_of(f_vec.begin(), f_vec.end(), [](bool f_vec)
                     { return f_vec; }))
    {
      // ROS_INFO_STREAM("[Expanding RRT] Current Tree not close to any frontier yet...");
      // Give the algorithm a chance to touch a frontier

      if (allow_decrementing_i)
      {
        i--;
        ROS_DEBUG_STREAM("[Expanding RRT] Decremented i:" << i);
      }

      for (size_t j = 0; j < curr_frontiers.size(); j++)
      {
        Eigen::Vector3d frontier = poseToEigenVector3d(curr_frontiers[j].getPose());
        double frontier_distance = 0.5;
        // If the frontier is < 0.5, then set to true
        if (isSampleCloseToRRTNode(frontier, frontier_distance))
        {
          f_vec[j] = true;
          ROS_DEBUG_STREAM("[Expanding RRT] Found at least ONE frontier close to RRT Node. f_vec[j] = true");
          break;
        }
      }
    }

    else
    {
      ROS_DEBUG_STREAM("RRT already close to atleast one frontier. :)");
    }
    return std::make_pair(f_vec, i);
  }

  bool NBVPlanner::findClosestNeighbor(const Eigen::Vector3d &sample, std::shared_ptr<RRTNode> &closest_neighbor)
  {
    // Turn the sample into a point to query
    double query_pt[3] = {sample[0], sample[1], sample[2]};

    // Do nanoflann magic
    std::size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    if (kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10)))
    {
      // std::cout << "Nearest neighbour found!" << std::endl;
      // std::cout << "Nearest neighbor index =" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
      closest_neighbor = tree_adapter_.nodes[ret_index];
      ROS_DEBUG_STREAM("Nearest neighbor: " << tree_adapter_.nodes[ret_index]->toString());
      return true;
    }
    else
    {
      // ROS_ERROR("No neighbors found! :(");
      return false;
    }
  }

  void NBVPlanner::findNewRRTNode(const Eigen::Vector3d &origin, const Eigen::Vector3d &point, Eigen::Vector3d &qnew)
  {
    Eigen::Vector3d direction(point - origin);
    if (direction.norm() > params->rrt_step_size)
      direction = params->rrt_step_size * direction.normalized();
    qnew[0] = origin[0] + direction[0];
    qnew[1] = origin[1] + direction[1];
    qnew[2] = origin[2] + direction[2];
  }

  std::shared_ptr<RRTNode> NBVPlanner::initializeRoot()
  {
    // Check if current camera pose is inside the polygon
    Eigen::Vector3d v = poseToEigenVector3d(curr_camera_pose_);

    // If the root is not inside the polygon do not start
    if (!isSampleInPolygon(v, curr_boundary_.polygon.polygon, params->arm_height_max))
    {
      ROS_ERROR("Current camera pose is not inside the polygon for exploration. ");
      root_initialized_ = false;
    }
    ROS_DEBUG("Current camera pose is inside the polygon");

    // Create the root node from current camera pose
    std::shared_ptr<RRTNode> root(new RRTNode);
    root->setPose(curr_camera_pose_);
    root->node_id_ = node_count;
    ROS_DEBUG_STREAM("Root node: " << root->toString());

    // Add root node to kd-tree
    tree_adapter_.addNode(root);
    kdtree_->addPoints(0, tree_adapter_.kdtree_get_point_count() - 1);
    nbv_publisher.addNodeMarkers(root, root->node_id_, true, false, false, marker_array_rrt_, true);
    node_count += 1;
    root_initialized_ = true;
    ROS_DEBUG_STREAM("Initialised Root node for RRT");
    return root;
  }

  bool NBVPlanner::isSampleCloseToRRTNode(const Eigen::Vector3d &sample, const double &distance) const
  {
    // Turn the sample into a point to query
    double query_pt[3] = {sample[0], sample[1], sample[2]};

    // Do nanoflann magic
    std::size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);

    // Find the nearest neighbor from the tree
    if (kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10)))
    {
      // std::cout << "Nearest neighbor index =" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
      // Check the distance to nearest neighbour, if sq distance to nearest neighbour < sq params->rrt_sample_threshold_distance, then the samples are too close
      if (out_dist_sqr < std::pow(distance, 2))
      {
        ROS_DEBUG_STREAM("[Sample Too close] Sample:(" << sample[0] << "," << sample[1] << "," << sample[2] << ") is at a distance of " << std::sqrt(out_dist_sqr) << ". But the threshold is:" << params->rrt_sample_threshold_distance);
        return true;
      }
    }

    return false;
  }

  geometry_msgs::PoseStamped NBVPlanner::transformPose(geometry_msgs::PoseStamped input_pose, std::string to_frame, std::string from_frame)
  {
    // std::cout << "transforming the pose from shoulder to world frame .." << std::endl;
    geometry_msgs::PoseStamped output_pose;
    geometry_msgs::TransformStamped shoulder_to_world;
    try
    {
      shoulder_to_world = tfBuffer.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));

      // shoulder_to_world = tfBuffer.lookupTransform(to_frame, ros::Time(), input_pose.header.frame_id, input_pose.header.stamp, input_pose.header.frame_id, ros::Duration(0.5));
      tf2::doTransform(input_pose, output_pose, shoulder_to_world);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    return output_pose;
  }

  bool NBVPlanner::collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r)
  {
    ROS_DEBUG_STREAM("Entered collision line");

    octomap::point3d start(p1[0], p1[1], p1[2]);
    octomap::point3d end(p2[0], p2[1], p2[2]);
    octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                         std::min(p1[2], p2[2]) - r);
    octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                         std::max(p1[2], p2[2]) + r);
    // Squared distance between the start and end along the axis
    double lsq = (end - start).norm_sq();

    // Squared radius
    double rsq = r * r;

    for (octomap::OcTree::leaf_bbx_iterator it = ot_->begin_leafs_bbx(min, max),
                                            it_end = ot_->end_leafs_bbx();
         it != it_end; ++it)
    {
      // ROS_INFO("Inside for loop");
      octomap::point3d pt(it.getX(), it.getY(), it.getZ());

      if (it->getLogOdds() > 0) // Node is occupied
      {
        // ROS_WARN("Node occupied");

        // Check if the point is inside the cylinder
        if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
        {
          ROS_DEBUG_STREAM("Sample collides");
          return true;
        }
      }
    }
    ROS_DEBUG_STREAM("In collision (exiting)");

    return false;
  }

  Eigen::Vector3d NBVPlanner::sampleNewPoint(double radius) const
  {
    // Samples one point uniformly over a sphere with a radius
    Eigen::Vector3d point;
    ROS_WARN_STREAM("Sampling using AEPS");
    do
    {
      for (int i = 0; i < 3; i++)
        point[i] = radius * 2.0 *
                   (((double)rand()) / ((double)RAND_MAX) - 0.5);
    } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) >
                 pow(radius, 2.0) or
             point[2] < 0);
    return point;
  }

} // namespace fkie_nbv_planner
