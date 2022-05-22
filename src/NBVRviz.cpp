#include "fkie_nbv_planner/NBVRviz.h"

namespace fkie_nbv_planner
{
  NBVRviz::NBVRviz()
  {
    // get params singletone
    params = &NBVParameters::getInstance();

    // Publish data for visualization
    best_branch_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_best_branch", 10, true);
    boundary_bbx_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>("viz_planning_bbx", 10, true);
    boundary_poly_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>("viz_planning_polygon", 10, true);
    cached_nodes_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_cached_nodes", 10, true);
    cached_best_branch_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_cached_best_branches", 10, true);
    curr_frontiers_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_curr_frontiers", 10, true);
    rrt_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_rrt", 10, true);
    best_frontier_rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("best_frontier", 10, true);
    path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("planner_paths", 5, true);

    clearAllPublishers();
  }

  NBVRviz::~NBVRviz()
  {
    best_branch_rviz_pub_.shutdown();
    boundary_bbx_rviz_pub_.shutdown();
    boundary_poly_rviz_pub_.shutdown();
    cached_nodes_rviz_pub_.shutdown();
    cached_best_branch_rviz_pub_.shutdown();
    curr_frontiers_rviz_pub_.shutdown();
    rrt_rviz_pub_.shutdown();
    best_frontier_rviz_pub_.shutdown();
    path_pub_.shutdown();
  }

  void NBVRviz::addNodeMarkers(const std::shared_ptr<RRTNode> &node,
                               const int &id,
                               const bool &root,
                               const bool &best_node, const bool &best_branch,
                               visualization_msgs::MarkerArray &marker_array,
                               const bool show_label,
                               const std::string &node_color, const std::string &node_marker_type) const
  {
    visualization_msgs::Marker marker;
    std::string label;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "rrt_nodes";
    marker.id = id;
    marker.lifetime = ros::Duration(0);
    if (root)
    {
      marker.type = marker.CUBE;
    }
    else
    {
      if (node_marker_type.compare("sphere") == 0)
      {
        marker.type = marker.SPHERE;
      }
      else
      {
        marker.type = marker.ARROW;
      }
    }
    marker.action = marker.ADD;
    marker.pose = node->getPose();

    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.color.a = 0.4;

    std::stringstream s_gain;
    std::stringstream s_score;
    std::stringstream s_cost_to_root;
    s_gain << std::fixed << std::setprecision(2) << node->getGain();
    s_score << std::fixed << std::setprecision(2) << node->getScore();
    s_cost_to_root << std::fixed << std::setprecision(2) << node->getCost();

    if (best_node)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.08;
      marker.color.b = 0.57;
      marker.color.a = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      label = "BN: (G:" + s_gain.str() + ", S:" + s_score.str() + ")";
    }

    else if (root)
    {
      marker.color.r = 0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.scale.x = 0.02;
      marker.scale.y = 0.06;
      marker.scale.z = 0.02;
      marker.color.a = 1.0;
      label = "ROOT";
    }

    else if (best_branch)
    {
      // Color best branch nodes green
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
    }

    else
    {
      if (node_color.compare("red") == 0)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }

      else if (node_color.compare("green") == 0)
      {
        marker.color.r = 0.0;
        marker.color.g = 128.0 / 255.0;
        marker.color.b = 0.0;
      }

      else if (node_color.compare("cyan") == 0)
      {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
      }

      else
      {
        marker.color.r = 1.0;
        marker.color.g = 0.6;
        marker.color.b = 0.0;
      }

      if (show_label)
      {
        label = "(G:" + s_gain.str() + ", S:" + s_score.str() + ")";
      }
    }

    marker_array.markers.push_back(marker);

    if (show_label)
    {
      addNodeTextMarkers(node, id, label, marker_array);
    }
  }

  void NBVRviz::addNodeTextMarkers(const std::shared_ptr<RRTNode> &node,
                                   const int &id,
                                   const std::string &label,
                                   visualization_msgs::MarkerArray &marker_array) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "rrt_nodes_label";
    marker.id = id;
    marker.lifetime = ros::Duration(0.0);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = marker.ADD;
    marker.pose = node->getPose();
    marker.pose.position.z += 0.04;

    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text = label;
    marker_array.markers.push_back(marker);
  }

  void NBVRviz::addEdgeMarkers(const std::shared_ptr<RRTNode> &node,
                               const int &id,
                               const bool &best_branch,
                               visualization_msgs::MarkerArray &marker_array,
                               const std::string &edge_color) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "rrt_edges";
    marker.id = id;
    marker.lifetime = ros::Duration(0.0);
    marker.type = marker.ARROW;
    marker.action = marker.ADD;
    marker.pose = node->parent_Wp_.lock()->getPose();

    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);

    Eigen::Vector3d dir(poseToEigenVector3d(node->getPose()) - poseToEigenVector3d(node->parent_Wp_.lock()->getPose()));
    q.setFromTwoVectors(init, dir);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = dir.norm();
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.4;

    if (best_branch)
    {
      // Green
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;
    }
    else
    {
      if (edge_color.compare("red") == 0)
      {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }

      else if (edge_color.compare("green") == 0)
      {
        marker.color.r = 0.0;
        marker.color.g = 128.0 / 255.0;
        marker.color.b = 0.0;
      }

      else
      {
        // Orange
        marker.color.r = 1.0;
        marker.color.g = 0.6;
        marker.color.b = 0.0;
      }
    }

    marker.frame_locked = false;
    marker_array.markers.push_back(marker);
  }

  void NBVRviz::clearAllPublishers()
  {
    sendClearAllMarker(best_branch_rviz_pub_);
    sendClearAllMarker(cached_best_branch_rviz_pub_);
    sendClearAllMarker(curr_frontiers_rviz_pub_);
    sendClearAllMarker(rrt_rviz_pub_);
    sendClearAllMarker(best_frontier_rviz_pub_);
    sendClearAllMarker(path_pub_);
  }

  visualization_msgs::Marker NBVRviz::createBoundingBoxMarker(std::string ns, int id, double bbx_min[3], double bbx_max[3], std::string color)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.ns = ns;
    marker.id = id;
    marker.frame_locked = true;
    marker.scale.x = 0.1;

    // marker color
    marker.color.a = 1.0;

    if (color.compare("red") == 0)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }

    if (color.compare("cyan") == 0)
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    if (color.compare("pink") == 0)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.08;
      marker.color.b = 0.57;
    }
    if (color.compare("blue") == 0)
    {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    if (color.compare("orange") == 0)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.6;
      marker.color.b = 0.0;
    }

    // marker orientaiton
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // marker position
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    // First position
    geometry_msgs::Point first_line_point;
    first_line_point.x = bbx_min[0];
    first_line_point.y = bbx_min[1];
    first_line_point.z = bbx_min[2];
    marker.points.push_back(first_line_point);

    // Second position
    geometry_msgs::Point second_line_point;
    second_line_point.x = bbx_max[0];
    second_line_point.y = bbx_min[1];
    second_line_point.z = bbx_min[2];
    marker.points.push_back(second_line_point);

    // Third position
    geometry_msgs::Point third_line_point;
    third_line_point.x = bbx_max[0];
    third_line_point.y = bbx_max[1];
    third_line_point.z = bbx_min[2];
    marker.points.push_back(third_line_point);

    // Fourth position
    geometry_msgs::Point forth_line_point;
    forth_line_point.x = bbx_min[0];
    forth_line_point.y = bbx_max[1];
    forth_line_point.z = bbx_min[2];
    marker.points.push_back(forth_line_point);

    // Dummy first position
    geometry_msgs::Point dummy_first_line_point;
    dummy_first_line_point.x = bbx_min[0];
    dummy_first_line_point.y = bbx_min[1];
    dummy_first_line_point.z = bbx_min[2];
    marker.points.push_back(dummy_first_line_point);

    // Fifth point
    geometry_msgs::Point fifth_line_point;
    fifth_line_point.x = bbx_min[0];
    fifth_line_point.y = bbx_min[1];
    fifth_line_point.z = bbx_max[2];
    marker.points.push_back(fifth_line_point);

    // Sixth point
    geometry_msgs::Point sixth_line_point;
    sixth_line_point.x = bbx_max[0];
    sixth_line_point.y = bbx_min[1];
    sixth_line_point.z = bbx_max[2];
    marker.points.push_back(sixth_line_point);

    // Seventh point
    geometry_msgs::Point seventh_line_point;
    seventh_line_point.x = bbx_max[0];
    seventh_line_point.y = bbx_max[1];
    seventh_line_point.z = bbx_max[2];
    marker.points.push_back(seventh_line_point);

    // Eighth point
    geometry_msgs::Point eighth_line_point;
    eighth_line_point.x = bbx_min[0];
    eighth_line_point.y = bbx_max[1];
    eighth_line_point.z = bbx_max[2];
    marker.points.push_back(eighth_line_point);

    // Dummy first position
    geometry_msgs::Point dummy_fifth_line_point;
    dummy_fifth_line_point.x = bbx_min[0];
    dummy_fifth_line_point.y = bbx_min[1];
    dummy_fifth_line_point.z = bbx_max[2];
    marker.points.push_back(dummy_fifth_line_point);

    return marker;
  }

  visualization_msgs::Marker NBVRviz::createBoundaryPolygon(std::string ns, int id, fkie_measurement_msgs::BoundaryPolygon curr_boundary, std::string color)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.ns = ns;
    marker.id = id;
    marker.frame_locked = true;
    marker.scale.x = 0.08;

    // marker color
    marker.color.a = 1.0;

    if (color.compare("red") == 0)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }

    if (color.compare("cyan") == 0)
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }

    // marker orientaiton
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // marker position
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    // First position
    geometry_msgs::Point first_line_point;
    first_line_point.x = curr_boundary.polygon.polygon.points[0].x;
    first_line_point.y = curr_boundary.polygon.polygon.points[0].y;
    first_line_point.z = params->arm_height_min;
    marker.points.push_back(first_line_point);

    // Second position
    geometry_msgs::Point second_line_point;
    second_line_point.x = curr_boundary.polygon.polygon.points[1].x;
    second_line_point.y = curr_boundary.polygon.polygon.points[1].y;
    second_line_point.z = params->arm_height_min;
    marker.points.push_back(second_line_point);

    // Third position
    geometry_msgs::Point third_line_point;
    third_line_point.x = curr_boundary.polygon.polygon.points[2].x;
    third_line_point.y = curr_boundary.polygon.polygon.points[2].y;
    third_line_point.z = params->arm_height_min;
    marker.points.push_back(third_line_point);

    // Fourth position
    geometry_msgs::Point forth_line_point;
    forth_line_point.x = curr_boundary.polygon.polygon.points[3].x;
    forth_line_point.y = curr_boundary.polygon.polygon.points[3].y;
    forth_line_point.z = params->arm_height_min;
    marker.points.push_back(forth_line_point);

    // Dummy first position
    geometry_msgs::Point dummy_first_line_point;
    dummy_first_line_point.x = curr_boundary.polygon.polygon.points[0].x;
    dummy_first_line_point.y = curr_boundary.polygon.polygon.points[0].y;
    dummy_first_line_point.z = params->arm_height_min;
    marker.points.push_back(dummy_first_line_point);

    // Fifth point
    geometry_msgs::Point fifth_line_point;
    fifth_line_point.x = curr_boundary.polygon.polygon.points[0].x;
    fifth_line_point.y = curr_boundary.polygon.polygon.points[0].y;
    fifth_line_point.z = params->arm_height_max;
    marker.points.push_back(fifth_line_point);

    // Sixth point
    geometry_msgs::Point sixth_line_point;
    sixth_line_point.x = curr_boundary.polygon.polygon.points[1].x;
    sixth_line_point.y = curr_boundary.polygon.polygon.points[1].y;
    sixth_line_point.z = params->arm_height_max;
    marker.points.push_back(sixth_line_point);

    // Seventh point
    geometry_msgs::Point seventh_line_point;
    seventh_line_point.x = curr_boundary.polygon.polygon.points[2].x;
    seventh_line_point.y = curr_boundary.polygon.polygon.points[2].y;
    seventh_line_point.z = params->arm_height_max;
    marker.points.push_back(seventh_line_point);

    // Eighth point
    geometry_msgs::Point eighth_line_point;
    eighth_line_point.x = curr_boundary.polygon.polygon.points[3].x;
    eighth_line_point.y = curr_boundary.polygon.polygon.points[3].y;
    eighth_line_point.z = params->arm_height_max;
    marker.points.push_back(eighth_line_point);

    // Dummy first position
    geometry_msgs::Point dummy_fifth_line_point;
    dummy_fifth_line_point.x = curr_boundary.polygon.polygon.points[0].x;
    dummy_fifth_line_point.y = curr_boundary.polygon.polygon.points[0].y;
    dummy_fifth_line_point.z = params->arm_height_max;
    marker.points.push_back(dummy_fifth_line_point);

    return marker;
  }

  void NBVRviz::pubCachedNodes(const std::vector<RRTNode> &cached_nodes, double max_node_score)
  {
    if (cached_nodes_rviz_pub_.getNumSubscribers() == 0)
      return;

    if (cached_nodes.size() == 0)
    {
      sendClearAllMarker(cached_nodes_rviz_pub_);
      return;
    }

    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < cached_nodes.size(); i++)
    {
      // Visualize the frontier pose
      visualization_msgs::Marker marker;
      marker.header.frame_id = params->world_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "cached_nodes";
      marker.id = i;
      marker.type = marker.ARROW;
      marker.action = marker.ADD;
      marker.pose = cached_nodes[i].toPose();

      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.04;
      marker.color.a = 0.8;

      // Check if max_node_score is a finite value and not NAN or infinite
      if (std::isfinite(max_node_score))
      {
        // Check if +ve value of max_node_score > 0
        if (std::abs(max_node_score) > 0)
        {
          // Color gradient of green
          marker.color.r = 0.0;
          marker.color.g = (cached_nodes[i].getScore() / max_node_score); // normalize score as green channel
          marker.color.b = 0.0;
          marker.color.a = (cached_nodes[i].getScore() / max_node_score); // normalize score as alpha
        }
        else
        {
          ROS_WARN_STREAM("pubCachedNodes: std::abs(max_node_score) less than zero");
        }
      }
      else
      {
        ROS_WARN_STREAM("pubCachedNodes: max_node_score is infinite/NAN");
        ROS_WARN_STREAM("pubCachedNodes: Ignoring invalid node: " << cached_nodes[i].toString() << "- max_node_score: " << max_node_score);
      }

      marker_array.markers.push_back(marker);

      // Visualize the gain value
      visualization_msgs::Marker label;
      label.header.frame_id = params->world_frame;
      label.header.stamp = ros::Time::now();
      label.ns = "cached_nodes_label";
      label.id = i;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = label.ADD;
      label.pose = marker.pose;
      label.pose.position.z = marker.pose.position.z + 0.04;
      label.scale.z = 0.05;
      label.color.a = (cached_nodes[i].getScore() / max_node_score); // normalize score as alpha
      label.color.r = 0.0;
      label.color.g = 0.0;
      label.color.b = 0.0;

      std::stringstream s_gain;
      std::stringstream s_score;
      s_gain << std::fixed << std::setprecision(2) << cached_nodes[i].getGain();
      s_score << std::fixed << std::setprecision(2) << cached_nodes[i].getScore();
      label.text = "(" + s_gain.str() + ", " + s_score.str() + ")";
      marker_array.markers.push_back(label);
    }

    cached_nodes_rviz_pub_.publish(marker_array);
  }

  void NBVRviz::addFrontierMarkers(const RRTNode &curr_frontier, const int &id, const double &utility, visualization_msgs::MarkerArray &marker_array)
  {
    // Visualize the frontier pose
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "frontiers";
    marker.id = id;
    marker.type = marker.ARROW;
    marker.action = marker.ADD;
    marker.pose = curr_frontier.toPose();

    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 0.6;

    // Color Light blue
    marker.color.r = 0.0;
    marker.color.g = 191.0 / 255.0;
    marker.color.b = 1.0;

    marker_array.markers.push_back(marker);

    // Visualize the gain value
    visualization_msgs::Marker label;
    label.header.frame_id = params->world_frame;
    label.header.stamp = ros::Time::now();
    label.ns = "frontier_gain_label";
    label.id = id;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = label.ADD;
    label.pose = marker.pose;
    label.pose.position.z = marker.pose.position.z + 0.06;
    label.scale.z = 0.05;
    label.color.a = 1.0;
    label.color.r = 0.0;
    label.color.g = 0.0;
    label.color.b = 0.0;

    std::stringstream s_gain;
    std::stringstream s_utility;
    s_gain << std::fixed << std::setprecision(2) << curr_frontier.getGain();
    s_utility << std::fixed << std::setprecision(2) << utility;
    label.text = "(C: " + s_utility.str() + ")";
    marker_array.markers.push_back(label);
  }

  void NBVRviz::pubPath(const std::vector<geometry_msgs::PoseStamped> &path,
                        const std::string ns,
                        const double color_r,
                        const double color_g,
                        const double color_b,
                        const double arrow_length)
  {
    if (path_pub_.getNumSubscribers() == 0)
      return;

    if (path.size() < 2)
    {
      sendClearAllMarker(path_pub_);
      return;
    }

    int marker_id = 0;

    visualization_msgs::MarkerArray marker_array;

    // Add lines
    visualization_msgs::Marker marker_line;
    marker_line.header.frame_id = params->world_frame;
    marker_line.header.stamp = ros::Time::now();
    marker_line.ns = ns;
    marker_line.id = marker_id;
    marker_line.lifetime = ros::Duration(0);
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.scale.x = arrow_length / 8.0;
    marker_line.color.a = 0.8;

    marker_line.color.r = color_r;
    marker_line.color.g = color_g;
    marker_line.color.b = color_b;

    for (size_t i = 1; i < path.size(); i++)
    {
      marker_line.points.push_back(path[i - 1].pose.position);
      marker_line.points.push_back(path[i].pose.position);
    }

    marker_array.markers.push_back(marker_line);
    marker_id++;

    // Add arrows
    for (size_t i = 0; i < path.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = params->world_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = ns + "_arrows";
      marker.id = marker_id;
      marker_line.lifetime = ros::Duration(0);
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = path[i].pose;

      marker.scale.x = arrow_length;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;

      marker.color.r = color_r;
      marker.color.g = color_g;
      marker.color.b = 0.0; // color_b;
      marker.color.a = 0.8;

      marker_array.markers.push_back(marker);
      marker_id++;
    }

    path_pub_.publish(marker_array);
  }

  void NBVRviz::pubPlanningBBx(double bbx_min[3], double bbx_max[3])
  {

    boundary_bbx_rviz_pub_.publish(createBoundingBoxMarker("planning_bbx", 0, bbx_min, bbx_max, "red"));
  }

  void NBVRviz::pubPlanningPolygon(fkie_measurement_msgs::BoundaryPolygon curr_boundary)
  {
    boundary_poly_rviz_pub_.publish(createBoundaryPolygon("planning_poly", 0, curr_boundary, "cyan"));
  }

  void NBVRviz::pubBestFrontier(const geometry_msgs::Pose &p)
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = params->world_frame;
    marker.header.stamp = ros::Time::now();
    marker.pose = p;
    marker.ns = "best_frontier";
    marker.id = 0;
    marker.type = marker.ARROW;
    marker.scale.x = 0.1;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 1.0;
    marker.color.r = 138.0 / 255.0;
    marker.color.g = 43.0 / 255.0;
    marker.color.b = 226.0 / 255.0;
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker label;
    label.header.frame_id = params->world_frame;
    label.header.stamp = ros::Time::now();
    label.ns = "best_frontier_label";
    label.id = 0;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = marker.ADD;
    label.pose = p;
    label.pose.position.z = p.position.z - 0.04;
    label.scale.z = 0.05;
    label.color.a = 1.0;
    label.color.r = 0.0;
    label.color.g = 0.0;
    label.color.b = 0.0;
    label.text = "BEST FRONTIER";
    marker_array.markers.push_back(label);

    best_frontier_rviz_pub_.publish(marker_array);
  }

  void NBVRviz::sendClearAllMarker(const ros::Publisher &pub) const
  {
    if (pub.getNumSubscribers() == 0)
      return;

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker &m = ma.markers.back();
    m.action = visualization_msgs::Marker::DELETEALL;
    m.header.stamp = ros::Time::now();
    pub.publish(ma);
  }
}
