#ifndef NBV_PUBLISHER_H
#define NBV_PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <fkie_measurement_msgs/BoundaryPolygon.h>

#include "fkie_nbv_planner/RRTNode.h"
#include "fkie_nbv_planner/NBVParameters.hpp"
#include "fkie_nbv_planner/utils.hpp"

namespace fkie_nbv_planner
{
  class NBVRviz
  {
  private:
    ros::NodeHandle nh_;
    NBVParameters *params = NULL;

    ros::Publisher boundary_bbx_rviz_pub_;
    ros::Publisher boundary_poly_rviz_pub_;
    ros::Publisher cached_best_branch_rviz_pub_;
    ros::Publisher cached_nodes_rviz_pub_;

    ros::Publisher random_samples_rviz_pub_;

    ros::Publisher best_frontier_rviz_pub_;
    ros::Publisher path_pub_;

  public:
    // TODO: Move publishers to private and encapsulate
    ros::Publisher best_branch_rviz_pub_;
    ros::Publisher rrt_rviz_pub_;
    ros::Publisher curr_frontiers_rviz_pub_;

  public:
    NBVRviz();

    ~NBVRviz();

    void addNodeMarkers(const std::shared_ptr<RRTNode> &node,
                        const int &id,
                        const bool &root,
                        const bool &best_node, const bool &best_branch,
                        visualization_msgs::MarkerArray &marker_array,
                        const bool show_label = false,
                        const std::string &node_color = std::string(), const std::string &node_marker_type = std::string()) const;

    void addNodeTextMarkers(const std::shared_ptr<RRTNode> &node,
                            const int &id,
                            const std::string &label,
                            visualization_msgs::MarkerArray &marker_array) const;

    void addEdgeMarkers(const std::shared_ptr<RRTNode> &node,
                        const int &id,
                        const bool &best_branch,
                        visualization_msgs::MarkerArray &marker_array,
                        const std::string &edge_color = std::string()) const;

    void clearAllPublishers();

    visualization_msgs::Marker createBoundingBoxMarker(std::string ns, int id, double bbx_min[3], double bbx_max[3], std::string color);
    visualization_msgs::Marker createBoundaryPolygon(std::string ns, int id, fkie_measurement_msgs::BoundaryPolygon curr_boundary, std::string color);

    void pubCachedNodes(const std::vector<RRTNode> &cached_nodes, double max_node_score);

    void addFrontierMarkers(const RRTNode &curr_frontier, const int &id, const double &utility, visualization_msgs::MarkerArray &marker_array);

    void pubPlanningBBx(double bbx_min[3], double bbx_max[3]);
    void pubPlanningPolygon(fkie_measurement_msgs::BoundaryPolygon curr_boundary);

    void pubBestFrontier(const geometry_msgs::Pose &p);

    void pubPath(const std::vector<geometry_msgs::PoseStamped> &path,
                 const std::string ns = "path",
                 const double color_r = 1.0,
                 const double color_g = 1.0,
                 const double color_b = 1.0,
                 const double arrow_length = 0.06);

    void sendClearAllMarker(const ros::Publisher &pub) const;
  };
}
#endif // NBV_PUBLISHER_H