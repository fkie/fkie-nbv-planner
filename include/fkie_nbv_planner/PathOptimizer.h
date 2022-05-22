#ifndef PATH_OPTIMIZER_H_
#define PATH_OPTIMIZER_H_

#include <geometry_msgs/PoseStamped.h>
#include "fkie_nbv_planner/NBVParameters.hpp"
#include "fkie_nbv_planner/structs.hpp"
#include "fkie_nbv_planner/utils.hpp"
#include <octomap/octomap.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>
#include <boost/iterator/counting_iterator.hpp>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <vector>
#include <fstream>
#include <unordered_set>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef K::Point_3 Point_3;
typedef CGAL::Sequential_tag Concurrency_tag;

typedef Polyhedron_3::Vertex_iterator PolyhedronVertexIterator;
typedef Polyhedron_3::Edge_const_iterator PolyhedronEdgeIterator;
typedef Polyhedron_3::Point_const_iterator PolyhedronPointIterator;
typedef Polyhedron_3::Halfedge_const_iterator PolyhedronHalfedgeIterator;

class PathOptimizer
{
private:
  NBVParameters *params = NULL;

public:
  PathOptimizer();
  ~PathOptimizer() = default;

  /**
   * @brief Compute and return an optimized path based on input [points]
   * TODO: Currently, only convex_hull optimization is available
   */
  std::vector<geometry_msgs::PoseStamped> optimizePath(std::vector<geometry_msgs::PoseStamped> points,
                                                       const std::shared_ptr<octomap::OcTree> ot_,
                                                       const geometry_msgs::Polygon &curr_robot_bbx_ = geometry_msgs::Polygon()) const;

protected:
  /**
   * @brief Optimize a path based on Convex Hull algorithm
   * Ref: https://doc.cgal.org/4.11.3/Convex_hull_3/index.html
   */
  std::vector<geometry_msgs::PoseStamped> applyConvexHull(const std::vector<geometry_msgs::PoseStamped> &input_points) const;

  /**
   * @brief Optimize a path based on point cloud outlier removal
   * Ref: https://doc.cgal.org/4.11.3/Point_set_processing_3/index.html#title14
   */
  std::vector<geometry_msgs::PoseStamped> applyOutlierRemoval(const std::vector<geometry_msgs::PoseStamped> &input_points) const;

  /**
   * @brief Optimize a path using up/down sampling
   */
  std::vector<geometry_msgs::PoseStamped> applySampling(const std::vector<geometry_msgs::PoseStamped> &input_points) const;

  /**
   * @brief Move the points closer to existing obstacles using ray tracing
   */
  std::vector<geometry_msgs::PoseStamped> applyShiftPointsTowardsObstacles(const std::vector<geometry_msgs::PoseStamped> &input_points,
                                                                           const std::shared_ptr<octomap::OcTree> ot_) const;
  /**
   * @brief Remove all points inside a given bounding box
   */
  std::vector<geometry_msgs::PoseStamped> removePointsInsideRobotBBX(const std::vector<geometry_msgs::PoseStamped> &input_points,
                                                                     const geometry_msgs::Polygon &bounding_box) const;

private:
  std::vector<geometry_msgs::PoseStamped> uniformLinearInterpolation(const std::vector<geometry_msgs::PoseStamped> &source, const std::size_t target_count) const;

  bool checkCollisionBoundingBox(const octomap::point3d &start,
                                 const octomap::point3d &end,
                                 const double &r,
                                 const std::shared_ptr<octomap::OcTree> ot_,
                                 octomap::point3d &p_collision) const;

  bool checkCollisionCylinder(const octomap::point3d &start,
                              const octomap::point3d &end,
                              const double &r,
                              const std::shared_ptr<octomap::OcTree> ot_,
                              octomap::point3d &p_collision) const;

  bool checkCollisionRayCasting(const octomap::point3d &start,
                                const octomap::point3d &end,
                                const std::shared_ptr<octomap::OcTree> ot_,
                                octomap::point3d &p_collision) const;

  // TODO: optimize paths using also spline:
  //    https://github.com/chen0040/cpp-spline
  //    https://ecode.dev/smoothing-a-3d-path-using-convolution/
};

#endif /* PATH_OPTIMIZER_H_ */
