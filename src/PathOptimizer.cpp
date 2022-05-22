#include "fkie_nbv_planner/PathOptimizer.h"

PathOptimizer::PathOptimizer()
{
  // get params singletone
  params = &NBVParameters::getInstance();
};

std::vector<geometry_msgs::PoseStamped> PathOptimizer::optimizePath(std::vector<geometry_msgs::PoseStamped> points,
                                                                    const std::shared_ptr<octomap::OcTree> ot_,
                                                                    const geometry_msgs::Polygon &curr_robot_bbx_) const
{
  ROS_INFO_STREAM("optimizePath: Original path length: " << points.size());
  // TODO: keep at least first  and last points

  if (params->optimize_path_remove_points_inside_bbx)
  {
    points = removePointsInsideRobotBBX(points, curr_robot_bbx_);
    ROS_INFO_STREAM("optimizePath: Path length after removing points inside BBX: " << points.size());
  }
  if (params->optimize_path_shift_points_towards_obstacles)
  {
    points = applyShiftPointsTowardsObstacles(points, ot_);
    ROS_INFO_STREAM("optimizePath: Path length after shifting points towards obstacles: " << points.size());
  }

  if (params->optimize_path_using_convex_hull)
  {
    points = applyConvexHull(points);
    ROS_INFO_STREAM("optimizePath: Path length after ConvexHull: " << points.size());
  }

  if (params->optimize_path_using_sampling)
  {
    points = applySampling(points);
    ROS_INFO_STREAM("optimizePath: Path length after Sampling: " << points.size());
  }

  return points;
}

std::vector<geometry_msgs::PoseStamped> PathOptimizer::applyConvexHull(const std::vector<geometry_msgs::PoseStamped> &input_points) const
{
  if (input_points.size() <= 3)
    return input_points;

  std::vector<geometry_msgs::PoseStamped> p_out;

  // save the points to keep
  std::unordered_set<PositionGrid2D, PositionGrid2DHasher> p_keep;

  std::vector<Point_3> points;

  // fill vector of [Point_3]
  for (const geometry_msgs::PoseStamped &ps : input_points)
  {
    // Point_3 p(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    Point_3 p(ps.pose.position.x, ps.pose.position.y, 0.0); // use the same hiehgt for all points
    points.push_back(p);
  }

  // define polyhedron to hold convex hull
  Polyhedron_3 poly;

  // compute convex hull of non-collinear points
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  // get items to keep
  for (PolyhedronPointIterator ei = poly.points_begin(); ei != poly.points_end(); ++ei)
  {
    Point_3 p = *ei;

    PositionGrid2D p_g(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    if (p_keep.count(p_g) == 0)
      p_keep.insert(p_g);
  }

  // fill output vector
  for (const geometry_msgs::PoseStamped &ps : input_points)
  {
    // get original pose stamped
    PositionGrid2D p_g(ps.pose.position.x, ps.pose.position.y);
    if (p_keep.count(p_g) > 0)
      p_out.push_back(ps);
  }

  // if output is empty, add final point
  if (p_out.size() == 0 && input_points.size() > 0)
  {
    p_out.push_back(input_points.at(input_points.size() - 1));
  }

  return p_out;
};

std::vector<geometry_msgs::PoseStamped> PathOptimizer::applyOutlierRemoval(const std::vector<geometry_msgs::PoseStamped> &input_points) const
{
  if (input_points.size() <= 3)
    return input_points;

  std::vector<geometry_msgs::PoseStamped> p_out;

  // save the points to keep
  std::unordered_set<PositionGrid, PositionGridHasher> p_keep;

  std::vector<Point_3> points;

  // fill vector of [Point_3]
  for (const geometry_msgs::PoseStamped &ps : input_points)
    points.push_back(Point_3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));

  const int nb_neighbors = std::min(2, (int)points.size());

  // Estimate scale of the point set with average spacing
  const double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, nb_neighbors);

  points.erase(CGAL::remove_outliers(points,
                                     nb_neighbors,
                                     CGAL::parameters::threshold_percent(100.).   // No limit on the number of outliers to remove
                                     threshold_distance(1.1 * average_spacing))); // Point with distance above 1.1*average_spacing are considered outliers

  ROS_ERROR_STREAM("average_spacing: " << average_spacing);

  // after erase(), use Scott Meyer's "swap trick" to trim excess capacity
  std::vector<Point_3>(points).swap(points);

  // get items to keep
  for (const Point_3 &p : points)
  {
    PositionGrid p_g(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
    if (p_keep.count(p_g) == 0)
      p_keep.insert(p_g);
  }

  // fill output vector
  for (const geometry_msgs::PoseStamped &ps : input_points)
  {
    // get original pose stamped
    PositionGrid p_g(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    if (p_keep.count(p_g) > 0)
      p_out.push_back(ps);
  }

  // if output is empty, add final point
  if (p_out.size() == 0 && input_points.size() > 0)
  {
    p_out.push_back(input_points.at(input_points.size() - 1));
  }

  return p_out;
};

std::vector<geometry_msgs::PoseStamped> PathOptimizer::applySampling(const std::vector<geometry_msgs::PoseStamped> &input_points) const
{
  if (input_points.size() <= 2)
    return input_points;

  std::vector<geometry_msgs::PoseStamped> p_out;

  if (params->path_optimizer_sampling_target_count == 1)
  {
    // return final point
    p_out.push_back(input_points.at(input_points.size() - 1));
  }
  else if (params->path_optimizer_sampling_target_count == 2)
  {
    // return start and final points
    p_out.push_back(input_points.at(0));
    p_out.push_back(input_points.at(input_points.size() - 1));
  }
  else
  {
    // compute linear interpolation between points
    p_out = uniformLinearInterpolation(input_points, params->path_optimizer_sampling_target_count);
  }

  // if output is empty, add final point
  if (p_out.size() == 0 && input_points.size() > 0)
  {
    p_out.push_back(input_points.at(input_points.size() - 1));
  }

  return p_out;
};

std::vector<geometry_msgs::PoseStamped> PathOptimizer::applyShiftPointsTowardsObstacles(const std::vector<geometry_msgs::PoseStamped> &input_points,
                                                                                        const std::shared_ptr<octomap::OcTree> ot_) const
{
  if (input_points.size() == 0 || !ot_)
    return input_points;

  std::vector<geometry_msgs::PoseStamped> p_out;

  for (const geometry_msgs::PoseStamped &p_in : input_points)
  {
    // get projected line
    double p_yaw = tf2::getYaw(p_in.pose.orientation);

    // project line in XY plane: TODO: Extend to XYZ plane using roll/pitch/yaw angles
    geometry_msgs::PoseStamped p_end = p_in;
    p_end.pose.position.x += params->shift_points_max_length * std::cos(p_yaw);
    p_end.pose.position.y += params->shift_points_max_length * std::sin(p_yaw);

    // ray-trace line to check for obstacles
    octomap::point3d p0 = octomap::point3d(p_in.pose.position.x, p_in.pose.position.y, p_in.pose.position.z);
    octomap::point3d p1 = octomap::point3d(p_end.pose.position.x, p_end.pose.position.y, p_end.pose.position.z);

    octomap::point3d p_collision;
    // if (checkCollisionBoundingBox(p0, p1, params->shift_points_size_bbx_search, ot_, p_collision))
    // if (checkCollisionCylinder(p0, p1, params->shift_points_size_bbx_search, ot_, p_collision))
    if (checkCollisionRayCasting(p0, p1, ot_, p_collision))
    {
      geometry_msgs::PoseStamped p_final;
      p_final = p_in; // copy header and orientation

      // update position
      p_final.pose.position.x = p_collision.x() - params->shift_points_threshold * std::cos(p_yaw);
      p_final.pose.position.y = p_collision.y() - params->shift_points_threshold * std::sin(p_yaw);

      p_out.push_back(p_final);
    }

    // if no obstacle found after params->shift_points_max_length, do nothing? TODO!
  }

  // if output is empty, add final point
  if (p_out.size() == 0 && input_points.size() > 0)
  {
    p_out.push_back(input_points.at(input_points.size() - 1));
  }

  return p_out;
}

std::vector<geometry_msgs::PoseStamped> PathOptimizer::removePointsInsideRobotBBX(const std::vector<geometry_msgs::PoseStamped> &input_points,
                                                                                  const geometry_msgs::Polygon &bounding_box) const
{
  if (input_points.size() == 0)
    return input_points;

  std::vector<geometry_msgs::PoseStamped> p_out;

  for (const geometry_msgs::PoseStamped &p_in : input_points)
  {
    if (!isSampleInPolygon(p_in.pose, bounding_box, params->self_collision_height))
    {
      p_out.push_back(p_in);
    }
  }

  // if output is empty, add final point
  if (p_out.size() == 0 && input_points.size() > 0)
  {
    p_out.push_back(input_points.at(input_points.size() - 1));
  }

  return p_out;
}

// Gives a vector of Points which are sampled as equally-spaced segments
// taken along the linear interpolation between points in the source.
// In general, consecutive points in the result will not be equidistant,
// because of a corner-cutting effect.
std::vector<geometry_msgs::PoseStamped> PathOptimizer::uniformLinearInterpolation(const std::vector<geometry_msgs::PoseStamped> &source,
                                                                                  const std::size_t target_count) const
{
  std::vector<geometry_msgs::PoseStamped> result;
  if (source.size() < 2 || target_count < 2)
  {
    return source;
  }

  // total_length is the total length along a linear interpolation
  // of the source points.
  const double total_length = linearCurveLength(source);

  // segment_length is the length between result points, taken as
  // distance traveled between these points on a linear interpolation
  // of the source points.  The actual Euclidean distance between
  // points in the result vector can vary, and is always less than
  // or equal to segment_length.
  const double segment_length = total_length / (target_count - 1);

  // start and finish are the current source segment's endpoints
  auto start = source.begin();
  auto finish = start + 1;

  // src_segment_offset is the distance along a linear interpolation
  // of the source curve from its first point to the start of the current
  // source segment.
  double src_segment_offset = 0;

  // src_segment_length is the length of a line connecting the current
  // source segment's start and finish points.
  double src_segment_length = distancePoseStamped(*start, *finish);

  // The first point in the result is the same as the first point
  // in the source.
  result.push_back(*start);

  for (std::size_t i = 1; i < target_count - 1; ++i)
  {
    // next_offset is the distance along a linear interpolation
    // of the source curve from its beginning to the location
    // of the i'th point in the result.
    // segment_length is multiplied by i here because iteratively
    // adding segment_length could accumulate error.
    const double next_offset = segment_length * i;

    // Check if next_offset lies inside the current source segment.
    // If not, move to the next source segment and update the
    // source segment offset and length variables.
    while (src_segment_offset + src_segment_length < next_offset)
    {
      src_segment_offset += src_segment_length;
      start = finish++;
      src_segment_length = distancePoseStamped(*start, *finish);
    }
    // part_offset is the distance into the current source segment
    // associated with the i'th point's offset.
    const double part_offset = next_offset - src_segment_offset;

    // part_ratio is part_offset's normalized distance into the
    // source segment. Its value is between 0 and 1,
    // where 0 locates the next point at "start" and 1
    // locates it at "finish".  In-between values represent a
    // weighted location between these two extremes.
    const double part_ratio = part_offset / src_segment_length;

    // Use part_ratio to calculate the next point's components
    // as weighted averages of components of the current
    // source segment's points.
    geometry_msgs::PoseStamped p_insert = source.at(0);
    // TODO: Fix angle of new point!
    p_insert.pose.position.x = start->pose.position.x + part_ratio * (finish->pose.position.x - start->pose.position.x);
    p_insert.pose.position.y = start->pose.position.y + part_ratio * (finish->pose.position.y - start->pose.position.y);

    result.push_back(p_insert);
  }

  // The first and last points of the result are exactly
  // the same as the first and last points from the input,
  // so the iterated calculation above skips calculating
  // the last point in the result, which is instead copied
  // directly from the source vector here.
  result.push_back(source.back());
  return result;
}

bool PathOptimizer::checkCollisionBoundingBox(const octomap::point3d &start,
                                              const octomap::point3d &end,
                                              const double &r,
                                              const std::shared_ptr<octomap::OcTree> ot_,
                                              octomap::point3d &p_collision) const
{
  octomap::point3d bbx_min(std::min(start.x(), end.x()) - r,
                           std::min(start.y(), end.y()) - r,
                           std::min(start.z(), end.z()) - r);

  octomap::point3d bbx_max(std::max(start.x(), end.x()) + r,
                           std::max(start.y(), end.y()) + r,
                           std::max(start.z(), end.z()) + r);

  for (auto it = ot_->begin_leafs_bbx(bbx_min, bbx_max); it != ot_->end_leafs_bbx(); ++it)
  {
    Eigen::Vector3d cube_center(it.getX(), it.getY(), it.getZ());
    int depth_level = it.getDepth();
    double cube_size = ot_->getNodeSize(depth_level);

    // Check if it is really inside bounding box, since leaf_bbx_iterator begins "too early"
    Eigen::Vector3d cube_lower_bound = cube_center - (cube_size / 2) * Eigen::Vector3d::Ones();
    Eigen::Vector3d cube_upper_bound = cube_center + (cube_size / 2) * Eigen::Vector3d::Ones();

    if (cube_upper_bound.x() < bbx_min.x() ||
        cube_lower_bound.x() > bbx_max.x() ||
        cube_upper_bound.y() < bbx_min.y() ||
        cube_lower_bound.y() > bbx_max.y() ||
        cube_upper_bound.z() < bbx_min.z() ||
        cube_lower_bound.z() > bbx_max.z())
    {
      continue;
    }

    if (ot_->isNodeOccupied(*it)) // Node is occupied
    {
      p_collision = octomap::point3d(it.getX(), it.getY(), it.getZ());
      return true;
    }
  }

  return false;
}

bool PathOptimizer::checkCollisionCylinder(const octomap::point3d &start,
                                           const octomap::point3d &end,
                                           const double &r,
                                           const std::shared_ptr<octomap::OcTree> ot_,
                                           octomap::point3d &p_collision) const
{
  octomap::point3d bbx_min(std::min(start.x(), end.x()) - r,
                           std::min(start.y(), end.y()) - r,
                           std::min(start.z(), end.z()) - r);

  octomap::point3d bbx_max(std::max(start.x(), end.x()) + r,
                           std::max(start.y(), end.y()) + r,
                           std::max(start.z(), end.z()) + r);

  for (auto it = ot_->begin_leafs_bbx(bbx_min, bbx_max); it != ot_->end_leafs_bbx(); ++it)
  {
    Eigen::Vector3d cube_center(it.getX(), it.getY(), it.getZ());
    int depth_level = it.getDepth();
    double cube_size = ot_->getNodeSize(depth_level);

    // Check if it is really inside bounding box, since leaf_bbx_iterator begins "too early"
    Eigen::Vector3d cube_lower_bound = cube_center - (cube_size / 2) * Eigen::Vector3d::Ones();
    Eigen::Vector3d cube_upper_bound = cube_center + (cube_size / 2) * Eigen::Vector3d::Ones();

    if (cube_upper_bound.x() < bbx_min.x() ||
        cube_lower_bound.x() > bbx_max.x() ||
        cube_upper_bound.y() < bbx_min.y() ||
        cube_lower_bound.y() > bbx_max.y() ||
        cube_upper_bound.z() < bbx_min.z() ||
        cube_lower_bound.z() > bbx_max.z())
    {
      continue;
    }

    if (ot_->isNodeOccupied(*it)) // Node is occupied
    {
      // Check if the point is inside the cylinder
      double lsq = (end - start).norm_sq(); // Squared distance between the start and end along the axis
      double rsq = r * r;                   // Squared radius
      p_collision = octomap::point3d(it.getX(), it.getY(), it.getZ());
      if (CylTest_CapsFirst(start, end, lsq, rsq, p_collision) > 0 or (end - p_collision).norm() < r)
      {
        return true;
      }
    }
  }

  return false;
}

bool PathOptimizer::checkCollisionRayCasting(const octomap::point3d &start,
                                             const octomap::point3d &end,
                                             const std::shared_ptr<octomap::OcTree> ot_,
                                             octomap::point3d &p_collision) const
{
  octomap::KeyRay key_ray;
  if (ot_->computeRayKeys(start, end, key_ray))
  {
    for (const octomap::OcTreeKey &key : key_ray)
    {
      octomap::OcTreeNode *node = ot_->search(key);

      // if obstacle found, use params->shift_points_threshold to set position of new point
      if (node != NULL && ot_->isNodeOccupied(node))
      {
        octomath::Vector3 p_node_key = ot_->keyToCoord(key);
        p_collision = octomap::point3d(p_node_key.x(), p_node_key.y(), p_node_key.z());
        return true;
      }
    }
  }

  return false;
}