#ifndef NBV_UTILS_H
#define NBV_UTILS_H

#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <octomap/octomap.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @brief Samples one point uniformly over a sphere with a [radius] and within [arm_height_min] and [arm_height_max]
 */
inline Eigen::Vector3d sampleNewPoint(const double &radius)
{
  NBVParameters *params = &NBVParameters::getInstance();
  Eigen::Vector3d point;
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = radius * 2.0 *
                 (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) >
               pow(radius, 2.0) or
           point[2] < params->arm_height_min or point[2] > params->arm_height_max);

  return point;
}

/**
 * @brief Check if [sample] is located inside of [polygon] and within [max_height]
 */
inline bool isSampleInPolygon(const Eigen::Vector3d &sample, const geometry_msgs::Polygon &polygon, const double &max_height)
{
  NBVParameters *params = &NBVParameters::getInstance();
  bool inside = false;
  // Ensure the point is inside the z bounds
  if (sample[2] >= params->arm_height_min and sample[2] <= max_height)
  {
    // ROS_DEBUG_STREAM("Current z:" << sample[2] << " is within z bounds, checking for xy");
    for (size_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
    {
      if ((polygon.points[i].y > sample[1]) != (polygon.points[j].y > sample[1]) && (sample[0] < (polygon.points[j].x - polygon.points[i].x) * (sample[1] - polygon.points[i].y) / (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x))
      {
        inside = !inside;
      }
    }
  }

  return inside;
}

/**
 * @brief Check if [sample] is located inside of [polygon] and within [max_height]
 */
inline bool isSampleInPolygon(const geometry_msgs::Pose &pose, const geometry_msgs::Polygon &polygon, const double &max_height)
{

  Eigen::Vector3d sample(pose.position.x, pose.position.y, pose.position.z);
  return isSampleInPolygon(sample, polygon, max_height);
}

/**
 * @brief Generate a random sample that lies in the upper half of the hemisphere
 */
inline Eigen::Vector3d generateRandomSample(const double &radius)
{
  NBVParameters *params = &NBVParameters::getInstance();
  Eigen::Vector3d q_rand;

  do
  {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    // Pick a float within the respective ranges
    std::uniform_real_distribution<> dis_theta(0.0, 2 * M_PI);
    std::uniform_real_distribution<> dis_noise(0.0, 1);
    std::uniform_real_distribution<> dis_r(0.0, radius);
    // std::cout
    //     << "Current random number is: " << dis_r(gen);

    double theta = dis_theta(gen);
    double noise = dis_noise(gen);
    double phi = acos((2 * noise) - 1);
    double radius = pow(dis_r(gen), 1.0 / 3.0);
    q_rand[0] = radius * sin(phi) * cos(theta);
    q_rand[1] = radius * sin(phi) * sin(theta);
    q_rand[2] = radius * cos(phi);
  } while (q_rand[2] < params->arm_height_min or q_rand[2] > params->arm_height_max); // Or if the sample is within robot collision bbx

  // ROS_INFO_STREAM("q_rand in robot frame: (" << q_rand[0] << "," << q_rand[1] << "," << q_rand[2] << ")");

  return q_rand;
}
//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
inline float CylTest_CapsFirst(const octomap::point3d &pt1, const octomap::point3d &pt2, float lsq, float rsq, const octomap::point3d &pt)
{
  float dx, dy, dz;    // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz; // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x(); // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y(); // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x(); // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq); // return distance squared to axis
  }
}

/**
 * @brief Computes the Euclidean distance between two [geometry_msgs::PoseStamped] points
 */
inline double distancePoseStamped(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
{
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;
  const double lsq = dx * dx + dy * dy;
  return std::sqrt(lsq);
}

/**
 * @brief LinearCurveLength calculates the total length of the linear
 *        interpolation through a vector of Points.  It is the sum of
 *        the Euclidean distances between all consecutive points in the vector.
 */
inline double linearCurveLength(std::vector<geometry_msgs::PoseStamped> const &points)
{
  auto start = points.begin();
  if (start == points.end())
    return 0;
  auto finish = start + 1;
  double sum = 0;
  while (finish != points.end())
  {
    sum += distancePoseStamped(*start, *finish);
    start = finish++;
  }
  return sum;
}

/**
 * @brief Computes the Euclidean distance between two [geometry_msgs::PoseStamped] points
 */
inline Eigen::Vector3d poseToEigenVector3d(const geometry_msgs::Pose &p)
{
  return Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
}

#endif /* NBV_UTILS_H */
