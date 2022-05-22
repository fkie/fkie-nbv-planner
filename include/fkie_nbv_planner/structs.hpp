#ifndef NBV_STRUCTS_H_
#define NBV_STRUCTS_H_

#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <gps_common/conversions.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

const double inf_double = std::numeric_limits<double>::max();
const float inf_float = std::numeric_limits<float>::max();
const int inf_int = std::numeric_limits<int>::max();

/**
 * @brief Struct that represent estimation values coming from the measurement server
 */
struct MeasurementValue
{
  double mean = 0.0;
  double variance = 0.0;
  MeasurementValue(){};
  MeasurementValue(double mean, double variance) : mean(mean), variance(variance){};

  double getValue() const
  {
    return mean;
  }

  std::string toString() const
  {
    return "mean: " + std::to_string(mean) + " variance: " + std::to_string(variance);
  }

  MeasurementValue operator-(const MeasurementValue &other) const
  {
    MeasurementValue diff;
    diff.mean = this->mean - other.mean;
    diff.variance = this->variance - other.variance;
    return diff;
  }

  bool operator<(const MeasurementValue &other) const
  {
    return mean < other.mean;
  }

  bool operator>(const MeasurementValue &other) const
  {
    return mean > other.mean;
  }
};

/**
 * @brief Struct that represent visited cells
 */
struct VisitedValue
{
  bool visited = false;
  VisitedValue(){};
  VisitedValue(bool visited) : visited(visited){};

  double getValue() const
  {
    return visited ? 1.0 : 0.0;
  }

  std::string toString() const
  {
    return "visited: " + std::to_string(visited);
  }
};

/**
 * @brief Storage 3D points (x, y, z)
 */
struct PositionGrid
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  PositionGrid()
      : x(inf_double), y(inf_double), z(0.0){};
  PositionGrid(const double x, const double y)
      : x(x), y(y), z(0.0){};
  PositionGrid(const double x, const double y, const double z)
      : x(x), y(y), z(z){};
  PositionGrid(const Eigen::Vector4d &v)
  {
    x = v[0];
    y = v[1];
    z = v[2];
  };
  PositionGrid(PositionGrid p, tf::StampedTransform st)
  {
    // convert point to world frame
    tf::Stamped<tf::Point> pout_test, pin_test;
    pin_test.setX(p.x);
    pin_test.setY(p.y);
    pin_test.setZ(p.z);
    pout_test.setData(st * pin_test);

    x = pout_test.getX();
    y = pout_test.getY();
    z = pout_test.getZ();
  };

  PositionGrid(geometry_msgs::Pose ps)
  {
    x = ps.position.x;
    y = ps.position.y;
    z = ps.position.z;
  };

  PositionGrid(geometry_msgs::Point ps)
  {
    x = ps.x;
    y = ps.y;
    z = ps.z;
  };

  bool operator==(const PositionGrid &other) const
  {
    // return (x == other.x && y == other.y && z == other.z);
    return (std::abs(x - other.x) < 0.001 && std::abs(y - other.y) < 0.001 && std::abs(z - other.z) < 0.001);
    // return (std::abs(x - other.x) < 0.001 && std::abs(y - other.y) < 0.001);
  }

  bool operator!=(const PositionGrid &other) const
  {
    return !(*this == other);
  }

  bool operator<(const PositionGrid &other) const
  {
    return (x < other.x) && (y < other.y) && (z < other.z);
  }

  PositionGrid operator+(const PositionGrid &other) const
  {
    PositionGrid sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.z = z + other.z;
    return sum;
  }

  void operator+=(const PositionGrid &other)
  {
    x = x + other.x;
    y = y + other.y;
    z = z + other.z;
  }

  PositionGrid operator-(const PositionGrid &other) const
  {
    PositionGrid sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    sum.z = z - other.z;
    return sum;
  }

  void operator-=(const PositionGrid &other)
  {
    x = x - other.x;
    y = y - other.y;
    z = z - other.z;
  }

  double distanceTo(const PositionGrid &other) const
  {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) + std::pow(z - other.z, 2));
  }

  double distanceTo(const geometry_msgs::Point &other) const
  {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) + std::pow(z - other.z, 2));
  }

  bool isValid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000;
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y) + " z: " + std::to_string(z));
  }

  geometry_msgs::Point toPoint() const
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  geometry_msgs::Pose toPose() const
  {
    geometry_msgs::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    return p;
  }

  geometry_msgs::PoseStamped toPoseStamped(const std::string frame_id) const
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    p.pose.orientation.w = 1.0;
    return p;
  }

  geometry_msgs::PointStamped toPointStamped(const std::string frame_id) const
  {
    geometry_msgs::PointStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id;
    p.point.x = x;
    p.point.y = y;
    p.point.z = z;
    return p;
  }
};

/**
 * @brief Hasher for [PositionGrid] in 2D
 */
struct PositionGridHasher
{
  std::size_t operator()(const PositionGrid &k) const
  {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1) ^ (std::hash<double>()(k.z) << 1);
    // return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1);
  }
};

/**
 * @brief Storage 3D indexes (x, y, z)
 */
struct IndexGrid
{
  int x = 0;
  int y = 0;
  int z = 0;

  IndexGrid()
      : x(inf_int), y(inf_int), z(inf_int){};
  IndexGrid(int x, int y, int z)
      : x(x), y(y), z(z){};

  bool operator==(const IndexGrid &other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator!=(const IndexGrid &other) const
  {
    return !(x == other.x && y == other.y && z == other.z);
  }

  IndexGrid operator+(const IndexGrid &other) const
  {
    IndexGrid sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.z = z + other.z;
    return sum;
  }

  IndexGrid operator-(const IndexGrid &other) const
  {
    IndexGrid sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    sum.z = z - other.z;
    return sum;
  }

  bool isValid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000 && std::abs(z) < 2147483000;
  }

  static IndexGrid positionToIndex(const PositionGrid &p, const double grid_size)
  {
    int x = (int)(std::round(p.x / grid_size));
    int y = (int)(std::round(p.y / grid_size));
    int z = (int)(std::round(p.z / grid_size));
    return IndexGrid(x, y, z);
  }

  static PositionGrid indexToPosition(const IndexGrid &i, const double grid_size)
  {
    double x = (i.x * grid_size);
    double y = (i.y * grid_size);
    double z = (i.z * grid_size);

    return PositionGrid(x, y, z);
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y) + " z: " + std::to_string(z));
  }
};

/**
 * @brief Hasher for [IndexGrid] in 3D
 */
struct IndexGridHasher
{
  std::size_t operator()(const IndexGrid &k) const
  {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1) ^ (std::hash<double>()(k.z) << 1);
    //return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1);
  }
};

/**
 * @brief Storage 2D indexes (x, y)
 */
struct IndexGrid2D
{
  int x = 0;
  int y = 0;

  IndexGrid2D()
      : x(inf_int), y(inf_int){};
  IndexGrid2D(int x, int y)
      : x(x), y(y){};

  bool operator==(const IndexGrid2D &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const IndexGrid2D &other) const
  {
    return !(x == other.x && y == other.y);
  }

  IndexGrid2D operator+(const IndexGrid2D &other) const
  {
    IndexGrid2D sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    return sum;
  }

  IndexGrid2D operator-(const IndexGrid2D &other) const
  {
    IndexGrid2D sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    return sum;
  }

  bool isValid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000;
  }

  static IndexGrid2D positionToIndex(const PositionGrid &p, const double grid_size)
  {
    int x = (int)(std::round(p.x / grid_size));
    int y = (int)(std::round(p.y / grid_size));
    return IndexGrid2D(x, y);
  }

  static PositionGrid indexToPosition(const IndexGrid2D &i, const double grid_size)
  {
    double x = (i.x * grid_size);
    double y = (i.y * grid_size);

    return PositionGrid(x, y, 0.0);
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y));
  }
};

/**
 * @brief Hasher for [IndexGrid2D] in 2D
 */
struct IndexGrid2DHasher
{
  std::size_t operator()(const IndexGrid2D &k) const
  {
    return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1);
  }
};

typedef std::vector<PositionGrid> PathGrid;

/**
 * @brief Storage 3D points (x, y, z)
 */
struct PositionGrid2D
{
  double x = 0.0;
  double y = 0.0;

  PositionGrid2D()
      : x(inf_double), y(inf_double){};
  PositionGrid2D(double x, double y)
      : x(x), y(y){};

  bool operator==(const PositionGrid2D &other) const
  {
    // return (x == other.x && y == other.y && z == other.z);
    // return ( std::abs( x - other.x ) < 0.001 && std::abs( y - other.y ) < 0.001 && std::abs( z - other.z ) < 0.001 );
    return (std::abs(x - other.x) < 0.001 && std::abs(y - other.y) < 0.001);
  }

  bool operator!=(const PositionGrid2D &other) const
  {
    return !(*this == other);
  }

  bool operator<(const PositionGrid2D &other) const
  {
    return (x < other.x) && (y < other.y);
  }

  PositionGrid2D operator+(const PositionGrid2D &other) const
  {
    PositionGrid2D sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    return sum;
  }

  void operator+=(const PositionGrid2D &other)
  {
    x = x + other.x;
    y = y + other.y;
  }

  PositionGrid2D operator-(const PositionGrid2D &other) const
  {
    PositionGrid2D sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    return sum;
  }

  void operator-=(const PositionGrid2D &other)
  {
    x = x - other.x;
    y = y - other.y;
  }

  bool isValid() const
  {
    return std::abs(x) < 2147483000 && std::abs(y) < 2147483000;
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y));
  }
};

/**
 * @brief Hasher for [PositionGrid] in 2D
 */
struct PositionGrid2DHasher
{
  std::size_t operator()(const PositionGrid2D &k) const
  {
    //return ( ( std::hash< double >()( k.x ) ^ ( std::hash< double >()( k.y ) << 1 ) ) >> 1 ) ^ ( std::hash< double >()( k.z ) << 1 );
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1);
  }
};

#endif /* NBV_STRUCTS_H_ */
