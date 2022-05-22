#ifndef SPARSE_GRID_H
#define SPARSE_GRID_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <fkie_measurement_msgs/MeasurementEstimation.h>
#include "fkie_nbv_planner/structs.hpp"
#include "fkie_nbv_planner/NBVParameters.hpp"

namespace fkie_nbv_planner
{
  template <typename Container, typename Index = IndexGrid, typename IndexHasher = IndexGridHasher>
  class SparseGrid
  {
  public:
    std::string name;
    Container max_value;

  protected:
    std::unordered_map<Index, Container, IndexHasher> data; // grid-based index map
    ros::Publisher pub_marker_grid;
    double grid_size = 1.0;

    NBVParameters *params = NULL;

  public:
    SparseGrid(std::string name = "grid")
    {
      // get params singletone
      params = &NBVParameters::getInstance();

      ros::NodeHandle public_node;
      pub_marker_grid = public_node.advertise<visualization_msgs::MarkerArray>(name, 10, false);
    }

    ~SparseGrid()
    {
      pub_marker_grid.shutdown();
      clear();
    }

    /**
     * @brief Return the total number of grid voxels
     */
    size_t size() const
    {
      return data.size();
    }

    /**
     * @brief Set a new grid_size and clear the data container
     */
    void setGridSize(const double grid_size)
    {
      this->grid_size = grid_size;
      clear();
    }

    /**
     * @brief Check if Index [i] already exists in grid [data]
     */
    [[nodiscard]] bool exists(const Index &i) const
    {
      return data.count(i) > 0;
    }

    /**
     * @brief Check if the Index from PositionGrid [p] already exists in grid [data]
     */
    [[nodiscard]] bool exists(const PositionGrid &p) const
    {
      return data.count(positionToIndex(p)) > 0;
    }

    /**
     * @brief Get the Index from PositionGrid
     */
    Index getIndex(const PositionGrid &p) const
    {
      return positionToIndex(p);
    }

    /**
     * @brief Get the PositionGrid from Index
     */
    PositionGrid getPosition(const Index &i) const
    {
      return indexToPosition(i);
    }

    /**
     * @brief Check if Index [i] has a value registered
     */
    [[nodiscard]] bool hasValue(const Index &i) const
    {
      return exists(i) && data.count(i) > 0;
    }

    /**
     * @brief Get the value [Container] associated to PositionGrid [p]
     */
    Container getValue(const PositionGrid &p) const
    {
      Index i = positionToIndex(p);

      if (exists(i))
        return data.at(i);
      else
        return Container();
    }

    /**
     * @brief Get the value [Container] associated to Index [i]
     */
    Container getValue(const Index &i) const
    {
      if (exists(i))
        return data.at(i);
      else
        return Container();
    }

    /**
     * @brief Try to add [vs] to PositionGrid [p] in [data]
     */
    bool addValue(const PositionGrid &p, const Container &vs)
    {
      return addIndex(positionToIndex(p), vs);
    }

    /**
     * @brief Clear [data]
     */
    void clear()
    {
      data.clear();
    }

    /**
     * @brief Return in [indices] the neighbors of [p] within a discrete [index_radius]
     */
    void getIndexNeighbors(const PositionGrid p, const int index_radius, std::vector<Index> &indices) const
    {
      indices.clear();
      Index c = positionToIndex(p);

      for (int x = c.x - index_radius; x <= c.x + index_radius; x++)
      {
        for (int y = c.y - index_radius; y <= c.y + index_radius; y++)
        {
          for (int z = c.z - index_radius; z <= c.z + index_radius; z++)
          {
            indices.push_back(Index(x, y, z));
          }
        }
      }
    }

    /**
     * @brief Publish grid data using RVIZ markers
     */
    void publishToRVIZ()
    {
      if (pub_marker_grid.getNumSubscribers() == 0)
        return;

      // delete all previous markers
      visualization_msgs::MarkerArray ma_clear;
      visualization_msgs::Marker m_clear;
      m_clear.action = visualization_msgs::Marker::DELETEALL;
      m_clear.header.stamp = ros::Time::now();
      ma_clear.markers.push_back(m_clear);
      pub_marker_grid.publish(ma_clear);

      if (data.size() == 0)
        return;

      visualization_msgs::MarkerArray ma;

      visualization_msgs::Marker m;
      m.header.frame_id = params->world_frame;
      m.header.stamp = ros::Time();
      m.ns = name; // TODO: add description to the grid
      m.id = 0;
      m.type = visualization_msgs::Marker::CUBE_LIST;
      m.action = visualization_msgs::Marker::ADD;

      m.scale.x = grid_size;
      m.scale.y = grid_size;
      m.scale.z = grid_size;
      m.pose.orientation.w = 1.0;

      std_msgs::ColorRGBA color;

      for (auto d : data)
      {
        PositionGrid pg = indexToPosition(d.first);
        Container gv = d.second;

        geometry_msgs::Point p = pg.toPoint();

        if (std::abs(max_value.getValue()) > 0)
          color.r = gv.getValue() / max_value.getValue();
        else
          color.r = 0.0;

        color.g = 0.0;
        color.b = 0.0;
        color.a = 0.8;

        m.points.push_back(p);
        m.colors.push_back(color);
      }
      ma.markers.push_back(m);

      pub_marker_grid.publish(ma);
    }

  protected:
    /**
     * @brief Try to add [vs] to Index [i] in [data]
     */
    bool addIndex(const Index &i, const Container &vs)
    {
      if (exists(i) || !i.isValid())
      {
        return false;
      }
      else
      {
        data.insert(std::make_pair(i, vs));

        if (vs.getValue() > max_value.getValue())
          max_value = vs;

        return true;
      }
    }

  private:
    Index positionToIndex(const PositionGrid &p) const
    {
      return Index::positionToIndex(p, grid_size);
    }

    PositionGrid indexToPosition(const Index &i) const
    {
      return Index::indexToPosition(i, grid_size);
    }
  };
}
#endif // SPARSE_GRID_H