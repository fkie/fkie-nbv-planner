#ifndef TREE_2_NANO_FLANN_ADAPTER_H
#define TREE_2_NANO_FLANN_ADAPTER_H

#include <ros/ros.h>

#include "fkie_nbv_planner/nanoflann.hpp"
#include "fkie_nbv_planner/RRTNode.h"

namespace fkie_nbv_planner
{
  /**
   * @brief Adapter between tree and the nanoflann kdtree
   */
  struct Tree2NanoflannAdapter
  {
    std::vector<std::shared_ptr<RRTNode>> nodes;

    /**
     * @brief Add node to the KD tree
     */
    void addNode(std::shared_ptr<RRTNode> node)
    {
      nodes.push_back(node);
    }

    /**
     * @brief Clear the KD tree
     */
    void clear()
    {
      nodes.clear();
    }

    // Return the number of nodes
    inline std::size_t kdtree_get_point_count() const { return nodes.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      if (dim == 0)
        return nodes[idx]->getPose().position.x;
      else if (dim == 1)
        return nodes[idx]->getPose().position.y;
      else
        return nodes[idx]->getPose().position.z;
    }

    template <class BBOX>
    [[nodiscard]] bool kdtree_get_bbox(BBOX & /*bb*/) const
    {
      return false;
    }
  };
} // namespace fkie_nbv_planner
#endif // TREE_2_NANO_FLANN_ADAPTER_H