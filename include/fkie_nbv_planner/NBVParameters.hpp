#ifndef NBV_PARAMETERS_H_
#define NBV_PARAMETERS_H_

#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

class NBVParameters : public fkie_ddynamic_reconfigure::GlobalParameters<NBVParameters>
{
protected:
  static NBVParameters *globalPtr;

public:
  // Group frontier
  int num_frontiers = 3;
  int frontiers_N_max_tries = 50;
  int max_num_cached_points_to_keep = 50;
  double min_gain_frontier = 0.8;
  bool free_space_frontier = true;

  // Group General
  int planner_boundary_id = 0;
  double arm_height_min = 0.8;
  double arm_height_max = 1.0;
  double self_collision_height = 0.8;
  double arm_goal_range = 1.1;
  double raycast_dr = 0.01; // this should be the same as the resolution of the octomap
  double raycast_dphi = 10;
  double raycast_dtheta = 10;
  double gain_r_min = 0.2;
  double gain_r_max = 1.5; // this should be same as the max range of the octomap
  double camera_hfov = 86.0;
  double camera_vfov = 57.0;
  double visited_cells_grid_size = 0.5;
  double min_distance_to_obstacle = 0.3; // Min z of camera, or min distance the camera has to maintain from the obstacle
  bool compute_yaw_from_free_space = false;
  bool compute_yaw_from_measurement = true;

  // Group pathOptimizer
  int path_optimizer_sampling_target_count = 1;
  double shift_points_max_length = 2.0;
  double shift_points_threshold = 0.9;
  bool optimize_path_using_convex_hull = true;
  bool optimize_path_using_sampling = false;
  bool optimize_path_remove_points_inside_bbx = true;
  bool optimize_path_shift_points_towards_obstacles = true;

  // Group RRT
  int rrt_tree_length = 800; // number of nodes in the tree
  int rrt_N_max_tries = 67;  // maximum number of tries allowed for a q_new (child)
  double rrt_sampling_radius = 34.0;
  double rrt_collision_cyl_radius = 0.3;      // size of cylinder from q_near (parent) to q_new (child)
  double rrt_sample_threshold_distance = 0.5; // ensure sample threshold distance is lesser than rrt_step_size
  double rrt_step_size = 0.6;                 // distance between parent and child nodes of the tree
  bool sample_in_unknown = false;             // RRT can be grown in known or unknown space
  bool do_rrt_star = false;                   // RRT star takes too long

  // Group Utility
  double utility_lambda = 0.25;
  double utility_min_gain = 0.5;
  double utility_weight_measurement = 5.0;
  double utility_weight_free_space = 1.0;
  double utility_weight_visited_cell = 200.0;
  double utility_weight_euclidean_cost = 0.0;

  // normalization parameters
  double utility_max_value_free_space = 1.70;
  double utility_max_value_measurement = 500;
  double utility_max_value_visited_cell = 10.0;

  // No group
  std::vector<double> planner_boundary_x;
  std::vector<double> planner_boundary_y;
  std::string world_frame = "world";
  std::string robot_sampling_frame = "turret_link";

  NBVParameters()
  {
    loadStaticParameter(world_frame);
    loadStaticParameter(robot_sampling_frame);
    getROSUnsupportedTypeParameter("planner_boundary_x", planner_boundary_x); // not supported by [fkie_ddynamic_reconfigure]
    getROSUnsupportedTypeParameter("planner_boundary_y", planner_boundary_y); // not supported by [fkie_ddynamic_reconfigure]

    // Group Frontiers
    dynreconfRegIntGroup(num_frontiers, "Max number of frontier points to be extracted", 0, 50, "Frontiers");
    dynreconfRegIntGroup(frontiers_N_max_tries, "Max number of tries for RRT to expand and grow the frontier", 0, 100, "Frontiers");
    dynreconfRegIntGroup(max_num_cached_points_to_keep, "Max number of cached nodes to keep (sorted based on its gain", 0, 1000, "Frontiers");
    dynreconfRegDoubleGroup(min_gain_frontier, "min_gain_frontier", 0.0, 2.0, "Frontiers");
    dynreconfRegBoolGroup(free_space_frontier, "Frontiers are cached nodes either with high combined gain (free-space + measurement) or only free-space", "Frontiers");

    // Group General
    dynreconfRegIntGroup(planner_boundary_id, "planner_boundary_id", 0, 100, "General");

    dynreconfRegDoubleGroup(arm_height_min, "arm_height_min", 0.0, 1.0, "General");
    dynreconfRegDoubleGroup(arm_height_max, "arm_height_max", 1.0, 3.0, "General");
    dynreconfRegDoubleGroup(self_collision_height, "self_collision_height", 0.0, 3.0, "General");
    dynreconfRegDoubleGroup(arm_goal_range, "arm_goal_range", 0.0, 2.0, "General");

    dynreconfRegDoubleGroup(raycast_dr, "raycast_dr", 0.0, 0.5, "General");
    dynreconfRegDoubleGroup(raycast_dphi, "raycast_dphi", 0.0, 100.0, "General");
    dynreconfRegDoubleGroup(raycast_dtheta, "raycast_dtheta", 0.0, 100.0, "General");

    dynreconfRegDoubleGroup(gain_r_min, "gain_r_min", 0.0, 0.5, "General");
    dynreconfRegDoubleGroup(gain_r_max, "gain_r_max", 0.0, 5.0, "General");

    dynreconfRegDoubleGroup(camera_hfov, "camera_hfov", 0.0, 180.0, "General");
    dynreconfRegDoubleGroup(camera_vfov, "camera_vfov", 0.0, 180.0, "General");

    dynreconfRegDoubleGroup(visited_cells_grid_size, "Determines the grid size of the visited sparse grid", 0.1, 5.0, "General");
    dynreconfRegDoubleGroup(min_distance_to_obstacle, "Min z of camera, or min distance the camera has to maintain from the obstacle", 0.0, 0.5, "General");

    dynreconfRegBoolGroup(compute_yaw_from_free_space, "compute_yaw_from_free_space", "General");
    dynreconfRegBoolGroup(compute_yaw_from_measurement, "compute_yaw_from_measurement", "General");

    // Group PathOptimizer
    dynreconfRegIntGroup(path_optimizer_sampling_target_count, "Final number of points when using the sampling path optimizer", 1, 100.0, "PathOptimizer");
    dynreconfRegDoubleGroup(shift_points_max_length, "Max length to check for obstacles", 0.0, 7.0, "PathOptimizer");
    dynreconfRegDoubleGroup(shift_points_threshold, "Defines how close is the generated point from the obstacle", 0.0, 5.0, "PathOptimizer");
    dynreconfRegBoolGroup(optimize_path_using_convex_hull, "Optimize the path generated by the tree using 3D Convex Hulls", "PathOptimizer");
    dynreconfRegBoolGroup(optimize_path_using_sampling, "Optimize the path generated by the tree using up/down sampling", "PathOptimizer");
    dynreconfRegBoolGroup(optimize_path_remove_points_inside_bbx, "Remove points inside of the robot bounding box", "PathOptimizer");
    dynreconfRegBoolGroup(optimize_path_shift_points_towards_obstacles, "Move points towards/closer to obstacles using current octree", "PathOptimizer");

    // Group RRT
    dynreconfRegBoolGroup(sample_in_unknown, "sample_in_unknown", "RRT");
    dynreconfRegBoolGroup(do_rrt_star, "do_rrt_star", "RRT");
    dynreconfRegDoubleGroup(rrt_sampling_radius, "rrt_sampling_radius", 0.0, 50.0, "RRT");
    dynreconfRegIntGroup(rrt_tree_length, "Number of nodes in the tree", 0, 800, "RRT");
    dynreconfRegDoubleGroup(rrt_collision_cyl_radius, "Size of the cylinder for collision checking, before adding a new point to the rrt.", 0.0, 5.0, "RRT");
    dynreconfRegIntGroup(rrt_N_max_tries, "Maximum number of tries allowed for expanding the tree", 0, 500, "RRT");
    dynreconfRegDoubleGroup(rrt_sample_threshold_distance, "Distance between one sample to any one other sample", 0.0, 5.0, "RRT");
    dynreconfRegDoubleGroup(rrt_step_size, "Distance between parent and child nodes of the tree", 0.0, 5.0, "RRT");

    // Group Utility
    dynreconfRegDoubleGroup(utility_lambda, "utility_lambda", 0.0, 100.0, "Utility");
    dynreconfRegDoubleGroup(utility_min_gain, "utility_min_gain", 0.0, 100.0, "Utility");

    dynreconfRegDoubleGroup(utility_weight_measurement, "utility_weight_measurement", 0.0, 100.0, "Utility");
    dynreconfRegDoubleGroup(utility_weight_free_space, "utility_weight_free_space", 0.0, 100.0, "Utility");
    dynreconfRegDoubleGroup(utility_weight_visited_cell, "utility_weight_visited_cell", 0.0, 1000.0, "Utility");
    dynreconfRegDoubleGroup(utility_weight_euclidean_cost, "utility_weight_euclidean_cost", 0.0, 100.0, "Utility");

    dynreconfRegDoubleGroup(utility_max_value_free_space, "Max expected value for gain of free space", 0.0, 10.0, "Utility");
    dynreconfRegDoubleGroup(utility_max_value_measurement, "Max expected value for gain of measurement values", 0.0, 700.0, "Utility");
    dynreconfRegDoubleGroup(utility_max_value_visited_cell, "Max expected value for visited cells", 0.0, 50.0, "Utility");

    ddr->publishServicesTopics();
  }

  template <typename T>
  bool getROSUnsupportedTypeParameter(std::string name, T &param)
  {
    ros::NodeHandle private_node = ros::NodeHandle("~");
    const T default_value = param;
    bool r = private_node.param<T>(name, param, default_value);
    return r;
  }
};

#endif /* NBV_PARAMETERS_H_ */
