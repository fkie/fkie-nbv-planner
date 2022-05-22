#include <fkie_nbv_planner/NBVPlanner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NBVPlanner");
  ros::NodeHandle nh;

  fkie_nbv_planner::NBVPlanner NBVPlanner(nh);

  ros::spin();
  return 0;
}