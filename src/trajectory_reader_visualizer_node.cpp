#include "ros/ros.h"
#include "trajectory_visualizer/backward.hpp"
#include "trajectory_visualizer/trajectory_reader.hpp"

// Uncomment to enable backtracing.
// namespace backward
// {
// backward::SignalHandling sh;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_reader_visualizer_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle(""));
    ros::NodeHandlePtr pnh(new ros::NodeHandle("~"));

    TrajectoryReader reader(nh, pnh, "trajectory_marker");
    ros::spin();
}
