#include "trajectory_visualizer/backward.hpp"
#include "trajectory_visualizer/trajectory_publisher.hpp"
#include "trajectory_visualizer/trajectory_saver.hpp"

// Uncomment to enable backtracing
// namespace backward
// {
// backward::SignalHandling sh;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_publisher_saver_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle(""));
    ros::NodeHandlePtr pnh(new ros::NodeHandle("~"));

    TrajectoryPublisher trajectory_pub(nh, pnh, "trajectory_marker");
    TrajectorySaver trajectory_saver(nh, pnh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    trajectory_pub.run();
}
