#ifndef TRAJECTORY_VISUALIZER_TRAJECTORY_IO_HPP
#define TRAJECTORY_VISUALIZER_TRAJECTORY_IO_HPP

#include "trajectory_visualizer/base_io.hpp"
#include "trajectory_visualizer/io_types.hpp"

// Trajectory data input/output handler class.
class TrajectoryIO {
public:
    static bool saveTrajectory(const std::string& filename, const std::shared_ptr<std::vector<geometry_msgs::PoseStamped>>& ps);
    static std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> readTrajectory(const std::string& filename);

private:
    static std::shared_ptr<BaseIO> getSerializer(IOType type);
    static std::shared_ptr<BaseIO> getSerializer(const std::string& filename);
};

#endif
