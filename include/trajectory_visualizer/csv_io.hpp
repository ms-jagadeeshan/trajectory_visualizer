#ifndef TRAJECTORY_VISUALIZER_CSV_IO_HPP
#define TRAJECTORY_VISUALIZER_CSV_IO_HPP

#include "trajectory_visualizer/base_io.hpp"

// Implementation class for CSV format.
class CsvIO : public BaseIO {
public:
    bool saveTrajectory(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& ps);
    std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> readTrajectory(const std::string& filename);
};

#endif
