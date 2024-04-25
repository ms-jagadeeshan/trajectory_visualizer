#ifndef TRAJECTORY_VISUALIZER_BASE_IO_HPP
#define TRAJECTORY_VISUALIZER_BASE_IO_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Abstract class for file Input/Output.
class BaseIO {
public:
    virtual bool saveTrajectory(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& ps) = 0;
    virtual std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> readTrajectory(const std::string& filename) = 0;

protected:
    std::string readFileToString(const std::string& filename);
    bool writeStringToFile(const std::string& filename, const std::string& content);
};

#endif
