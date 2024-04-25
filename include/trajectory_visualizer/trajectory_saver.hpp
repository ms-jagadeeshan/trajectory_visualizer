#ifndef TRAJECTORY_VISUALIZER_TRAJECTORY_SAVER_HPP
#define TRAJECTORY_VISUALIZER_TRAJECTORY_SAVER_HPP

#include "ros/ros.h"
#include "trajectory_visualizer/SaveTrajectory.h"
#include "trajectory_visualizer/transform_listener.hpp"

/*  Params          Type      Default Value
 *  map_frame      string      /map
 *  base_frame     string      /base_link
 *  publish_rate   double      10
 */
class TrajectorySaver {
public:
    // Constructor.
    TrajectorySaver();
    TrajectorySaver(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh);

    // Initialize.
    void init();
    // Trajectory saver service.
    bool saverService(trajectory_visualizer::SaveTrajectory::Request& req, trajectory_visualizer::SaveTrajectory::Response& res);

private:
    // Node Handle.
    ros::NodeHandlePtr nh_;
    // Private Node Handle.
    ros::NodeHandlePtr pnh_;
    // Frequency.
    double frequency_;
    // Map frame name.
    std::string map_frame_;
    // Base frame name.
    std::string base_frame_;
    // Trajectory Saver service.
    ros::ServiceServer service_;
    // Transform Listener.
    std::shared_ptr<TransformListener> tf_listener_;
};

#endif
