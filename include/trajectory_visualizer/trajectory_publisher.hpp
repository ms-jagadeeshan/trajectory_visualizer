#ifndef TRAJECTORY_VISUALIZER_TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_VISUALIZER_TRAJECTORY_PUBLISHER_HPP

#include "trajectory_visualizer/transform_listener.hpp"
#include "visualization_msgs/MarkerArray.h"

/*  Params          Type      Default Value
 *  map_frame      string      /map
 *  base_frame     string      /base_link
 *  publish_rate   double      10
 */
class TrajectoryPublisher {
public:
    // Constructor.
    TrajectoryPublisher(const std::string topic = "trajectory_marker");
    TrajectoryPublisher(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh, const std::string topic);
    void setFrequency(double rate_hz);
    // Append single marker to the markerArray.
    void updateTrajectory(geometry_msgs::PoseStamped pose);
    // Append multiple marker to the markerArray.
    void updateTrajectory(const std::vector<geometry_msgs::PoseStamped>& trajectory);
    // Clear all the existing markers.
    void clearAll();
    // Initialize.
    void init();
    // Run based on tf_listener.
    void run();

private:
    // Node Handle.
    ros::NodeHandlePtr nh_;
    // Private Node Handle.
    ros::NodeHandlePtr pnh_;
    // Marker publisher.
    ros::Publisher marker_pub_;
    // Frequency.
    double frequency_;
    // Map frame name.
    std::string map_frame_;
    // Base frame name.
    std::string base_frame_;
    // Transform Listener.
    std::shared_ptr<TransformListener> tf_listener_;
    // Marker array of trajectory.
    visualization_msgs::MarkerArray markerArray_;
};

#endif
