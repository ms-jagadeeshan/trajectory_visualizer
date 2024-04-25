#ifndef TRAJECTORY_VISUALIZER_TRANSFORM_LISTENER_HPP
#define TRAJECTORY_VISUALIZER_TRANSFORM_LISTENER_HPP

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Wrapper class on tf2 Transform listener
class TransformListener {
public:
    TransformListener(const std::string& frame_id, const std::string& child_frame_id, double frequency);

    void setFrame(std::string frame_id);
    void setChildFrame(std::string child_frame_id);
    geometry_msgs::PoseStamped transformStampedToPoseStamped(const geometry_msgs::TransformStamped& transformStamped);
    geometry_msgs::PoseStamped getPose();
    std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> getPoses(ros::Duration d);

private:
    // Listen Rate.
    double frequency_;
    // Source frame.
    std::string frame_id_;
    // Target frame.
    std::string child_frame_id_;
    // Transform buffer.
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    // Transform Listener.
    std::shared_ptr<tf2_ros::TransformListener> listener_;
};

#endif
