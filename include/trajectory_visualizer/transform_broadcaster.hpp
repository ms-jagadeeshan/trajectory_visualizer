#ifndef TRAJECTORY_VISUALIZER_TRANSFORM_BROADCASTER_HPP
#define TRAJECTORY_VISUALIZER_TRANSFORM_BROADCASTER_HPP

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// Wrapper class on tf2 transform broadcaster
class TransformBroadcaster {
public:
    TransformBroadcaster(const std::string& frame_id, const std::string& child_frame_id);
    void setFrame(std::string frame_id);
    void setChildFrame(std::string child_frame_id);
    geometry_msgs::TransformStamped poseStampedtoTransformStamped(const geometry_msgs::PoseStamped& transformStamped);
    // Broadcast single pose.
    void broadcast(geometry_msgs::PoseStamped pose);
    // Broadcast multiple poses.
    void broadcast(std::shared_ptr<std::vector<geometry_msgs::PoseStamped>>& poses);

private:
    // Parent frame.
    std::string frame_id_;
    // Child frame.
    std::string child_frame_id_;
    // Transform Broadcaster.
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

#endif
