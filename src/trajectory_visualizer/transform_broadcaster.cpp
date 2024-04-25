#include "trajectory_visualizer/transform_broadcaster.hpp"

TransformBroadcaster::TransformBroadcaster(const std::string& frame_id, const std::string& child_frame_id)
    : frame_id_(frame_id), child_frame_id_(child_frame_id)
{
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
}

void TransformBroadcaster::setFrame(std::string frame_id)
{
    frame_id_ = frame_id;
}

void TransformBroadcaster::setChildFrame(std::string child_frame_id)
{
    child_frame_id_ = child_frame_id;
}

void TransformBroadcaster::broadcast(geometry_msgs::PoseStamped pose)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = pose.header.stamp;
    transformStamped.header.frame_id = frame_id_;
    transformStamped.child_frame_id = child_frame_id_;
    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;
    transformStamped.transform.rotation = pose.pose.orientation;
    broadcaster_->sendTransform(transformStamped);
}
