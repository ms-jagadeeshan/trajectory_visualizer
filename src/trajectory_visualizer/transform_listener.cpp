#include "trajectory_visualizer/transform_listener.hpp"

TransformListener::TransformListener(const std::string& frame_id, const std::string& child_frame_id, double frequency)
    : frame_id_(frame_id), child_frame_id_(child_frame_id), frequency_(frequency)
{
    tfBuffer_.reset(new tf2_ros::Buffer(ros::Duration(1.0)));
    listener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
}

void TransformListener::setFrame(std::string frame_id)
{
    frame_id_ = frame_id;
}

void TransformListener::setChildFrame(std::string child_frame_id)
{
    child_frame_id_ = child_frame_id;
}

geometry_msgs::PoseStamped TransformListener::transformStampedToPoseStamped(const geometry_msgs::TransformStamped& transformStamped)
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = transformStamped.header;
    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;
    poseStamped.pose.orientation = transformStamped.transform.rotation;
    return poseStamped;
}
geometry_msgs::PoseStamped TransformListener::getPose()
{
    try
    {
        auto transform = tfBuffer_->lookupTransform(frame_id_, child_frame_id_, ros::Time(0));
        return transformStampedToPoseStamped(transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Failed to lookup transform: %s", ex.what());
        return geometry_msgs::PoseStamped();
    }
}
std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> TransformListener::getPoses(ros::Duration d)
{
    auto poses = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();

    ros::Time startTime = ros::Time::now();
    ros::Rate rate(frequency_);
    while ((ros::Time::now() - startTime) < d)
    {
        try
        {
            auto transformStamped = tfBuffer_->lookupTransform(frame_id_, child_frame_id_, ros::Time(0));
            poses->push_back(transformStampedToPoseStamped(transformStamped));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("Failed to lookup transform: %s", ex.what());
        }
        rate.sleep();
    }

    return poses;
}
