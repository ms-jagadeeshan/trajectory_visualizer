#include "trajectory_visualizer/trajectory_publisher.hpp"

TrajectoryPublisher::TrajectoryPublisher(std::string topic /* = "trajectory_marker" */)
    : nh_(new ros::NodeHandle()), pnh_(new ros::NodeHandle("~"))
{

    marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(topic, 1);
    init();
}

TrajectoryPublisher::TrajectoryPublisher(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh, std::string topic)
    : nh_(nh), pnh_(pnh), marker_pub_(nh->advertise<visualization_msgs::MarkerArray>(topic, 1)), markerArray_()
{
    init();
}

void TrajectoryPublisher::init()
{
    pnh_->param<std::string>("map_frame", map_frame_, "/map");
    pnh_->param<std::string>("base_frame", base_frame_, "/base_link");
    pnh_->param<double>("publish_rate", frequency_, 10);

    tf_listener_ = std::make_shared<TransformListener>(map_frame_, base_frame_, frequency_);
}

void TrajectoryPublisher::updateTrajectory(geometry_msgs::PoseStamped pose)
{
    visualization_msgs::Marker marker;
    marker.header = pose.header;
    marker.ns = "trajectory";
    marker.id = markerArray_.markers.size(); // Assign a unique ID for each marker
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.1; // Sphere diameter
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0; // Red color
    marker.color.a = 1.0; // Fully opaque
    markerArray_.markers.push_back(marker);
    marker_pub_.publish(markerArray_);
}

void TrajectoryPublisher::clearAll()
{
    markerArray_.markers.clear();
    visualization_msgs::Marker marker;
    marker.ns = "trajectory";
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray_.markers.push_back(marker);
    marker_pub_.publish(markerArray_);
    markerArray_.markers.clear();
}
void TrajectoryPublisher::updateTrajectory(const std::vector<geometry_msgs::PoseStamped>& trajectory)
{
    for (auto& pose : trajectory)
    {
        visualization_msgs::Marker marker;
        marker.header = pose.header;
        marker.ns = "trajectory";
        marker.id = markerArray_.markers.size(); // Assign a unique ID for each marker
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose.pose;
        marker.scale.x = 0.1; // Sphere diameter
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0; // Red color
        marker.color.a = 1.0; // Fully opaque
        markerArray_.markers.push_back(marker);
    }
    marker_pub_.publish(markerArray_);
}

void TrajectoryPublisher::setFrequency(double frequency)
{
    this->frequency_ = frequency;
}

void TrajectoryPublisher::run()
{
    ros::Rate rate(frequency_);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped poseStamped = tf_listener_->getPose();
        updateTrajectory({poseStamped});
        rate.sleep();
    }
}
