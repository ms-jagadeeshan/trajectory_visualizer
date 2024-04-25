#include "trajectory_visualizer/json_io.hpp"
#include "trajectory_visualizer/json.hpp"

bool JsonIO::saveTrajectory(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& ps)
{
    nlohmann::json j;
    for (const auto& pose : ps)
    {
        j.push_back({{"header", {{"seq", pose.header.seq}, {"stamp", pose.header.stamp.toSec()}, {"frame_id", pose.header.frame_id}}}, {"pose", {{"position", {{"x", pose.pose.position.x}, {"y", pose.pose.position.y}, {"z", pose.pose.position.z}}}, {"orientation", {{"x", pose.pose.orientation.x}, {"y", pose.pose.orientation.y}, {"z", pose.pose.orientation.z}, {"w", pose.pose.orientation.w}}}}}});
    }
    std::string jsonString = j.dump(4);
    return writeStringToFile(filename, jsonString);
}

std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> JsonIO::readTrajectory(const std::string& filename)
{
    std::string jsonString = readFileToString(filename);
    if (jsonString.empty())
    {
        ROS_ERROR("%s is empty", filename.c_str());
        return nullptr;
    }
    auto j = nlohmann::json::parse(jsonString);
    auto trajectory = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
    ROS_INFO("Trajectory Size : %zu", j.size());
    for (const auto& jsonPose : j)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.seq = jsonPose["header"]["seq"].get<uint>();
        pose.header.stamp = ros::Time(jsonPose["header"]["stamp"].get<double>());
        pose.header.frame_id = jsonPose["header"]["frame_id"].get<std::string>();

        pose.pose.position.x = jsonPose["pose"]["position"]["x"].get<double>();
        pose.pose.position.y = jsonPose["pose"]["position"]["y"].get<double>();
        pose.pose.position.z = jsonPose["pose"]["position"]["z"].get<double>();
        pose.pose.orientation.x = jsonPose["pose"]["orientation"]["x"].get<double>();
        pose.pose.orientation.y = jsonPose["pose"]["orientation"]["y"].get<double>();
        pose.pose.orientation.z = jsonPose["pose"]["orientation"]["z"].get<double>();
        pose.pose.orientation.w = jsonPose["pose"]["orientation"]["w"].get<double>();

        trajectory->push_back(pose);
    }

    return trajectory;
}
