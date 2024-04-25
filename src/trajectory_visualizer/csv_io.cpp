#include "trajectory_visualizer/csv_io.hpp"

bool CsvIO::saveTrajectory(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& ps)
{
    std::stringstream ss;
    for (const auto& pose : ps)
    {
        ss << pose.header.seq << ",";
        ss << pose.header.stamp.toSec() << ",";
        ss << pose.header.frame_id << ",";
        ss << pose.pose.position.x << ",";
        ss << pose.pose.position.y << ",";
        ss << pose.pose.position.z << ",";
        ss << pose.pose.orientation.x << ",";
        ss << pose.pose.orientation.y << ",";
        ss << pose.pose.orientation.z << ",";
        ss << pose.pose.orientation.w << "\n";
    }

    return writeStringToFile(filename, ss.str());
}

std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> CsvIO::readTrajectory(const std::string& filename)
{
    std::string csvString = readFileToString(filename);
    if (csvString.empty())
    {
        return nullptr;
    }

    auto trajectory = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
    std::istringstream ss(csvString);
    std::string line;
    while (std::getline(ss, line))
    {
        std::istringstream lineStream(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(lineStream, token, ','))
        {
            tokens.push_back(token);
        }
        if (tokens.size() != 10)
        {
            ROS_WARN("Skipping invalid CSV line: %s", line.c_str());
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.seq = std::stoi(tokens[0]);
        pose.header.stamp = ros::Time(std::stod(tokens[1]));
        pose.header.frame_id = tokens[2];
        pose.pose.position.x = std::stod(tokens[3]);
        pose.pose.position.y = std::stod(tokens[4]);
        pose.pose.position.z = std::stod(tokens[5]);
        pose.pose.orientation.x = std::stod(tokens[6]);
        pose.pose.orientation.y = std::stod(tokens[7]);
        pose.pose.orientation.z = std::stod(tokens[8]);
        pose.pose.orientation.w = std::stod(tokens[9]);
        trajectory->push_back(pose);
    }

    return trajectory;
}
