#include "trajectory_visualizer/trajectory_io.hpp"
#include <filesystem>
#include "trajectory_visualizer/csv_io.hpp"
#include "trajectory_visualizer/json_io.hpp"
#include "trajectory_visualizer/yaml_io.hpp"

std::string getFileExtension(const std::string& filename)
{
    std::filesystem::path filePath(filename);
    if (filePath.has_extension())
    {
        return filePath.extension().string();
    }
    else
    {
        return "";
    }
}
bool TrajectoryIO::saveTrajectory(const std::string& filename, const std::shared_ptr<std::vector<geometry_msgs::PoseStamped>>& ps)
{
    std::shared_ptr<BaseIO> serializer = getSerializer(filename);
    if (!serializer)
    {
        ROS_ERROR("Failed to obtain serializer.");
        return false;
    }

    return serializer->saveTrajectory(filename, *ps);
}

std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> TrajectoryIO::readTrajectory(const std::string& filename)
{
    std::shared_ptr<BaseIO> serializer = getSerializer(filename);
    if (!serializer)
    {
        ROS_ERROR("Failed to obtain serializer.");
        return nullptr;
    }

    return serializer->readTrajectory(filename);
}

std::shared_ptr<BaseIO> TrajectoryIO::getSerializer(const std::string& filename)
{
    std::string extension = getFileExtension(filename);
    IOType type = IOType::UNKNOWN;
    ROS_INFO("File ext: %s", extension.c_str());
    if (extension == ".json")
    {
        type = IOType::JSON_TYPE;
    }
    else if (extension == ".yaml" || extension == ".yml")
    {
        type = IOType::YAML_TYPE;
    }
    else if (extension == ".csv")
    {
        type = IOType::CSV_TYPE;
    }
    else
    {
        type = IOType::UNKNOWN;
    }
    return getSerializer(type);
}

std::shared_ptr<BaseIO> TrajectoryIO::getSerializer(IOType type)
{
    switch (type)
    {
    case JSON_TYPE:
        return std::make_shared<JsonIO>();
    case YAML_TYPE:
        return std::make_shared<YamlIO>();
    case CSV_TYPE:
        return std::make_shared<CsvIO>();
    default:
        ROS_ERROR("Unsupported serializer type.");
        return nullptr;
    }
}
