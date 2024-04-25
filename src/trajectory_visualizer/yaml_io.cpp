#include "trajectory_visualizer/yaml_io.hpp"
#include "yaml-cpp/yaml.h"

bool YamlIO::saveTrajectory(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& trajectory)
{
    YAML::Emitter emitter;

    emitter << YAML::BeginMap;
    emitter << YAML::Key << "poses";
    emitter << YAML::Value << YAML::BeginSeq;

    for (const auto& pose : trajectory)
    {
        emitter << YAML::BeginMap;

        emitter << YAML::Key << "header";
        emitter << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "seq" << YAML::Value << pose.header.seq;
        emitter << YAML::Key << "stamp" << YAML::Value << pose.header.stamp.toSec();
        emitter << YAML::Key << "frame_id" << YAML::Value << pose.header.frame_id;
        emitter << YAML::EndMap;

        emitter << YAML::Key << "pose";
        emitter << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "position";
        emitter << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "x" << YAML::Value << pose.pose.position.x;
        emitter << YAML::Key << "y" << YAML::Value << pose.pose.position.y;
        emitter << YAML::Key << "z" << YAML::Value << pose.pose.position.z;
        emitter << YAML::EndMap;
        emitter << YAML::Key << "orientation";
        emitter << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "x" << YAML::Value << pose.pose.orientation.x;
        emitter << YAML::Key << "y" << YAML::Value << pose.pose.orientation.y;
        emitter << YAML::Key << "z" << YAML::Value << pose.pose.orientation.z;
        emitter << YAML::Key << "w" << YAML::Value << pose.pose.orientation.w;
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;

        emitter << YAML::EndMap;
    }

    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
    std::string yamlString = emitter.c_str();
    return writeStringToFile(filename, yamlString);
}

std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> YamlIO::readTrajectory(const std::string& filename)
{
    std::string yamlString = readFileToString(filename);
    if (yamlString.empty())
    {
        return nullptr;
    }

    auto trajectory = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
    try
    {
        YAML::Node yaml = YAML::LoadFile(filename);
        ROS_ERROR("Invali fi key not found. %s", filename.c_str());
        if (!yaml["poses"])
        {
            ROS_ERROR("Invalid YAML file format: 'poses' key not found.");
            return nullptr;
        }

        const YAML::Node& poses = yaml["poses"];
        if (!poses.IsSequence())
        {
            ROS_ERROR("Invalid YAML file format: 'poses' is not a sequence.");
            return nullptr;
        }

        for (const auto& poseNode : poses)
        {
            geometry_msgs::PoseStamped pose;

            const YAML::Node& headerNode = poseNode["header"];
            pose.header.seq = headerNode["seq"].as<uint32_t>();
            pose.header.stamp = ros::Time(headerNode["stamp"].as<double>());
            pose.header.frame_id = headerNode["frame_id"].as<std::string>();

            const YAML::Node& poseData = poseNode["pose"];
            const YAML::Node& positionNode = poseData["position"];
            pose.pose.position.x = positionNode["x"].as<double>();
            pose.pose.position.y = positionNode["y"].as<double>();
            pose.pose.position.z = positionNode["z"].as<double>();

            const YAML::Node& orientationNode = poseData["orientation"];
            pose.pose.orientation.x = orientationNode["x"].as<double>();
            pose.pose.orientation.y = orientationNode["y"].as<double>();
            pose.pose.orientation.z = orientationNode["z"].as<double>();
            pose.pose.orientation.w = orientationNode["w"].as<double>();

            trajectory->push_back(pose);
        }
    }
    catch (const YAML::ParserException& e)
    {
        ROS_ERROR_STREAM("Failed to parse YAML file: " << e.what());
    }
    catch (const YAML::BadFile& e)
    {
        ROS_ERROR_STREAM("Failed to open YAML file: " << e.what());
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Caught exception " << e.what());
    }

    return trajectory;
}
