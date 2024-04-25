#include "trajectory_visualizer/base_io.hpp"
#include <fstream>

std::string BaseIO::readFileToString(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file %s for reading.", filename.c_str());
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    file.close();
    return content;
}

bool BaseIO::writeStringToFile(const std::string& filename, const std::string& content)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file %s for writing.", filename.c_str());
        return false;
    }

    file << content;
    file.close();
    return true;
}
