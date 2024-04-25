#ifndef TRAJECTORY_VISUALIZER_IO_TYPES_HPP
#define TRAJECTORY_VISUALIZER_IO_TYPES_HPP

#include <map>

enum IOType
{
    JSON_TYPE,
    YAML_TYPE,
    CSV_TYPE,
    UNKNOWN
};
static std::map<std::string, IOType> StringToSerializerType = {
    {"JSON", JSON_TYPE},
    {"YAML", YAML_TYPE},
    {"CSV", CSV_TYPE}};

#endif
