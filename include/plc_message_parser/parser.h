#ifndef PARSER_H
#define PARSER_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <plc_message_parser/ParsedPLCMessage.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>

struct PLCField {
    std::string tag_name;
    std::string address;
    std::string type;
    std::string topic_name;
    std::string field_name;
    std::string data_type;
};

class PLCParser {
public:
    PLCParser(ros::NodeHandle& nh, const std::string& config_file);
    void parseMessage(const std::vector<uint8_t>& raw_data);

private:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, PLCField> fields_;

    void loadConfig(const std::string& config_file);
};

#endif // PARSER_H

