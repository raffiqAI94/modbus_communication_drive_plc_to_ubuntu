#ifndef CONSTRUCTOR_H
#define CONSTRUCTOR_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <plc_message_constructor/ConstructedPLCMessage.h>
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

class PLCConstructor {
public:
    PLCConstructor(ros::NodeHandle& nh, const std::string& config_file);
    void handleMessage(const plc_message_constructor::ConstructedPLCMessage::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, PLCField> fields_;

    void loadConfig(const std::string& config_file);
    void writeToPLC(const std::string& address, uint64_t value);
};

#endif // CONSTRUCTOR_H

