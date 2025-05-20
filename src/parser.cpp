#include "plc_message_parser/parser.h"
#include <std_msgs/UInt64.h>
#include <iostream>

PLCParser::PLCParser(ros::NodeHandle& nh, const std::string& config_file) : nh_(nh) {
    loadConfig(config_file);
}

void PLCParser::loadConfig(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    for (const auto& message : config["messages"]) {
        PLCField field;
        field.tag_name = message["tag_name"].as<std::string>();
        field.address = message["address"].as<std::string>();
        field.type = message["type"].as<std::string>();
        field.topic_name = message["topic_name"].as<std::string>();
        field.field_name = message["field_name"].as<std::string>();
        field.data_type = message["data_type"].as<std::string>();

        fields_[field.address] = field;
        publishers_[field.topic_name] = nh_.advertise<plc_message_parser::ParsedPLCMessage>(field.topic_name, 10);
    }
}

void PLCParser::parseMessage(const std::vector<uint8_t>& raw_data) {
    for (const auto& [address, field] : fields_) {
        plc_message_parser::ParsedPLCMessage msg;
        msg.header.stamp = ros::Time::now();
        msg.tag_name = field.tag_name;
        msg.field_name = field.field_name;

        // Extract and convert raw data based on field address and type
        if (field.data_type == "uint64") {
            uint64_t value = 0;
            memcpy(&value, &raw_data[std::stoi(field.address.substr(1))], sizeof(uint64_t));
            msg.value = value;
        }
        publishers_[field.topic_name].publish(msg);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plc_parser_node");
  ros::NodeHandle nh;

  PLCParser parser(nh, "config/plc_parser_config.yaml");

  // Example: Subscribe to a topic that provides raw data
  ros::Subscriber sub = nh.subscribe("raw_plc_data", 10, &PLCParser::parseMessage, &parser);

  ros::spin();
  return 0;
}

