#include "plc_message_constructor/constructor.h"
#include <modbus/modbus.h>
#include <iostream>

PLCConstructor::PLCConstructor(ros::NodeHandle& nh, const std::string& config_file) : nh_(nh) {
    loadConfig(config_file);

    for (const auto& [address, field] : fields_) {
        ros::Subscriber sub = nh_.subscribe<plc_message_constructor::ConstructedPLCMessage>(
            field.topic_name, 10, &PLCConstructor::handleMessage, this);
    }
}

void PLCConstructor::loadConfig(const std::string& config_file) {
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
    }
}

void PLCConstructor::handleMessage(const plc_message_constructor::ConstructedPLCMessage::ConstPtr& msg) {
    for (const auto& [address, field] : fields_) {
        if (field.field_name == msg->field_name) {
            writeToPLC(field.address, msg->value);
        }
    }
}

void PLCConstructor::writeToPLC(const std::string& address, uint64_t value) {
    // Set up a MODBUS connection
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1); // Adjust the parameters as per your setup
    if (ctx == nullptr) {
        ROS_ERROR("Failed to create MODBUS context.");
        return;
    }

    if (modbus_connect(ctx) == -1) {
        ROS_ERROR("Failed to connect to MODBUS.");
        modbus_free(ctx);
        return;
    }

    // Parse the address string to get the register address
    try {
        int reg_address = std::stoi(address.substr(1), nullptr, 16); // Convert "D0" to integer
        int write_result = modbus_write_register(ctx, reg_address, static_cast<uint16_t>(value));

        if (write_result == -1) {
            ROS_ERROR("Failed to write value %lu to register %d.", value, reg_address);
        } else {
            ROS_INFO("Successfully wrote value %lu to register %d.", value, reg_address);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error parsing address %s: %s", address.c_str(), e.what());
    }

    // Clean up the MODBUS connection
    modbus_close(ctx);
    modbus_free(ctx);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plc_constructor_node");
  ros::NodeHandle nh;

  PLCConstructor constructor(nh, "config/plc_constructor_config.yaml");

  ros::spin();
  return 0;
}

