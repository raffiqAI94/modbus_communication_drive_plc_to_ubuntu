#ifndef PLC_DRIVER_H
#define PLC_DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <modbus/modbus.h>

class PLCDriver
{
public:
    PLCDriver(ros::NodeHandle& nh);
    ~PLCDriver();

    void readModbus();
    void writeModbusBool(const std_msgs::Bool::ConstPtr& msg);
    void writeModbusInt(const std_msgs::Int32::ConstPtr& msg);
    void writeModbusFloat(const std_msgs::Float32::ConstPtr& msg);
    void writeModbusPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    modbus_t* modbus_ctx_;

    ros::Publisher pub_bool_;
    ros::Publisher pub_int_;
    ros::Publisher pub_float_;
    ros::Publisher pub_PoseStamped_;

    ros::Subscriber sub_bool_;
    ros::Subscriber sub_int_;
    ros::Subscriber sub_float_;
    ros::Subscriber sub_PoseStamped_;

    std::string modbus_port_;
    int modbus_baudrate_;
    int modbus_timeout_;
    std::string modbus_parity_;

    void modbusConnect();
    void modbusDisconnect();
};

#endif

