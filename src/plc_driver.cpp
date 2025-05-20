#include "plc_driver/plc_driver.h"
#include <geometry_msgs/PoseStamped.h> // Updated header inclusion

PLCDriver::PLCDriver(ros::NodeHandle& nh) : nh_(nh)
{
    // Load parameters
    nh_.param<std::string>("modbus/port", modbus_port_, "/dev/ttyUSB0");
    nh_.param<int>("modbus/baudrate", modbus_baudrate_, 9600);
    nh_.param<int>("modbus/timeout", modbus_timeout_, 1);
    nh_.param<std::string>("modbus/parity", modbus_parity_, "N");

    // Publishers
    pub_bool_ = nh_.advertise<std_msgs::Bool>("plc/bool", 10);
    pub_int_ = nh_.advertise<std_msgs::Int32>("plc/int", 10);
    pub_float_ = nh_.advertise<std_msgs::Float32>("plc/float", 10);
    pub_PoseStamped_ = nh_.advertise<geometry_msgs::PoseStamped>("plc/PoseStamped", 10);

    // Subscribers
    sub_bool_ = nh_.subscribe("plc/write_bool", 10, &PLCDriver::writeModbusBool, this);
    sub_int_ = nh_.subscribe("plc/write_int", 10, &PLCDriver::writeModbusInt, this);
    sub_float_ = nh_.subscribe("plc/write_float", 10, &PLCDriver::writeModbusFloat, this);
    sub_PoseStamped_ = nh_.subscribe("plc/write_PoseStamped", 10, &PLCDriver::writeModbusPoseStamped, this);

    // Connect to MODBUS
    modbusConnect();
}

PLCDriver::~PLCDriver()
{
    modbusDisconnect();
}

void PLCDriver::modbusConnect()
{
    modbus_ctx_ = modbus_new_rtu(modbus_port_.c_str(), modbus_baudrate_, modbus_parity_[0], 8, 1);
    if (modbus_ctx_ == nullptr) {
        ROS_ERROR("Failed to create MODBUS context.");
        return;
    }

    if (modbus_connect(modbus_ctx_) == -1) {
        ROS_ERROR("Failed to connect to MODBUS.");
        modbus_free(modbus_ctx_);
    }
}

void PLCDriver::modbusDisconnect()
{
    if (modbus_ctx_ != nullptr) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
    }
}

void PLCDriver::readModbus()
{
    uint16_t reg;
    if (modbus_read_registers(modbus_ctx_, 0, 1, &reg) > 0) {
        std_msgs::Int32 msg;
        msg.data = reg;
        pub_int_.publish(msg);
    }
}

void PLCDriver::writeModbusBool(const std_msgs::Bool::ConstPtr& msg)
{
    uint8_t value = msg->data ? 1 : 0;
    modbus_write_bit(modbus_ctx_, 0, value);
}

void PLCDriver::writeModbusInt(const std_msgs::Int32::ConstPtr& msg)
{
    uint16_t value = static_cast<uint16_t>(msg->data);
    modbus_write_register(modbus_ctx_, 0, value);
}

void PLCDriver::writeModbusFloat(const std_msgs::Float32::ConstPtr& msg)
{
    float value = msg->data;
    uint16_t regs[2];
    memcpy(regs, &value, sizeof(value));
    modbus_write_registers(modbus_ctx_, 0, 2, regs);
}

void PLCDriver::writeModbusPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    float position[3] = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    float orientation[4] = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};

    uint16_t position_regs[6]; // 3 floats for position
    uint16_t orientation_regs[8]; // 4 floats for orientation

    memcpy(position_regs, position, sizeof(position));
    memcpy(orientation_regs, orientation, sizeof(orientation));

    // Assuming MODBUS memory mapping for position and orientation
    modbus_write_registers(modbus_ctx_, 0, 6, position_regs);       // Write position (6 registers)
    modbus_write_registers(modbus_ctx_, 6, 8, orientation_regs);   // Write orientation (8 registers)
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plc_driver_node"); // Match the name in CMakeLists.txt
    ros::NodeHandle nh;

    PLCDriver driver(nh);
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        driver.readModbus();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
