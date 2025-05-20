cmake_minimum_required(VERSION 3.0.2)
project(plc_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  yaml-cpp
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plc_driver
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs dynamic_reconfigure 
)

#Ensure libmodbus is installed on our system (e.g., using apt-get install libmodbus-dev on Ubuntu)
#If the compiler can't find <modbus/modbus.h>, we might need to add the include directory for libmodbus to our include_directories in CMakeLists.txt

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plc_driver_node src/plc_driver.cpp)
target_link_libraries(plc_driver_node ${catkin_LIBRARIES} modbus)

add_executable(plc_parser_node src/parser.cpp)
target_link_libraries(plc_parser_node ${catkin_LIBRARIES} modbus)

add_executable(plc_constructor_node src/constructor.cpp)
target_link_libraries(plc_constructor_node ${catkin_LIBRARIES} modbus)

# Add and generate messages
add_message_files(FILES
  ParsedPLCMessage.msg
  ConstructedPLCMessage.msg
)
generate_messages(
DEPENDENCIES std_msgs

