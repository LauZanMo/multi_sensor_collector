aux_source_directory(. TEST_SOURCE)
add_executable(h52bag ${TEST_SOURCE})
target_link_libraries(h52bag multi_sensor_collector)

# ROS
find_package(catkin REQUIRED COMPONENTS
    rosbag
    sensor_msgs
    dvs_msgs
)
target_link_libraries(h52bag ${catkin_LIBRARIES})
target_include_directories(h52bag PUBLIC ${catkin_INCLUDE_DIRS})