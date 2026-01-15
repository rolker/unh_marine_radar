cmake_minimum_required(VERSION 3.8)
project(marine_radar_tracker)

set(ROS_DEPENDS
    geometry_msgs
    grid_map_ros
    marine_sensor_msgs
    pcl_ros
    marine_interfaces
    rclcpp
    rosbag2
    sensor_msgs
    tf2
    tf2_ros
    visualization_msgs
)

set(EXTERNAL_DEPENDS
)

set(EXTERNAL_DEPENDS_LIB
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL 1.12 REQUIRED)

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

# Build marine_radar_tracker node:
add_executable(${PROJECT_NAME} src/marine_radar_tracker.cpp)
target_include_directories(${PROJECT_NAME}
   PRIVATE
   "include/${PROJECT_NAME}/"
)
ament_target_dependencies(${PROJECT_NAME}
   ${ROS_DEPENDS}
   ${EXTERNAL_DEPENDS_LIB}
)


# Build marine_radar_to_pointcloud node:
add_executable(marine_radar_to_pointcloud src/marine_radar_to_pointcloud.cpp)
ament_target_dependencies(marine_radar_to_pointcloud
    ${ROS_DEPENDS}
    ${EXTERNAL_DEPENDS_LIB}
)
target_link_libraries(marine_radar_to_pointcloud ${PCL_LIBRARIES})

message("  Installing: " ${EXECUTABLES})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS marine_radar_to_pointcloud #  ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
