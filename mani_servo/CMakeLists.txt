cmake_minimum_required(VERSION 3.8)
project(mani_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(
  mani_joy SHARED
  src/mani_joy.cpp
  )

#add_executable(mani_joy src/mani_joy.cpp)
ament_target_dependencies(
    mani_joy 
    rclcpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    control_msgs 
    std_srvs 
    moveit_msgs
    rclcpp_components
)
#target_link_libraries(mani_joy)
rclcpp_components_register_nodes(mani_joy "moveit_servo::JoyToServoPub")


install(TARGETS
  mani_joy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
