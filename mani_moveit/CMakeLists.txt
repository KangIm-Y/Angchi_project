cmake_minimum_required(VERSION 3.8)
project(mani_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_srvs REQUIRED)  # 추가: std_srvs 패키지 필요

# 기존 mani_moveit 실행 파일 설정
add_executable(mani_moveit src/mani_moveit.cpp)
target_include_directories(mani_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mani_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  mani_moveit
  moveit_ros_planning_interface
  moveit_visual_tools
  custom_interfaces
  rclcpp
)

install(TARGETS mani_moveit
  DESTINATION lib/${PROJECT_NAME})

# 추가: activegrippertest 실행 파일 설정
add_executable(activegrippertest src/activegrippertest.cpp)
target_include_directories(activegrippertest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(activegrippertest PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  activegrippertest
  rclcpp
  std_srvs
)

install(TARGETS activegrippertest
  DESTINATION lib/${PROJECT_NAME})

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
