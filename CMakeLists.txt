cmake_minimum_required(VERSION 3.8)
project(ti_es_cam_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(
  include
  ${OPENCV_INCLUDE_DIRS}
  /usr/include/opencv4
)


add_executable(ti_es_cam_node src/ti_es_cam_node.cpp src/camera.cpp)
target_include_directories(ti_es_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ti_es_cam_package>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ti_es_cam_node ${OPENCV_LIBRARIES} ASICamera2)
target_compile_features(ti_es_cam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ti_es_cam_node
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
