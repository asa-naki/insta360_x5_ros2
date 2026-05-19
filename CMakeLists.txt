cmake_minimum_required(VERSION 3.16)
project(insta360_x5_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
)

# --- Service interfaces --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetCameraMode.srv
  srv/GetCameraMode.srv
  DEPENDENCIES
)

# --- Library -------------------------------------------------------------
add_library(${PROJECT_NAME}_core
  src/camera_mode.cpp
  src/gst_camera.cpp
  src/insta360_x5_node.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC PkgConfig::GST)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  diagnostic_updater
  diagnostic_msgs
  image_transport
  camera_info_manager
)

# Link generated interfaces (Jazzy way)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${cpp_typesupport_target})

# --- Executable ---------------------------------------------------------
add_executable(insta360_x5_node src/main.cpp)
target_link_libraries(insta360_x5_node PRIVATE ${PROJECT_NAME}_core)

# --- Install ------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS insta360_x5_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/smoke_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests --------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint heavy checks for a minimal first cut.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # xmllint requires network access to fetch schemas — skip in offline builds.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_mode test/test_camera_mode.cpp)
  target_link_libraries(test_camera_mode ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  diagnostic_updater
  image_transport
  camera_info_manager
  rosidl_default_runtime
)
ament_package()
