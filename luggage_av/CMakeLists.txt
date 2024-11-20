cmake_minimum_required(VERSION 3.8)
project(luggage_av)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# DEPENDENCIES
set(HARDWARE_INTERFACE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Protobuf
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${HARDWARE_INTERFACE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# COMPILE
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/hardware_interface/proto/wheel_commands.proto ${PROJECT_SOURCE_DIR}/hardware_interface/proto/wheel_states.proto)
message(NOTICE ${PROTO_SRCS})
message(NOTICE ${PROTO_HDRS})
add_library(
  luggage_av
  SHARED
  hardware_interface/src/luggage_av_hardware_interface.cpp
  hardware_interface/src/cobs.c
  ${PROTO_SRCS}
)

target_compile_features(luggage_av PUBLIC cxx_std_17)

target_include_directories(luggage_av PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware_interface/include>
  $<INSTALL_INTERFACE:include/luggage_av>
  ${Protobuf_INCLUDE_DIRS}
)

ament_target_dependencies(
  luggage_av PUBLIC
  ${HARDWARE_INTERFACE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface luggage_av.xml)


# INSTALL
install(
  DIRECTORY hardware_interface/include
  DESTINATION include/luggage_av_hardware_interface
)

install(
  DIRECTORY launch parameters rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS luggage_av
  EXPORT export_luggage_av
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(hardware_interface)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# EXPORT
ament_export_targets(export_luggage_av HAS_LIBRARY_TARGET)
ament_export_dependencies(${HARDWARE_INTERFACE_INCLUDE_DEPENDS})
ament_package()
