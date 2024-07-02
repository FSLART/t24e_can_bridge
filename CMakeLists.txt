cmake_minimum_required(VERSION 3.8)
project(t24e_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bridge src/bridge.cpp)
target_include_directories(bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS bridge
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(bridge 
  "rclcpp"
  "std_msgs"
  "lart_msgs"
  "std_msgs"
)

ament_package()
