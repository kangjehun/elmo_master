cmake_minimum_required(VERSION 3.8)
project(elmo_master)

# Specify the C++ standard as 17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# - 3rd party libraries
find_package(yaml-cpp REQUIRED)

# Add the library
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)

# Add the executable
ament_auto_add_executable(${PROJECT_NAME}_exe
  src/${PROJECT_NAME}_node_main.cpp)
target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}_lib)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)