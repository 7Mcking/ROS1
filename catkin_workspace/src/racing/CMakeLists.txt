# For usage of the mdoern cmake features a high version of CMake is requierd.
cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

# Prefent this CMakeLists to be read multiple times in a single project.
include_guard(GLOBAL)

# Define library meta information her. Since CMake 3.0.0 a version and language deinfintion is required.
project(racing VERSION 0.1 LANGUAGES CXX)

# Set Cxx11 the standard of this project. This makes a compiler that supports Cxx11 necesarry.
set(CMAKE_CXX_STANDARD 11)
# Enforce Cxx11 to be used by the compiler.
set(CMAKE_CXX_STANDARD_REQUIRED  11)

# Find catkin and all required components. Roscpp is the implementation of ROS und Cxx. The Sensor-Msgs are used to
# send the laserscanner data. The Geometry-Msgs are used to send the target vehicle state.
find_package(catkin REQUIRED REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs)

# Create the catkin package that depends on all requirements mentions above.
catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs)

# Adding library that doesn't make use of Ros.
add_library(racing_library
    src/RacingController.cpp)

# Add executable that uses ROS to receive and send data calculated by the library.
add_executable(racing_controller_node src/racing_controller_node.cpp)

# Define headers for this library. PUBLIC headers are used by compiling by the exteral developers, and will be added to
# consumers' build paths.
target_include_directories(racing_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Set the install directories to the correct locations (provided by catkin).
install(TARGETS racing_library racing_controller_node EXPORT RacingLibraryConfig
    ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Linking the libraray to the ROS node.
target_link_libraries(racing_controller_node racing_library)

# Link the node to the ROS packages used to encode send and receive signals.
add_dependencies(racing_controller_node ${catkin_EXPORTED_TARGETS})
target_include_directories(racing_controller_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(racing_controller_node ${catkin_LIBRARIES})
