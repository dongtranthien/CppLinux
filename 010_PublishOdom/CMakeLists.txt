cmake_minimum_required(VERSION 2.8.3)
project(myproject)

# Build the talker and listener. Each one uses the following ROS packages,
# which we need to find_package() individually:
#   roscpp (the client library)
#   std_msgs (contains the std_msgs/String message type)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf REQUIRED)
find_package (Threads REQUIRED)
# We've found them; now use their variables in the usual way to configure
# the compile and link steps.
# Note: we skip calling link_directories() because ROS packages follow the
# recommended CMake practice of returning absolute paths to libraries
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${tf_LIBRARIES})
target_link_libraries(main Threads::Threads)

# (optional) Install the executables.
install(TARGETS main
        DESTINATION bin)
