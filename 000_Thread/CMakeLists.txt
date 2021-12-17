cmake_minimum_required(VERSION 2.8.3)
project(myproject)

find_package (Threads REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main Threads::Threads)

# (optional) Install the executables.
install(TARGETS main
        DESTINATION bin)