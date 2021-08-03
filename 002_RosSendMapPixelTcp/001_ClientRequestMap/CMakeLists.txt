cmake_minimum_required(VERSION 2.8.3)
project(myproject)

add_executable(main main.cpp)
target_link_libraries(main)


# (optional) Install the executables.
install(TARGETS main
        DESTINATION bin)