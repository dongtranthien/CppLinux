cmake_minimum_required(VERSION 3.8)

project(main LANGUAGES C CXX VERSION 1.0.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to find_package(Threads).
#
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
#
# If you've installed LibSerial in a non-standard location, please add it
# to PKG_CONFIG_PATH first. For example, if LibSerial is installed under
# /opt/libserial, set PKG_CONFIG_PATH environment variable to 
# /opt/libserial/lib/pkgconfig/ before running cmake for this example.
#
# export PKG_CONFIG_PATH=/opt/libserial/lib/pkgconfig/
#
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(main main.cpp)
target_include_directories(main PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(main PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})