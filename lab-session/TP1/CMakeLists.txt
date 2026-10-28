cmake_minimum_required(VERSION 3.16)
project(unitree_go2_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Unitree SDK path (adjust according to your installation)
set(UNITREE_SDK_PATH "/home/student/unitree_ws/unitree_sdk2")

# Try to find Cyclone DDS
find_package(CycloneDX QUIET)
if(NOT CycloneDX_FOUND)
    # Manual paths for Cyclone DDS
    set(CycloneDX_INCLUDE_DIRS "/usr/local/include;/usr/include")
    set(CycloneDX_LIBRARIES "ddsc;ddscxx")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UNITREE_SDK_PATH}/include
    ${EIGEN3_INCLUDE_DIR}
    /usr/local/include
    /usr/local/include/ddscxx
    /usr/include
    ${CycloneDX_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${UNITREE_SDK_PATH}/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

# Source files
set(SOURCES
    src/main.cpp
    src/unitree_controller.cpp
    src/odometry_tracker.cpp
    src/ukf_filter.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries - Using Cyclone DDS instead of Fast-DDS
target_link_libraries(${PROJECT_NAME}
    unitree_sdk2
    ddsc
    ddscxx
    cycloneddsidl
    pthread
    ${CMAKE_DL_LIBS}
)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O3)

# Install target
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
