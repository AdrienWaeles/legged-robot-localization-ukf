cmake_minimum_required(VERSION 3.16)
project(unitree_go2_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# Root of unitree_sdk2, pass it with -DUNITREE_SDK_PATH=<path> or set the environment variable
set(UNITREE_SDK_PATH "$ENV{UNITREE_SDK_PATH}" CACHE PATH "Root of the unitree_sdk2 checkout or install")
if(NOT UNITREE_SDK_PATH)
    message(FATAL_ERROR "UNITREE_SDK_PATH is not set. Use -DUNITREE_SDK_PATH=<path to unitree_sdk2>")
endif()

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/unitree_controller.cpp
    src/odometry_tracker.cpp
    src/ukf_filter.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UNITREE_SDK_PATH}/include
    /usr/local/include/ddscxx
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ${UNITREE_SDK_PATH}/lib
    /usr/local/lib
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    unitree_sdk2
    ddsc
    ddscxx
    cycloneddsidl
    pthread
    ${CMAKE_DL_LIBS}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O3)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
