cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)
include_directories(${PROJECT_SOURCE_DIR}/include/mujoco)
#编译安装，从cmake安装位置opt使用
set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
if (mujoco_FOUND)
    message(STATUS "Find mujoco:  ${mujoco_INCLUDE_DIRS}, ${mujoco_LIBRARIES}")
endif()

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

add_executable(basic ${SRC_LIST})
#从cmake安装位置opt使用
target_link_libraries(basic mujoco::mujoco GL GLU glfw)

