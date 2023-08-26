# 声明要求的cmake 最低版本
cmake_minimum_required( VERSION 2.8)

# 设置 debug 模式
set( CMAKE_BUILD_TYPE "Debug")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)


target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})

# # 为使用 sophus，需要使用find_package命令找到它
# find_package(Sophus REQUIRED)
# include_directories( ${Sophus_INCLUDE_DIRS})
# target_link_libraries(trajectoryError Sophus::Sophus)
