# 声明要求的cmake 最低版本
cmake_minimum_required( VERSION 2.8)

# 设置 debug 模式
set( CMAKE_BUILD_TYPE "Debug")

# 声明一个cmake工程
project(coordinateTransform)

# 添加一个可执行程序
# 语法 add_executable( 程序名 源代码文件 )
add_executable( coordinateTransform coordinateTransform.cpp )

# 添加头文件
include_directories( "/usr/include/eigen3" )

