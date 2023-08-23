# 声明要求的cmake 最低版本
cmake_minimum_required( VERSION 2.8)

# 设置 debug 模式
set( CMAKE_BUILD_TYPE "Debug")

# 声明一个cmake工程
project(HelloSLAM)

# 添加一个可执行程序
# 语法 add_executable( 程序名 源代码文件 )
add_executable( helloSLAM helloSlam.cpp )


# 编译库
#add_library( hello libHelloSLAM.cpp ) # 这是编译静态库.a
add_library( hello_shared SHARED libHelloSLAM.cpp ) # 这是编译共享库.so


add_executable( useHello useHello.cpp )# 添加一个可执行程序
target_link_libraries( useHello hello_shared )# 链接到刚才使用的库上

