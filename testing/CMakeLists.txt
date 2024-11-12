
cmake_minimum_required(VERSION 3.14)
project(myclion C)
 
set(CMAKE_C_STANDARD 99)

# link_directories(../libs)
add_executable(prog main.c)

# add_library(lib ../target/debug/box_gen.lib)
target_link_libraries(prog ../libs/box_gen.dll)

