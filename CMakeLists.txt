cmake_minimum_required(VERSION 2.8)

project(learnrl)

set(CMAKE_BUILD_TYPE "Debug")

set(learnrl_SRC)
add_subdirectory(src)

add_executable(learnrl ${learnrl_SRC})


