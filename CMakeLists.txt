cmake_minimum_required(VERSION 3.15)
project(Project2)

set(CMAKE_CXX_STANDARD 14)

add_executable(Project2 main.cpp State.cpp State.h Neighbors.cpp Neighbors.h Node.cpp Node.h)