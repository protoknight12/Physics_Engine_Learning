cmake_minimum_required(VERSION 4.0)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 20)

add_executable(Physics_Engine main.cpp
        vector2d/vector2d.h
        vector2d/vector2d.cpp
        config.h
)
