cmake_minimum_required(VERSION 3.20)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Include only your pure C++ core modules
add_executable(Physics_Engine
        main.cpp
        config.h
)

# 2. Tell the compiler where to find <raylib.h>
target_include_directories(Physics_Engine PRIVATE "C:/raylib/raylib/src")

# 3. Direct the linker to the static binary and native Windows subsystems
target_link_libraries(Physics_Engine PRIVATE
        "C:/raylib/raylib/src/libraylib.a"
        opengl32
        gdi32
        winmm
)