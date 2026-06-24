cmake_minimum_required(VERSION 4.0)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 20)

# 1. Point CMake to your exact Qt6 installation prefix path
set(CMAKE_PREFIX_PATH "C:/Qt/6.11.1/mingw_64")

# 2. Enable automatic Qt Meta-Object Compiler (MOC) and UI compilation
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 3. Locate the mandatory Qt6 Graphics Widgets package
find_package(Qt6 COMPONENTS Widgets REQUIRED)

# 4. Include all project source (.cpp) and header (.h) modules
add_executable(Physics_Engine
        main.cpp
        config.h
)

# 5. Link the Qt6 user interface targets to your compilation binary
target_include_directories(Physics_Engine PRIVATE "C:/Qt/6.11.1/mingw_64/include")
target_link_libraries(Physics_Engine PRIVATE Qt6::Widgets)