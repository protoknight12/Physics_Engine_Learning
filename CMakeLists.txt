cmake_minimum_required(VERSION 3.20)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RAYLIB_DIR "C:\\raylib\\raylib" CACHE PATH "Path to raylib source root")

set(PHYSICS_CORE_SOURCES
        collider/collider.cpp
        collider/collider.h
        collision/collision.cpp
        collision/collision.h
        simulation/simulation.cpp
        simulation/simulation.h
        vector2d/vector2d.cpp
        vector2d/vector2d.h
        config.h
)

add_executable(Physics_Engine
        main.cpp
        ${PHYSICS_CORE_SOURCES}
)

target_include_directories(Physics_Engine PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        "${RAYLIB_DIR}/src"
)

target_link_libraries(Physics_Engine PRIVATE
        "${RAYLIB_DIR}/src/libraylib.a"
        opengl32
        gdi32
        winmm
)

option(BUILD_TESTS "Build physics unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    add_executable(Physics_Engine_Tests
            tests/test_main.cpp
            tests/test_vector2d.cpp
            tests/test_collision.cpp
            ${PHYSICS_CORE_SOURCES}
    )

    target_include_directories(Physics_Engine_Tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    add_test(NAME Physics_Engine_Tests COMMAND Physics_Engine_Tests)
endif()
