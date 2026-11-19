cmake_minimum_required(VERSION 3.20)

project(RobotArmSimulator
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robotics_core STATIC
    src/ForwardKinematics.cpp
    src/Simulation2D.cpp
    src/Jacobian2D.cpp
    src/InverseKinematics2D.cpp
    src/SmoothMotion2D.cpp
)

target_include_directories(robotics_core PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)

add_executable(robot_cli apps/robot_cli.cpp)
target_link_libraries(robot_cli PRIVATE robotics_core)

# Keep the CLI and tests independent from graphics libraries.
option(ROBOTARM_BUILD_GUI "Build the interactive raylib 2D GUI" OFF)
if(ROBOTARM_BUILD_GUI)
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 6.2
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(raylib)

    add_executable(robot_gui apps/robot_gui.cpp)
    target_link_libraries(robot_gui PRIVATE robotics_core raylib)
endif()

enable_testing()

add_executable(test_vec2 tests/test_vec2.cpp)
target_link_libraries(test_vec2 PRIVATE robotics_core)
add_test(NAME Vec2 COMMAND test_vec2)

add_executable(test_forward_kinematics tests/test_forward_kinematics.cpp)
target_link_libraries(test_forward_kinematics PRIVATE robotics_core)
add_test(NAME ForwardKinematics COMMAND test_forward_kinematics)

add_executable(test_simulation tests/test_simulation.cpp)
target_link_libraries(test_simulation PRIVATE robotics_core)
add_test(NAME Simulation2D COMMAND test_simulation)

add_executable(test_jacobian tests/test_jacobian.cpp)
target_link_libraries(test_jacobian PRIVATE robotics_core)
add_test(NAME Jacobian2D COMMAND test_jacobian)

add_executable(test_inverse_kinematics tests/test_inverse_kinematics.cpp)
target_link_libraries(test_inverse_kinematics PRIVATE robotics_core)
add_test(NAME InverseKinematics2D COMMAND test_inverse_kinematics)

add_executable(test_smooth_motion tests/test_smooth_motion.cpp)
target_link_libraries(test_smooth_motion PRIVATE robotics_core)
add_test(NAME SmoothMotion2D COMMAND test_smooth_motion)
