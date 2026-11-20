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
    src/PathPlanning2D.cpp
    src/RobotConfiguration2D.cpp
    src/Obstacles2D.cpp
    src/AsyncPlanner2D.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(robotics_core PUBLIC Threads::Threads)

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

add_executable(test_robot_configuration tests/test_robot_configuration.cpp)
target_link_libraries(test_robot_configuration PRIVATE robotics_core)
add_test(NAME RobotConfiguration2D COMMAND test_robot_configuration)

add_executable(test_joint_limits tests/test_joint_limits.cpp)
target_link_libraries(test_joint_limits PRIVATE robotics_core)
add_test(NAME JointLimits2D COMMAND test_joint_limits)

add_executable(test_path_planning tests/test_path_planning.cpp)
target_link_libraries(test_path_planning PRIVATE robotics_core)
add_test(NAME PathPlanning2D COMMAND test_path_planning)

add_executable(test_target_safety tests/test_target_safety.cpp)
target_link_libraries(test_target_safety PRIVATE robotics_core)
add_test(NAME TargetSafety2D COMMAND test_target_safety)

add_executable(test_target_recovery tests/test_target_recovery.cpp)
target_link_libraries(test_target_recovery PRIVATE robotics_core)
add_test(NAME TargetRecovery2D COMMAND test_target_recovery)

add_executable(test_two_joint_detour tests/test_two_joint_detour.cpp)
target_link_libraries(test_two_joint_detour PRIVATE robotics_core)
add_test(NAME TwoJointDetour2D COMMAND test_two_joint_detour)

add_executable(test_five_joint_recovery tests/test_five_joint_recovery.cpp)
target_link_libraries(test_five_joint_recovery PRIVATE robotics_core)
add_test(NAME FiveJointRecovery2D COMMAND test_five_joint_recovery)

# Position-pair transition matrix; validates planning AND executed frame motion.
add_executable(test_motion_scenarios tests/test_motion_scenarios.cpp)
target_link_libraries(test_motion_scenarios PRIVATE robotics_core)
add_test(NAME MotionScenarioMatrix COMMAND test_motion_scenarios)

# Release-grade transitions: six groups (2..7 joints), Cartesian start states
# and targets. The stress test executes 600 independently witnessed motions.
add_executable(test_release_motions tests/test_release_motions.cpp)
target_link_libraries(test_release_motions PRIVATE robotics_core)
add_test(NAME ReleaseMotionMatrix COMMAND test_release_motions)
add_test(NAME ReleaseMotionStress COMMAND test_release_motions --stress)
set_tests_properties(ReleaseMotionStress PROPERTIES LABELS "stress;release" TIMEOUT 600)

# Shared geometry and scene-aware collision/path/follower regressions.
add_executable(test_obstacles tests/test_obstacles.cpp)
target_link_libraries(test_obstacles PRIVATE robotics_core)
add_test(NAME Obstacles2D COMMAND test_obstacles)

add_executable(test_async_planner tests/test_async_planner.cpp)
target_link_libraries(test_async_planner PRIVATE robotics_core)
add_test(NAME AsyncPlanner2D COMMAND test_async_planner)

add_executable(test_smooth_trajectory tests/test_smooth_trajectory.cpp)
target_link_libraries(test_smooth_trajectory PRIVATE robotics_core)
add_test(NAME SmoothTrajectory2D COMMAND test_smooth_trajectory)

# Planned joint-space trajectories projected into world-space tip traces.
add_executable(test_trajectory_preview tests/test_trajectory_preview.cpp)
target_link_libraries(test_trajectory_preview PRIVATE robotics_core)
add_test(NAME TrajectoryPreview2D COMMAND test_trajectory_preview)

# Cartesian-distance guided path optimization and joint-limit barrier diagnosis.
add_executable(test_cartesian_optimization tests/test_cartesian_optimization.cpp)
target_link_libraries(test_cartesian_optimization PRIVATE robotics_core)
add_test(NAME CartesianTipOptimization COMMAND test_cartesian_optimization)

# Optimized end-effector distance around obstacles, with scene-motion safety.
add_executable(test_obstacle_tip_optimization tests/test_obstacle_tip_optimization.cpp)
target_link_libraries(test_obstacle_tip_optimization PRIVATE robotics_core)
add_test(NAME ObstacleTipOptimization COMMAND test_obstacle_tip_optimization)
