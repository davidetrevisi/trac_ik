cmake_minimum_required(VERSION 3.16)
project(trac_ik_lib)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(NLopt REQUIRED)
# This package includes <kdl/*.hpp> itself, so it finds KDL itself rather than inheriting it from
# kdl_parser: kdl_parser 3.x (rolling) dropped its orocos_kdl_vendor dependency.
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

# Build
add_library(trac_ik SHARED
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)
# BEFORE: an apt-installed ros-<distro>-trac-ik-lib puts /opt/ros/<distro>/include on the include
# path, and some dependency puts it ahead of this workspace's. Without BEFORE the build silently
# compiles against the installed package's headers instead of these.
target_include_directories(trac_ik
  BEFORE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(trac_ik PUBLIC cxx_std_17)
target_link_libraries(trac_ik
  PUBLIC
    ${geometry_msgs_TARGETS}
    Eigen3::Eigen
    kdl_parser::kdl_parser
    orocos-kdl
    rclcpp::rclcpp
    urdf::urdf
    # Not the imported target by name: on Ubuntu, find_package(NLopt) resolves to the nlopt_cxx
    # config, which defines NLopt::nlopt_cxx and not NLopt::nlopt (checked on noble aarch64,
    # nlopt 2.7.1). NLOPT_LIBRARIES is set by whichever config won and already names an imported
    # target, so it is the portable spelling, not the legacy one.
    ${NLOPT_LIBRARIES})

# Benchmark tool (wayfinder ticket 11): the ROS 2 successor of trac_ik_examples/src/ik_tests.cpp.
# Built unconditionally, so every CI job type-checks it, and installed so it can be run by hand with
#   ros2 run trac_ik_lib ik_benchmark
# It reads the fixtures ticket 02 keeps under test/fixtures, so it must be run where the sources live.
# -DIK_BENCHMARK_GIT_REV=... wins, for a tree synced without its .git (vm/sync.sh does exactly that).
if(NOT IK_BENCHMARK_GIT_REV)
  execute_process(
    COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE IK_BENCHMARK_GIT_REV
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()
if(NOT IK_BENCHMARK_GIT_REV)
  set(IK_BENCHMARK_GIT_REV "unknown")
endif()
add_executable(ik_benchmark benchmark/ik_benchmark.cpp)
target_link_libraries(ik_benchmark trac_ik kdl_parser::kdl_parser orocos-kdl urdf::urdf)
target_compile_definitions(ik_benchmark
  PRIVATE
    IK_BENCHMARK_GIT_REV="${IK_BENCHMARK_GIT_REV}"
    IK_BENCHMARK_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ik_benchmark
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS trac_ik
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Ament export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs urdf kdl_parser orocos_kdl_vendor orocos_kdl NLopt)

# Tests (wayfinder ticket 02): the library seam, TRAC_IK::CartToJnt, judged by KDL forward
# kinematics. Under BUILD_TESTING so the existing industrial_ci `colcon test` picks the target up
# with no workflow change; every CI cell runs it and gates on it. Fixtures are found through the
# TRAC_IK_TEST_FIXTURE_DIR compile definition rather than a hard-coded path or ament_index: one less
# install step, and --symlink-install keeps edits live.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trac_ik test/test_trac_ik.cpp TIMEOUT 300)
  if(TARGET test_trac_ik)
    target_link_libraries(test_trac_ik trac_ik kdl_parser::kdl_parser orocos-kdl urdf::urdf)
    target_compile_definitions(test_trac_ik
      PRIVATE TRAC_IK_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()
endif()

# Package
ament_package()
