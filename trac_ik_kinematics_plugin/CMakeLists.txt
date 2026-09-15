cmake_minimum_required(VERSION 3.16)
project(trac_ik_kinematics_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(trac_ik_lib REQUIRED)

# Generate parameter library
generate_parameter_library(${PROJECT_NAME}_parameters src/trac_ik_kinematics_plugin_parameters.yaml)

# Build
add_library(${PROJECT_NAME} SHARED src/trac_ik_kinematics_plugin.cpp)
# BEFORE: an apt-installed ros-<distro>-trac-ik-kinematics-plugin (and ros-<distro>-trac-ik-lib)
# puts /opt/ros/<distro>/include on the include path ahead of this workspace's, so without BEFORE
# the plugin compiles against the installed package's headers instead of these.
target_include_directories(${PROJECT_NAME}
  BEFORE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# BEFORE above only orders this package's own headers. trac_ik_lib's arrive through its imported
# target's interface, which CMake emits after every dependency's -isystem entries -- including
# /opt/ros/<distro>/include, where an apt ros-<distro>-trac-ik-lib keeps a different copy of
# <trac_ik/trac_ik.hpp> that then wins. Repeating them in this target's own list puts them ahead of
# all of it. (Verified: without this the plugin compiled against /opt/ros/jazzy/include.)
get_target_property(_trac_ik_lib_includes trac_ik_lib::trac_ik INTERFACE_INCLUDE_DIRECTORIES)
if(_trac_ik_lib_includes)
  target_include_directories(${PROJECT_NAME} PRIVATE ${_trac_ik_lib_includes})
endif()
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_parameters
    class_loader::class_loader
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    rclcpp::rclcpp
    tf2_kdl::tf2_kdl
    trac_ik_lib::trac_ik
    urdf::urdf)

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "TRAC_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

# Pluginlib export
pluginlib_export_plugin_description_file(moveit_core
                                         trac_ik_kinematics_description.xml)

# Ament export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  class_loader
  generate_parameter_library
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  trac_ik_lib
  urdf)

# --- PROTOTYPE TEST HARNESS (wayfinder ticket 02); throwaway, not production wiring ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(tf2_eigen REQUIRED)
  ament_add_gtest(test_plugin test/test_plugin.cpp TIMEOUT 300)
  if(TARGET test_plugin)
    target_link_libraries(test_plugin
      ${PROJECT_NAME}
      moveit_core::moveit_robot_model
      moveit_core::moveit_robot_state
      srdfdom::srdfdom
      tf2_eigen::tf2_eigen)
    target_compile_definitions(test_plugin
      PRIVATE TRAC_IK_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()
endif()

# Package
ament_package()
