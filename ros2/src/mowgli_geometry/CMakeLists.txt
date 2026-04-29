cmake_minimum_required(VERSION 3.8)
project(mowgli_geometry)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Header-only INTERFACE library
# ---------------------------------------------------------------------------

add_library(mowgli_geometry INTERFACE)
target_include_directories(mowgli_geometry INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowgli_geometry INTERFACE Eigen3::Eigen)
target_compile_features(mowgli_geometry INTERFACE cxx_std_17)
ament_target_dependencies(mowgli_geometry INTERFACE geometry_msgs Boost)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(TARGETS mowgli_geometry
  EXPORT mowgli_geometry
  INCLUDES DESTINATION include)

ament_export_targets(mowgli_geometry HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs Eigen3 Boost)
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
