cmake_minimum_required(VERSION 3.20)
project(mad_icp_core VERSION 1.0.0 LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Define the static library target
add_library(mad_icp_core STATIC
  src/mad_tree.cpp
  src/mad_icp.cpp
  src/vel_estimator.cpp
  src/pipeline.cpp)

target_include_directories(mad_icp_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(mad_icp_core
  PUBLIC
    Eigen3::Eigen
    pcl_common
  PRIVATE
    OpenMP::OpenMP_CXX)

target_compile_features(mad_icp_core PUBLIC cxx_std_17)

target_compile_options(mad_icp_core PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install header files
install(DIRECTORY include/
  DESTINATION include)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
