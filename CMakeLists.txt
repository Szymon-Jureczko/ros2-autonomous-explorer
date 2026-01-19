cmake_minimum_required(VERSION 3.8)
project(autonomous_explorer)

find_package(ament_cmake REQUIRED)

# Install launch files and world
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
