cmake_minimum_required(VERSION 3.8)
project(bluerov2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  add_test(
    NAME test_gazebo_assets
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gazebo_assets.py
  )
  set_tests_properties(test_gazebo_assets PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
