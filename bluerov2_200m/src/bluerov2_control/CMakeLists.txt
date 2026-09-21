cmake_minimum_required(VERSION 3.8)
project(bluerov2_control)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(PROGRAMS
  scripts/environment_markers.py
  scripts/fish_controller.py
  scripts/keyboard_disturbance.py
  scripts/keyboard_wrench_teleop.py
  scripts/navigation_goal_panel.py
  scripts/simple_dynamics_sim.py
  scripts/simple_localizer.py
  scripts/simple_sonar_sim.py
  scripts/simple_sensor_suite.py
  scripts/waypoint_autopilot.py
  scripts/virtual_thruster_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  add_test(
    NAME test_control_core
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_core.py
  )
  set_tests_properties(test_control_core PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
