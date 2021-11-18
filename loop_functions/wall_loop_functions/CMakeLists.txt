include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(wall_loop_functions MODULE 
  wall_qt_user_functions.h
  wall_loop_functions.h
  wall_qt_user_functions.cpp
  wall_loop_functions.cpp)

target_link_libraries(wall_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
