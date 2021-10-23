add_library(footbot_wall MODULE footbot_wall.h footbot_wall.cpp)
target_link_libraries(footbot_wall
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
