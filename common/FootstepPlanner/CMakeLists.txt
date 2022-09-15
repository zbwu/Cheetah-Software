file(GLOB_RECURSE sources "*.cpp")

add_library(footstep_planner SHARED ${sources})
target_link_libraries(footstep_planner biomimetics)