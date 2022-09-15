include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/Controllers")
include_directories("${CMAKE_SOURCE_DIR}/robot/include")
include_directories("${CMAKE_SOURCE_DIR}/common/include")
include_directories("${CMAKE_SOURCE_DIR}/common/FootstepPlanner")

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})

file(GLOB sources
    "FSM_States/ControlFSM.cpp"
    "FSM_States/FSM_State.cpp"
    "FSM_States/FSM_State_BalanceStand.cpp"
    "FSM_States/FSM_State_ImpedanceControl.cpp"
    "FSM_States/FSM_State_JointPD.cpp"
    "FSM_States/FSM_State_Locomotion.cpp"
    "FSM_States/FSM_State_Passive.cpp"
    "FSM_States/FSM_State_RecoveryStand.cpp"
    "FSM_States/FSM_State_StandUp.cpp"
    "FSM_States/SafetyChecker.cpp"
    "Controllers/BalanceController/*.cpp"
    "Controllers/convexMPC/*.cpp")



add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)

if(LOCO_BACKFLIP)
  file(GLOB backflip_sources "Controllers/BackFlip/*.cpp")
  list(APPEND sources
      "FSM_States/FSM_State_BackFlip.cpp"
      "FSM_States/FSM_State_FrontJump.cpp")
  list(APPEND sources ${backflip_sources})
endif(LOCO_BACKFLIP)

if(LOCO_VISION)
  add_subdirectory(Controllers/VisionMPC)
  list(APPEND sources
      "FSM_States/FSM_State_Vision.cpp")
endif(LOCO_VISION)

add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(mit_ctrl robot biomimetics)
target_link_libraries(mit_ctrl qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)
target_link_libraries(mit_ctrl VisionMPC)

add_executable(loco_ctrl ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(loco_ctrl robot-static biomimetics-static)
target_link_libraries(loco_ctrl WBC_Ctrl-static)
target_link_libraries(loco_ctrl qpOASES)
