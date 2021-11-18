include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

include_directories("./")
include_directories("include/")
include_directories("../common/include/")
include_directories("../third-party/ParamHandler")
include_directories("../lcm-types/cpp")

file(GLOB sources "src/*.cpp")

set(rt_sources "src/rt/rt_rc_interface.cpp" "src/rt/rt_sbus.cpp" "src/rt/rt_spi.cpp")

add_library(robot SHARED ${sources} ${rt_sources})

target_link_libraries(robot biomimetics pthread lcm dynacore_param_handler)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot libvnc rt)
endif()
