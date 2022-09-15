include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_SOURCE_DIR}/common/include")

file(GLOB sources "src/*.cpp")

set(rt_sources "src/rt/rt_spi.cpp")

add_library(robot SHARED ${sources} ${rt_sources})
target_link_libraries(robot biomimetics pthread lcm)

add_library(robot-static STATIC ${sources} ${rt_sources})
target_link_libraries(robot-static biomimetics-static pthread)
# target_link_libraries(robot-static lcm-static)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot rt)
target_link_libraries(robot-static rt)
endif()
