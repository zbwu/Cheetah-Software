include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

include_directories("./")
include_directories("include/")
include_directories("../common/include/")
include_directories("../third-party/yaml-cpp/include")
include_directories("../lcm-types/cpp")

file(GLOB sources "src/*.cpp")

set(rt_sources "src/rt/rt_spi.cpp")

add_library(robot SHARED ${sources} ${rt_sources})
target_link_libraries(robot biomimetics pthread lcm)

add_library(robot-static STATIC ${sources} ${rt_sources})
target_link_libraries(robot-static biomimetics-static lcm-static)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot rt)
target_link_libraries(robot-static rt)
endif()
