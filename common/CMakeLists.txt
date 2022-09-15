# Source/Include files
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/FootstepPlanner")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# common library cpp files
file(GLOB sources
    "src/Collision/*.cpp"
    "src/ControlParameters/*.cpp"
    "src/Controllers/*.cpp"
    "src/Dynamics/*.cpp"
    "src/SimUtilities/*.cpp"
    "src/Utilities/*.cpp")

if(LOCO_SPARSE_MPC)
    file(GLOB sparse_mpc_sources
        "src/SparseCMPC/*.cpp")
    list(APPEND sources ${sparse_mpc_sources})
endif(LOCO_SPARSE_MPC)

# Library
add_library(biomimetics SHARED ${sources})       # produce a library used by sim/robot
target_link_libraries(biomimetics yaml-cpp JCQP osqp)

add_library(biomimetics-static STATIC ${sources})
target_link_libraries(biomimetics-static yaml-cpp)

if(COMMON_TEST)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
# Pull in Google Test
include(CTest)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()
include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
                 QUIET
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

# Test
file(GLOB_RECURSE test_sources "test/test_*.cpp")             # test cpp files
add_executable(test-common ${test_sources})
target_link_libraries(test-common gtest gmock_main lcm rt osqp pthread biomimetics)
target_link_libraries(test-common Goldfarb_Optimizer)
target_link_libraries(test-common JCQP)


add_test(NAME example_test COMMAND test-common)

endif(CMAKE_SYSTEM_NAME MATCHES Linux)
endif(COMMON_TEST)

# Our libraries
add_subdirectory(FootstepPlanner)

