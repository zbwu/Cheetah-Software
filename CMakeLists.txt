cmake_minimum_required(VERSION 3.5)
project(cheetah)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)

SET (XTHIS_COM "${PROJECT_SOURCE_DIR}" )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
  ${CMAKE_BINARY_DIR}/Configuration.h)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Wshadow -Wsign-promo \
  -Wno-unused-variable -Wno-unused-parameter -Wno-format-truncation \
  -Wno-misleading-indentation")
elseif(APPLE)
  set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo")
endif()

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")

# include_directories("lcm-types/cpp")

# include_directories("third-party/lcm")   # lcm includes
include_directories("third-party/eigen")   # eigen3 includes
include_directories("third-party/yaml-cpp/include")
include_directories("third-party/qpOASES/include")
include_directories("third-party/Goldfarb_Optimizer")
# include_directories("third-party/osqp/include")
# include_directories("third-party/JCQP")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)

if(NOT NO_SIM)
  add_subdirectory(sim)
endif()

add_subdirectory(user)
add_subdirectory(rc_test)
