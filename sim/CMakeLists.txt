cmake_minimum_required(VERSION 3.10.0)
project(sim)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)

find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Gui REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS OpenGL REQUIRED)

if (QT_VERSION_MAJOR EQUAL 6)
  find_package(Qt${QT_VERSION_MAJOR} COMPONENTS OpenGLWidgets REQUIRED)
endif ()

if (QT_VERSION_MAJOR EQUAL 5)
  find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Gamepad REQUIRED)
endif ()

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

include_directories("${CMAKE_SOURCE_DIR}/common/include/")
file(GLOB sources "src/*.cpp")

add_executable(sim ${sources})
target_include_directories(sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim biomimetics-static pthread
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::OpenGL
        OpenGL::GL OpenGL::GLU GLUT::GLUT
        ${SDL2_LIBRARIES})

# target_link_libraries(sim lcm-static)

if (QT_VERSION_MAJOR EQUAL 6)
  target_link_libraries(sim Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
endif ()

if (QT_VERSION_MAJOR EQUAL 5)
  target_link_libraries(sim Qt${QT_VERSION_MAJOR}::Gamepad)
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(sim rt)
endif ()



