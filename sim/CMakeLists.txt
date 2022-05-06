cmake_minimum_required(VERSION 3.10.0)
project(sim)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

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

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

set(moc_headers
        include/moc_Graphics3D.cpp
        include/moc_GameController.cpp
        include/moc_SimControlPanel.cpp
        include/moc_JoystickTest.cpp)

include_directories("include/")
include_directories("../common/include/")
include_directories("../lcm-types/cpp")
include_directories("../third-party/yaml-cpp/include")
include_directories(${CMAKE_BINARY_DIR})
file(GLOB sources "src/*.cpp")

add_executable(sim ${sources} ${moc_headers})
target_link_libraries(sim biomimetics-static lcm-static
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::OpenGL
        OpenGL::GL OpenGL::GLU GLUT::GLUT
        ${SDL2_LIBRARIES})

if (QT_VERSION_MAJOR EQUAL 6)
  target_link_libraries(sim Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(sim rt)
endif ()



