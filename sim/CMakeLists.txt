cmake_minimum_required(VERSION 3.5)
project(sim)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Gui REQUIRED)
find_package(Qt6 COMPONENTS Core REQUIRED)
find_package(Qt6 COMPONENTS Widgets REQUIRED) 
find_package(Qt6 COMPONENTS OpenGL REQUIRED)
find_package(Qt6 COMPONENTS OpenGLWidgets REQUIRED)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

set(QOBJECT_HEADERS
        include/Graphics3D.h
        include/GameController.h
        include/SimControlPanel.h
        include/JoystickTest.h)

QT6_WRAP_CPP(tst_hdr_moc ${QOBJECT_HEADERS})

include_directories("include/")
include_directories("../common/include/")
include_directories("../lcm-types/cpp")
include_directories("../third-party/ParamHandler")
include_directories(${CMAKE_BINARY_DIR})
file(GLOB sources "src/*.cpp")

add_executable(sim ${sources} ${tst_hdr_moc})
target_link_libraries(sim biomimetics pthread lcm dynacore_param_handler 
    Qt6::Gui Qt6::Core Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
    ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${SDL2_LIBRARIES})

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(sim rt ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)



