FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (WBC_Ctrl-static STATIC ${headers} ${sources})
target_link_libraries (WBC_Ctrl-static WBIC-static biomimetics-static lcm-static)

add_library (WBC_Ctrl SHARED ${headers} ${sources})
target_link_libraries (WBC_Ctrl biomimetics lcm)
target_link_libraries (WBC_Ctrl WBIC)

