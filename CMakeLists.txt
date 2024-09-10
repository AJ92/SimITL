cmake_minimum_required(VERSION 3.3)
#Sim 
project(Sim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GEN_COVERAGE "Generate coverage profile" OFF)

# Linker options for betaflight and windows
if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/extern/betaflight/src/main/target/SITL/pg.ld")
endif()

if (WIN32)
    message("-- Win32 --")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj -g -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj -O2")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
endif ()

if (GEN_COVERAGE)
  message("Building with coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif ()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# simitl library
add_subdirectory(extern/)

# SimITL executable
add_subdirectory(src/)

# SimITL-Trigger
add_subdirectory(tools/simitl-tester)