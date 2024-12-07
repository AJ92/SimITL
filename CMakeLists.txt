cmake_minimum_required(VERSION 3.3)
project(SimITL)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GEN_COVERAGE "Generate coverage profile" OFF)

if (WIN32)
  message("-- Win32 --")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
endif ()

if (GEN_COVERAGE)
  message("Building with coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif ()

set(CMAKE_SIZEOF_VOID_P 8)  # Enforces 64-bit pointer size
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")  # -g -O0

# sitl lib 
#  used by the SimITL executable which contains the betaflight sitl target and extensions
add_subdirectory(extern/)

# SimITL executable
add_subdirectory(src/)

# SimITL shared memory lib for game clients to interface with a running SimITL process
add_subdirectory(lib/)

# SimITL-Tester
# enables testing of a SimITL process without a gameclient
add_subdirectory(tools/simitl-tester)


install(TARGETS simitl-tester SimITL sitl simitlsharedmem
        RUNTIME DESTINATION bin           COMPONENT Runtime
        LIBRARY DESTINATION lib           COMPONENT Runtime
        ARCHIVE DESTINATION lib/simitl    COMPONENT Development)