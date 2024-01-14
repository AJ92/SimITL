cmake_minimum_required(VERSION 3.3)
#Sim In The Loop
project(SimITL)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GEN_COVERAGE "Generate coverage profile" OFF)

# Linker options for betaflight and windows
if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/extern/betaflight/src/main/target/SITL/pg.ld")
endif()

if (WIN32)
    message("-- Win32 --")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj -g -O0")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
endif ()

if (GEN_COVERAGE)
  message("Building with coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif ()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

add_subdirectory(extern/fmt EXCLUDE_FROM_ALL)

# Get betaflight sources:
add_subdirectory(extern/)

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/util/SimplexNoise.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sim/sim.cpp
)

add_library(libsitl OBJECT ${BETAFLIGHT_SOURCES} ${SOURCE_FILES})
target_compile_features(libsitl PUBLIC cxx_std_17)

target_compile_definitions(libsitl PUBLIC "SITL")
target_compile_definitions(libsitl PUBLIC "__TARGET__=\"SimITL\"")
target_compile_definitions(libsitl PUBLIC "__REVISION__=\"1\"")

target_compile_definitions(libsitl PUBLIC SIMULATOR_BUILD)
target_compile_definitions(libsitl PUBLIC "HSE_VALUE=8000000")
target_compile_definitions(libsitl PUBLIC "FLASH_SIZE=2048")


target_include_directories(libsitl PUBLIC extern)
target_include_directories(libsitl PUBLIC extern/betaflightext/src/main)
target_include_directories(libsitl PUBLIC extern/betaflightext/src/main/target/simITL)
target_include_directories(libsitl PUBLIC extern/betaflight/lib/main/dyad)
target_include_directories(libsitl PUBLIC extern/betaflight/src/main)

target_include_directories(libsitl PUBLIC 
  extern/kissnet
  src
)

target_link_libraries(libsitl PUBLIC fmt-header-only)

# SimInTheLoop executable
add_executable(SimITL
  src/main.cpp $<TARGET_OBJECTS:libsitl>
)
target_compile_features(SimITL PUBLIC cxx_std_17)

target_link_libraries(SimITL PUBLIC 
  libsitl
)

if (UNIX)
  target_link_libraries(SimITL PUBLIC -lpthread)
endif (UNIX)


if (WIN32)
  target_link_libraries(SimITL PUBLIC wsock32 ws2_32)
endif (WIN32)

#add_subdirectory(test)
#add_subdirectory(gdscript)
