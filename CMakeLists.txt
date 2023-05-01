cmake_minimum_required(VERSION 3.3)
project(SimFlight)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GEN_COVERAGE "Generate coverage profile" OFF)

# Linker options for betaflight and windows
if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/extern/betaflight/src/main/target/SITL/pg.ld")
endif()

if (WIN32)
    message("-- Win32 --")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")  # -g -O0
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
endif ()

if (GEN_COVERAGE)
  message("Building with coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif ()

add_subdirectory(extern/fmt EXCLUDE_FROM_ALL)

# Get betaflight sources:
add_subdirectory(extern/)

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sim.cpp
)

add_library(libsim OBJECT ${BETAFLIGHT_SOURCES} ${SOURCE_FILES})
target_compile_features(libsim PUBLIC cxx_std_17)

target_compile_definitions(libsim PUBLIC "SITL")
target_compile_definitions(libsim PUBLIC "__TARGET__=\"SITL\"")
target_compile_definitions(libsim PUBLIC "__REVISION__=\"0.0.1\"")

target_compile_definitions(libsim PUBLIC SIMULATOR_BUILD)
target_compile_definitions(libsim PUBLIC "HSE_VALUE=8000000")
target_compile_definitions(libsim PUBLIC "FLASH_SIZE=2048")


target_include_directories(libsim PUBLIC extern)
target_include_directories(libsim PUBLIC extern/src/)
target_include_directories(libsim PUBLIC extern/betaflight/lib/main/dyad)
target_include_directories(libsim PUBLIC extern/betaflight/src/main)

target_include_directories(libsim PUBLIC extern/kissnet)

target_link_libraries(libsim PUBLIC fmt-header-only)

add_executable(SimFlight
  src/main.cpp $<TARGET_OBJECTS:libsim>
)
target_compile_features(SimFlight PUBLIC cxx_std_17)

target_link_libraries(SimFlight PUBLIC 
  libsim
)

if (UNIX)
  target_link_libraries(SimFlight PUBLIC -lpthread)
endif (UNIX)


if (WIN32)
  target_link_libraries(SimFlight PUBLIC wsock32 ws2_32)
endif (WIN32)

#add_subdirectory(test)
#add_subdirectory(gdscript)
