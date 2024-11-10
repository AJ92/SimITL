cmake_minimum_required(VERSION 3.3)

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/util/SimplexNoise.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sim/bf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sim/physics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sim/sim.cpp
)

# SimInTheLoop executable
add_executable(SimITL
  main.cpp
  ${SOURCE_FILES}
)

target_include_directories(SimITL PUBLIC 
  "./"
  "../extern/kissnet"
  "../extern"
  "../extern/betaflightext/src/main"
  "../extern/betaflightext/src/main/target/simITL"
  "../extern/betaflight/lib/main/dyad"
  "../extern/betaflight/src/main"
  "../lib"
)

target_compile_features(SimITL PUBLIC cxx_std_17)

target_link_libraries(SimITL PUBLIC 
  sitl              # bf sitl
  simitlsharedmem   # shared mem for game client communication
)

if (UNIX)
  target_link_libraries(SimITL PUBLIC -lpthread)
endif (UNIX)


if (WIN32)
  target_link_libraries(SimITL PUBLIC wsock32 ws2_32)
endif (WIN32)

