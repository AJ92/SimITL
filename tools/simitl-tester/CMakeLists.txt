cmake_minimum_required(VERSION 3.3)
# Test tool to instrument SimITL process without a game client

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

# SimInTheLoop executable
add_executable(simitl-tester
  ${SOURCE_FILES}
)

target_include_directories(simitl-tester PUBLIC 
  "../../src"
  "../../extern"
  "../../extern/kissnet"
)

target_compile_features(simitl-tester PUBLIC cxx_std_17)

if (UNIX)
  target_link_libraries(simitl-tester PUBLIC -lpthread)
endif (UNIX)


if (WIN32)
  target_link_libraries(simitl-tester PUBLIC wsock32 ws2_32)
endif (WIN32)

target_link_libraries(simitl-tester PUBLIC fmt-header-only)

