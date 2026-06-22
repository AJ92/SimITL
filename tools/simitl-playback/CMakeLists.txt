cmake_minimum_required(VERSION 3.3)
# Test tool to instrument SimITL process without a game client

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

# SimInTheLoop executable
add_executable(simitl-playback
  ${SOURCE_FILES}
)

target_include_directories(simitl-playback PUBLIC 
  "../../src"
  "../../extern"
  "../../lib"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Set RPATH so the executable finds shared libraries (DLLs/SOs) in its own directory
set_target_properties(simitl-playback PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)

target_compile_features(simitl-playback PUBLIC cxx_std_17)

if (UNIX)
  target_link_libraries(simitl-playback PUBLIC -lpthread -lrt)
endif (UNIX)


if (WIN32)
  target_link_libraries(simitl-playback PUBLIC user32 kernel32)
endif (WIN32)

target_link_libraries(simitl-playback PUBLIC 
  fmt-header-only 
  SimITL
  sitl
)
