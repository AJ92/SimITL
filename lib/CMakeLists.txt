cmake_minimum_required(VERSION 3.3)

set(SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/sharedmem.cpp
)

# shared mem game client interface
add_library(
  simitlsharedmem
  SHARED
  ${SOURCE_FILES}
)

target_include_directories(
  simitlsharedmem PUBLIC 
  "./"
)

target_compile_features(simitlsharedmem PUBLIC cxx_std_17)

if (UNIX)
  target_link_libraries(simitlsharedmem PUBLIC -lrt)
endif (UNIX)


if (WIN32)
  target_link_libraries(simitlsharedmem PUBLIC user32 kernel32)
endif (WIN32)

