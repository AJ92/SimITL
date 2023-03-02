# Betaflight sources for cmake


set(BETAFLIGHT_SOURCES
  build/build_config.c
  build/debug.c
  build/debug_pin.c
  build/version.c

  target/SITL/udplink.c
  #target/SITL/target.c  #replaced extern/src/target

  #main.c  #replaced by main.cpp in parent folder

  drivers/accgyro/accgyro_fake.c
  flight/imu.c

  config/feature.c
  config/config_streamer.c

  drivers/motor.c

  fc/init.c

  scheduler/scheduler.c

  common/maths.c
)

list(TRANSFORM BETAFLIGHT_SOURCES PREPEND 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/src/main/")

set(BETAFLIGHT_SOURCES ${BETAFLIGHT_SOURCES} 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/dyad/dyad.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/google/olc/olc.c"
)

file(GLOB BETAFLIGHT_G_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

set(BETAFLIGHT_SOURCES 
  ${BETAFLIGHT_SOURCES} ${BETAFLIGHT_G_SOURCES}
  PARENT_SCOPE
)
