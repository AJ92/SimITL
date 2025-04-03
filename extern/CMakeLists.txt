#dependencies

add_subdirectory(fmt EXCLUDE_FROM_ALL)

set(LWS_WITH_MINIMAL_EXAMPLES OFF CACHE BOOL "disable minimal examples" FORCE)
set(LWS_WITHOUT_TESTAPPS ON CACHE BOOL "Don't build the libwebsocket-test-apps" FORCE)
set(LWS_WITHOUT_TEST_SERVER ON CACHE BOOL "Don't build the test server" FORCE)
set(LWS_WITHOUT_TEST_SERVER_EXTPOLL ON CACHE BOOL "Don't build the test server version that uses external poll" FORCE)
set(LWS_WITHOUT_TEST_PING ON CACHE BOOL "Don't build the ping test application" FORCE)
set(LWS_WITHOUT_TEST_CLIENT ON CACHE BOOL "Don't build the client test application" FORCE)
set(LWS_WITH_SSL OFF CACHE BOOL "disable ssl" FORCE)
set(LWS_WITH_SHARED OFF CACHE BOOL "Build the shared version of the library" FORCE)
add_subdirectory(libwebsockets)

# Betaflight sources for cmake

set(BETAFLIGHT_SOURCES
  build/build_config.c
  build/debug.c
  build/debug_pin.c
  build/version.c

  #target/SITL/udplink.c
  #target/SITL/target.c  #replaced extern/src/target

  #main.c  #replaced by main.cpp in parent folder
  common/maths.c
  common/streambuf.c
  common/crc.c
  common/filter.c
  common/sensor_alignment.c
  common/bitarray.c
  common/printf.c
  common/printf_serial.c
  common/time.c
  common/typeconversion.c
  common/encoding.c
  #common/string_light.c
  common/vector.c
  common/gps_conversion.c

  #pg/pg.c
  #pg/motor.c
  #pg/scheduler.c
  #pg/rx.c
  #pg/gyrodev.c
  #pg/board.c
  #pg/msp.c
  #pg/stats.c
  #pg/rcdevice.c
  #pg/gps.c
  #pg/gps_rescue.c
  #pg/vcd.c

  config/config.c
  config/feature.c
  config/config_streamer.c
  config/config_eeprom.c
  config/simplified_tuning.c

  drivers/accgyro/accgyro_virtual.c
  drivers/accgyro/gyro_sync.c
  drivers/barometer/barometer_virtual.c
  drivers/timer_common.c
  drivers/motor.c
  drivers/bus.c
  drivers/dshot.c
  drivers/serial.c
  #drivers/serial_tcp.c
  drivers/buf_writer.c
  drivers/stack_check.c
  drivers/io.c
  drivers/display.c
  drivers/light_led.c
  #drivers/max7456.c
  #drivers/pwm_esc_detect.c
  drivers/rx/rx_pwm.c
  drivers/mco.c
  drivers/pinio.c
  drivers/resource.c
  #drivers/sound_beeper.c
  drivers/bus_i2c_busdev.c
  drivers/bus_i2c_soft.c
  drivers/vtx_common.c
  drivers/vtx_table.c
  drivers/pwm_output.c

  fc/init.c

  #scheduler/scheduler.c

  rx/rx.c
  rx/msp.c
  rx/msp_override.c

  sensors/initialisation.c
  sensors/compass.c
  sensors/acceleration.c
  sensors/acceleration_init.c
  sensors/gyro.c
  sensors/gyro_init.c
  sensors/boardalignment.c
  #sensors/battery.c
  sensors/barometer.c
  sensors/voltage.c
  sensors/current.c
  sensors/adcinternal.c
  sensors/esc_sensor.c

  fc/core.c
  fc/runtime_config.c
  fc/rc_controls.c
  fc/tasks.c
  fc/rc_modes.c
  fc/controlrate_profile.c
  fc/rc.c
  fc/rc_adjustments.c
  fc/board_info.c
  fc/stats.c
  fc/dispatch.c

  flight/failsafe.c
  flight/mixer.c
  flight/mixer_init.c
  flight/mixer_tricopter.c
  flight/servos.c
  flight/servos_tricopter.c
  flight/imu.c
  flight/pid.c
  flight/pid_init.c
  flight/position.c
  #flight/feedforward.c
  flight/gps_rescue_multirotor.c
  flight/autopilot_multirotor.c

  #io/serial.c
  io/beeper.c
  io/statusindicator.c
  io/rcdevice_cam.c
  io/rcdevice.c
  io/gps.c
  #io/displayport_max7456.c
  io/displayport_msp.c
  io/serial_resource.c
  io/gps_virtual.c

  cms/cms.c
  cms/cms_menu_blackbox.c
  cms/cms_menu_failsafe.c
  cms/cms_menu_firmware.c
  cms/cms_menu_gps_rescue_multirotor.c
  cms/cms_menu_imu.c
  cms/cms_menu_ledstrip.c
  cms/cms_menu_main.c
  cms/cms_menu_misc.c
  cms/cms_menu_osd.c
  cms/cms_menu_persistent_stats.c
  cms/cms_menu_power.c
  cms/cms_menu_saveexit.c
  cms/cms_menu_vtx_common.c
  cms/cms_menu_vtx_msp.c
  cms/cms_menu_vtx_rtc6705.c
  cms/cms_menu_vtx_smartaudio.c
  cms/cms_menu_vtx_tramp.c

  osd/osd.c
  osd/osd_elements.c
  osd/osd_warnings.c

  msp/msp.c
  msp/msp_box.c
  msp/msp_serial.c
  msp/msp_build_info.c

  cli/cli.c
  #cli/settings.c

  blackbox/blackbox.c
  blackbox/blackbox_encoding.c
)

list(TRANSFORM BETAFLIGHT_SOURCES PREPEND 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/src/main/")

set(BETAFLIGHT_SOURCES ${BETAFLIGHT_SOURCES} 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/dyad/dyad.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/google/olc/olc.c"
)

file(GLOB BETAFLIGHT_PG_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/src/main/pg/*.c"
)

file(GLOB BETAFLIGHT_EXTENSIONS_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/target/simITL/*.c"
  #"${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/drivers/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/cli/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/common/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/io/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/scheduler/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/sensors/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/blackbox/*.c"

  "${CMAKE_CURRENT_SOURCE_DIR}/sitl.h"
)

set(BETAFLIGHT_EXTENSIONS_SOURCES ${BETAFLIGHT_EXTENSIONS_SOURCES} 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/drivers/pwm_output_fake.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/drivers/serial_ws.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflightext/src/main/drivers/sound_beeper.c"
)


set(BETAFLIGHT_SOURCES 
  ${BETAFLIGHT_SOURCES} ${BETAFLIGHT_PG_SOURCES} ${BETAFLIGHT_EXTENSIONS_SOURCES}
  #PARENT_SCOPE
)

add_library(
  sitl 
  #OBJECT
  STATIC
  #SHARED won't work as pg code has to be part of the binary and is missing if shared
  ${BETAFLIGHT_SOURCES} 
)

target_compile_features(sitl PUBLIC cxx_std_17)

target_compile_definitions(sitl PUBLIC "SITL")
target_compile_definitions(sitl PUBLIC "__TARGET__=\"SimITL\"")
target_compile_definitions(sitl PUBLIC "__REVISION__=\"1\"")

target_compile_definitions(sitl PUBLIC SIMULATOR_BUILD=1)
target_compile_definitions(sitl PUBLIC "HSE_VALUE=8000000")
target_compile_definitions(sitl PUBLIC "FLASH_SIZE=2048")


#target_include_directories(sitl PUBLIC extern)
target_include_directories(sitl PUBLIC betaflightext/src/main)
target_include_directories(sitl PUBLIC betaflightext/src/main/target/simITL)
target_include_directories(sitl PUBLIC betaflight/lib/main/dyad)
target_include_directories(sitl PUBLIC betaflight/src/main)
target_include_directories(sitl PUBLIC libwebsockets/include)

set_property(TARGET sitl PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET websockets PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(sitl PUBLIC fmt-header-only)
target_link_libraries(sitl PUBLIC websockets)
