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

  drivers/accgyro/accgyro_fake.c
  drivers/accgyro/gyro_sync.c
  drivers/barometer/barometer_fake.c
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
  drivers/max7456.c
  drivers/pwm_esc_detect.c
  drivers/rx/rx_pwm.c
  drivers/mco.c
  drivers/pinio.c
  drivers/resource.c
  drivers/sound_beeper.c
  drivers/bus_i2c_busdev.c
  drivers/bus_i2c_soft.c


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
  sensors/battery.c
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
  flight/feedforward.c
  flight/gps_rescue.c

  io/serial.c
  io/beeper.c
  io/statusindicator.c
  io/rcdevice_cam.c
  io/rcdevice.c
  io/gps.c
  io/displayport_max7456.c

  msp/msp.c
  msp/msp_box.c
  msp/msp_serial.c

  cli/cli.c
  cli/settings.c

  blackbox/blackbox.c
  blackbox/blackbox_encoding.c
)

list(TRANSFORM BETAFLIGHT_SOURCES PREPEND 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/src/main/")

set(BETAFLIGHT_SOURCES ${BETAFLIGHT_SOURCES} 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/dyad/dyad.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/lib/main/google/olc/olc.c"
)

file(GLOB BETAFLIGHT_G_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/betaflight/src/main/pg/*.c"

  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/common/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/io/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/scheduler/*.c"
)

set(BETAFLIGHT_SOURCES 
  ${BETAFLIGHT_SOURCES} ${BETAFLIGHT_G_SOURCES}
  PARENT_SCOPE
)
