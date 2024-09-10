#include "bf.h"
#include "sitl.h" // access to bf internals
#include <array>

namespace SimITL{
  #define USE_QUAT_ORIENTATION

  #ifndef M_PI
  #define M_PI 3.14159265358979
  #endif

  const static auto GYRO_SCALE = 16.4f;
  const static auto RAD2DEG = (180.0f / float(M_PI));
  const static auto ACC_SCALE = (256 / 9.80665f);

  namespace BF {
    
    void resetRcData(){
      //reset rc data to valid data...
      for(int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++){
        rcDataCache[i] = 1000U;
      }
    }

    void setRcData(float data[8]) {
      uint32_t timeUs = BF::micros_passed & 0xFFFFFFFF;

      std::array<uint16_t, 8> rcData;
      for (int i = 0; i < 8; i++) {
        rcDataCache[i] = uint16_t(1500 + data[i] * 500);
      }
      rcDataReceptionTimeUs = timeUs;
      //BF::rxMspFrameReceive(&rcData[0], 8);
      //hack to trick bf into using sim data...
      BF::rxRuntimeState.channelCount     = SIMULATOR_MAX_RC_CHANNELS; //SimITL target.h
      BF::rxRuntimeState.rcReadRawFn      = BF::rxRcReadData;
      BF::rxRuntimeState.rcFrameStatusFn  = BF::rxRcFrameStatus;
      BF::rxRuntimeState.rxProvider       = BF::RX_PROVIDER_UDP;
      BF::rxRuntimeState.rcFrameTimeUsFn  = BF::rxRcFrameTimeUs;
      BF::rxRuntimeState.lastRcFrameTimeUs = timeUs;
    }

    char * EEPROM_FILENAME = 0;

    void setEepromFileName(const char* filename){
      const size_t maxFileSize = 32;
      EEPROM_FILENAME = new char[maxFileSize];
      std::fill(EEPROM_FILENAME, EEPROM_FILENAME + maxFileSize, 0);
      EEPROM_FILENAME[31] = '\0';
      memcpy(EEPROM_FILENAME, filename, strnlen(filename, maxFileSize));
    }

    void updateBattery(const SimState& simState){
      BF::setCellCount(simState.initPacket.quadBatCellCount);
      // voltage
      BF::voltageMeter_t* vMeter = BF::getVoltageMeter();
      vMeter->unfiltered      = static_cast<uint16_t>(simState.batteryState.batVoltageSag * 1e2);
      vMeter->displayFiltered = static_cast<uint16_t>(simState.batteryState.batVoltageSag * 1e2);
      vMeter->sagFiltered     = static_cast<uint16_t>(simState.batteryState.batVoltage    * 1e2);
      // ampere
      BF::currentMeter_t* cMeter = BF::getCurrentMeter();
      cMeter->amperage        = static_cast<int32_t>(simState.batteryState.amperage * 1e2);
      cMeter->amperageLatest  = static_cast<int32_t>(simState.batteryState.amperage * 1e2);
      cMeter->mAhDrawn        = static_cast<int32_t>(simState.batteryState.mAhDrawn);
    }

    void updateGyroAcc(const SimState& simState){
      int16_t x, y, z;
      if (BF::sensors(BF::SENSOR_ACC)) {
  //#ifdef USE_QUAT_ORIENTATION
          BF::imuSetAttitudeQuat(
             simState.rotation[3],
            -simState.rotation[2],
            -simState.rotation[0],
             simState.rotation[1]
          );

  //#else
          x = int16_t(BF::constrain(int(-simState.acc[2] * ACC_SCALE), -32767, 32767));
          y = int16_t(BF::constrain(int( simState.acc[0] * ACC_SCALE), -32767, 32767));
          z = int16_t(BF::constrain(int( simState.acc[1] * ACC_SCALE), -32767, 32767));
          BF::virtualAccSet(BF::virtualAccDev, x, y, z);
  //#endif
      }

      x = int16_t(BF::constrain(int(-simState.gyro[2] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      y = int16_t(BF::constrain(int( simState.gyro[0] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      z = int16_t(BF::constrain(int(-simState.gyro[1] * GYRO_SCALE * RAD2DEG), -32767, 32767));
      BF::virtualGyroSet(BF::virtualGyroDev, x, y, z);
    }

    void updateGps(const SimState& simState){
      const auto DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS = 1.113195f;
      const auto cosLon0 = 0.63141842418f;

      // set gps:
      static int64_t last_millis = 0;
      int64_t millis = BF::micros_passed / 1000;

      //if (millis - last_millis > 100) {
      {
        vec3 pos;
        copy(pos, simState.statePacket.position);

        BF::EnableState(BF::GPS_FIX);
        BF::gpsSol.numSat = 10;
        BF::gpsSol.llh.lat =
          int32_t(
            pos[2] * 100 /
            DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS) +
          simState.initPacket.gps.lat;
        BF::gpsSol.llh.lon =
          int32_t(
            pos[0] * 100 /
            (cosLon0 *
            DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS)) +
          simState.initPacket.gps.lon;
        BF::gpsSol.llh.altCm = int32_t(pos[1] * 100) + simState.initPacket.gps.alt;
        vec3 linearVelocity;
        copy(linearVelocity, simState.statePacket.linearVelocity);
        BF::gpsSol.groundSpeed = uint16_t(length(linearVelocity) * 100);
        BF::GPS_update |= BF::GPS_MSP_UPDATE;

        last_millis = millis;
      }
    }

    void updateOsd(SimState& simState){
      bool osdChanged = false;
      for (int y = 0; y < VIDEO_LINES; y++) {
        for (int x = 0; x < CHARS_PER_LINE; x++) {
          //TODO: access to osdScreen has to be guarded
          if(simState.osdUpdatePacket.osd[y * CHARS_PER_LINE + x] != BF::osdScreen[y][x]){
            osdChanged = true;
          }
          simState.osdUpdatePacket.osd[y * CHARS_PER_LINE + x] = BF::osdScreen[y][x];
        }
      }
      // keep osdChanged true. is reset when the osdUpdate is sent out.
      simState.osdChanged = simState.osdChanged || osdChanged;
    }

    bool update(uint64_t dt, SimState& simState){
      bool schedulerExecuted = false;

      BF::micros_passed += dt;

      updateBattery(simState);
      updateGyroAcc(simState);
      updateGps(simState);

      if (BF::sleep_timer > 0) {
        BF::sleep_timer -= dt;
        BF::sleep_timer = std::max(int64_t(0), BF::sleep_timer);
      } else {
        BF::scheduler();
        schedulerExecuted = true;
      }

      updateOsd(simState);

      simState.armed = (BF::armingFlags & BF::ARMED) == BF::ARMED;
      simState.armingDisabledFlags = (int)BF::getArmingDisableFlags();
      simState.microsPassed = BF::micros_passed;
      simState.motorsState[0].pwm = BF::motorsPwm[0] / 1000.0f;
      simState.motorsState[1].pwm = BF::motorsPwm[1] / 1000.0f;
      simState.motorsState[2].pwm = BF::motorsPwm[2] / 1000.0f;
      simState.motorsState[3].pwm = BF::motorsPwm[3] / 1000.0f;

      simState.beep = BF::getBeeper();
      simState.stateUpdatePacket.beep =  simState.beep;

      return schedulerExecuted;
    }

    void setDebugValue(uint8_t mode, uint8_t index, int16_t value){
      BF_DEBUG_SET(mode, index, value);
    }
  } // namespace bf
} // namespace SimITL