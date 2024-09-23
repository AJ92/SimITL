// provides access to betaflight internals
// used by SimITL

#ifndef SITL_BF
#define SITL_BF

namespace SimITL{
  namespace BF {
    extern "C" {
      #include "common/maths.h"

      #include "fc/init.h"
      #include "fc/runtime_config.h"
      #include "fc/tasks.h"

      #include "flight/imu.h"

      #include "scheduler/scheduler.h"
      #include "sensors/sensors.h"

      #include "drivers/accgyro/accgyro_virtual.h"
      #include "drivers/pwm_output.h"
      #include "drivers/pwm_output_fake.h"
      #include "drivers/sound_beeper.h"

      #include "sensors/current.h"

      //added, not sure if needed
      #include "rx/rx.h"
      #include "rx/msp.h"

      #include "io/displayport_fake.h"
      #include "io/gps.h"

      #include "target.h"
      #include "sensors/battery_fake.h"

      #include "build/debug.h"

      #include "drivers/serial_ws.h"

      #undef ENABLE_STATE

      //custom macro with bf namespaces
      #define BF_DEBUG_SET(mode, index, value) do { if (BF::debugMode == (mode)) { BF::debug[(index)] = (value); } } while (0)

      void EnableState(stateFlags_t mask) {
        stateFlags |= mask;
      }

      // rc data
      uint16_t rcDataCache[16] {};
      uint32_t rcDataReceptionTimeUs = 0U;

      static float rxRcReadData(const BF::rxRuntimeState_t *rxRuntimeState, uint8_t channel)
      {
        UNUSED(rxRuntimeState);
        return rcDataCache[channel];
      }

      static uint32_t rxRcFrameTimeUs(void)
      {
        return rcDataReceptionTimeUs;
      }

      static uint8_t rxRcFrameStatus(BF::rxRuntimeState_t *rxRuntimeState)
      {
        UNUSED(rxRuntimeState);
        return BF::RX_FRAME_COMPLETE;
      }

      static void updateSerialWs(){
        BF::wsUpdate();
      }

      extern uint64_t micros_passed;
      extern int64_t sleep_timer;

      extern int16_t motorsPwm[MAX_SUPPORTED_MOTORS];

    } // end extern "C"
  }
}
#endif