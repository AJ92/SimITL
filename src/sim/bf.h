/**
 * Abstraction of betaflight functions.
 */

#include <cstdint>
#include "state.h"

#ifndef BF_H
#define BF_H

namespace SimITL{
  namespace BF{
    extern "C" {
      // betaflight's init
      extern void init(void);
      extern void scheduler(void);
    }

    /**
     * \brief Resets rc channels to default values.
     * Prevents initial random channel states.
     */
    void resetRcData();

    /**
     * \brief Resets rc channels to default values.
     * Prevents initial random channel states.
     */
    void setRcData(float data[8]);

    /**
     * \brief Sets an eeprom name, where the virtual fc writes to and reads from.
     * (Path is set by the execution dir)
     * \param[in] filename The name of the eeprom file, the fc writes to and reads from.
     */
    void setEepromFileName(const char* filename = "eeprom.bin");

    /**
     * \brief Updates the virtual fc's data with the provided state, 
     * performs a betaflight schedule or sleeps and updates the state.
     * \param[in] delta Time delta in micro seconds.
     * \param[in] simState The state that is writen to the fc.
     * \return True if scheduler was executed, false if slept.
     */
    bool update(uint64_t dt, SimState& simState);

    /**
     * \brief BF debug call. Writes to blackbox.
     * \param[in] mode  The mode.
     * \param[in] index The index.
     * \param[in] value The value.
     */
    template <class A, class B, class C>
    void setDebugValue(A mode, B index, C value);
  }
}

#endif // BF_H