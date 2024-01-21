#ifndef PHYSICS_H
#define PHYSICS_H

#include "state.h"
#include "util/SampleCurve.h"

namespace SimITL{
  class Physics{
    public:
      Physics() = default;
      ~Physics() = default;

      /**
       * \brief Check if SimState was set.
       * \return True if SimState was set, false otherwise.
       */
      bool checkSimState();

      /**
       * \brief Sets the sim-wide state, which is shared across all components.
       * \param[in,out] state The state of the sim that is updated and accessed per simulation step, 
       * from various components.
       */
      void setSimState(SimState* state);

      /**
       * \brief Sets the initally received state of the simulation.
       * \param[in] initPacket The inital state, received from the game client, to init the sim.
       */
      void initState(const InitPacket& initPacket);

      /**
       * \brief Process the state update, and stages it for update calls.
       * \param[in] statePacketUpdate The state update received from the game client.
       */
      void updateState(const StatePacket& statePacketUpdate);

      /**
       * \brief Updates the gyro input for bf. Includes noise from various sources.
       * Has to be executed before the bf scheduler is run to process new input!
       * \param[in] dt The delta time in seconds.
       */
      void updateGyro(double dt);

      /**
       * \brief Updates motor outputs, physics, battery state 
       * and prepares the update packet for the game client.
       * \param[in] dt The delta time in seconds.
       */
      void updatePhysics(double dt);

    private:

      void updateRotation(double dt, StatePacket& state);

      float motorCurrent(float volts, float kV);
      float motorTorque(float volts, float rpm, float kV, float R, float I0);
      float propThrust(float rpm, float vel);
      float propTorque(float rpm, float vel);

      void updateBat(double dt);
      float shiftedPhase(const double dt, float hz, float phase);
      mat3 motorNoise(const double dt, MotorState& motorState);
      void updateGyroNoise(const StatePacket& state, vec3& angularNoise);
      void updateMotorNoise(const double dt, const StatePacket& state, vec3& angularNoise);

      float calculateMotors(double dt,
                            StatePacket& state,
                            std::array<MotorState, 4>& motors);

      vec3 calculatePhysics(double dt,
                            StatePacket& state,
                            const std::array<MotorState, 4>& motors,
                            float motorsTorque);

      void updateCommands(int commands);
      void repair();
      void reset();

      SimState* mSimState = nullptr;

      SampleCurve mBatVoltageCurve{{
        {-0.06, 4.4  }, //allows overcharge
        {0.0,   4.2  }, 
        {0.01,  4.05 }, 
        {0.04,  3.97 }, 
        {0.30,  3.82 },
        {0.40,  3.7  },
        {1.0,   3.49 },
        {1.01,  3.4  },
        {1.03,  3.3  },
        {1.06,  3.0  },
        {1.08,  0.0  }
      }};

  };
}

#endif // PHYSICS_H