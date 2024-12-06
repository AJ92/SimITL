#include "sim/sim.h"
#include "network/packets.h"
#include <fmt/format.h>
#include <stdlib.h>
#include <stdio.h>

SimITL::Sim* sim = nullptr;

// interface for c lib
extern "C" {
  void simitl_init(const StateInit& state){
    sim = &SimITL::Sim::getInstance();
    sim->init(state);
  }

  void simitl_reinit_physics(const StateInit& state){
    sim->reinitPhysics(state);
  }

  void simitl_update(const StateInput& state){
    sim->update(state);
  }

  StateOutput simitl_get_state(){
    return sim->getStateUpdate();
  }

  void simitl_command(const CommandType cmd){
    sim->command(cmd);
  }

  void simitl_stop(){
    sim->stop();
  }
}
