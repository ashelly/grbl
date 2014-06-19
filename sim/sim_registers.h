/*
  config.h - replacement for the include of the same name in grbl
  to define dummy registers

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef config_h_sim
#include "../config.h"
#include <inttypes.h>

enum {
  SIM_A, SIM_B, SIM_C, SIM_D, SIM_E,
  //  SIM_F, SIM_G, SIM_H, SIM_I, SIM_J,  //for mega
  SIM_PORT_COUNT
};

struct sim_vars {
  uint8_t ddr[SIM_PORT_COUNT];
  uint8_t port[SIM_PORT_COUNT];
  uint8_t pin[SIM_PORT_COUNT];
  uint8_t pcmsk[3]; //pin change interrupt register
};
extern struct sim_vars sim;


#endif
