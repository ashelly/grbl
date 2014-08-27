/*
  probe.h - code pertaining to encoders and pulse counting
  Part of Grbl

  Copyright (c) 2014 Adam Shelly

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
  
#ifndef counters_h
#define counters_h 

typedef struct  counters {
  int16_t counts[N_AXIS];
} counters_t;

extern counters_t counters;

// Counters pin initialization routine.
void counters_init();

// Returns counters pin state.
uint8_t counters_get_state();

// Returns counts for a given axis
uint8_t counters_get_count(uint8_t axis);

uint8_t counters_reset(uint8_t axis);

// Monitors counters pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void counters_state_monitor();

#endif
