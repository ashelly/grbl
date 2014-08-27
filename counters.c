/*
  counters.c - code pertaining to encoders and other counting methods
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
  
#include "system.h"
#include "counters.h"


counters_t counters = {{0}};

// Counters pin initialization routine.
void counters_init() 
{
  //encoders and feedback //TODO: move to new file
#ifdef KEYME_BOARD
  FDBK_DDR &= ~(FDBK_MASK); // Configure as input pins
  FDBK_PORT |= FDBK_MASK;   // Enable internal pull-up resistors. Normal high operation. //TODO test
#endif
  /*
  FDBK_PCMSK |= FDBK_MASK;  // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << FDBK_INT);   // Enable Pin Change Interrupt
  */
}

// Returns the counters pin state. Triggered = true. Called by gcode parser and counters state monitor.
uint8_t counters_reset(uint8_t axis)
{
  return counters.counts[axis]=0;
}


// Returns the counters pin state. Triggered = true. Called by gcode parser and counters state monitor.
uint8_t counters_get_count(uint8_t axis)
{
  return counters.counts[axis];
}


// Monitors counters pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void counters_state_monitor()
{
  //TODO: 
}

/* TODO: implement encoder counter - maybe needs debounce/
ISR(FDBK_INT_vect) {
  counters.state = FDBK_PIN&FDBK_MASK;
}
*/
