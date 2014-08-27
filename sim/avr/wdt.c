#include "wdt.h"
#include "io.h"
#include "interrupt.h"


static uint32_t wdt_set_counts(void) {
  uint32_t cycles = 2048;
  uint8_t prescale_sel = io.wdtcsr&0x7; //put together 4 bits of prescaler
  prescale_sel|= (io.wdtcsr&(1<<WDP3))>>2;
  if (prescale_sel>9) prescale_sel=9;
  while (prescale_sel--) cycles*=2;
  return cycles;
}


volatile uint8_t* watchdog_sim() {
  const uint8_t wd_protect = 0x2F;
  static uint32_t counts=0;
  static uint8_t last_wdctsr=0;
  static uint8_t wdce_cycles=0;


  //wdce is cleared 4 cpu cycles after setting it
  if (wdce_cycles){
    wdce_cycles--;
  }
  else {
    if (io.wdtcsr&(1<<WDCE)) {
      wdce_cycles = 4;
    }
    else {
      io.wdtcsr&=~(1<<WDCE);
    }
  }

  //wdce must be set to update prescaler or Enable.
  if ((io.wdtcsr^last_wdctsr)&wd_protect) {
    if (!wdce_cycles) { //restore protected bits if not.
        io.wdtcsr&=~wd_protect;
        io.wdtcsr|=last_wdctsr&wd_protect;
    }
    else { //update prescaler count
      counts = wdt_set_counts();
    }
  }

  last_wdctsr = io.wdtcsr;

  //clear wdif by writing 1 to it.
  if (io.wdtcsr&(1<<WDIF)) {
    io.wdtcsr&=~(1<<WDIF);
  }

  //WDE is overridden by WDRF (both are bit 3)
  io.wdtcsr |= io.mcusr&(1<<WDRF);

  //do timer calcs
  if (counts) {
    if (!counts--) {
      //if interrupt mode
      if ((io.wdtcsr&(1<<WDIE))) {
        io.wdtcsr|=(1<<WDIF);
        //interrupt mode: call ISR if registered
        if (io.sreg&SEI) {
          if (wdt_vect) wdt_vect();
          //interrupt call clears WDIF & WDIE
          io.wdtcsr&=~((1<<WDIF)|(1<<WDIE));
        }
      }
      //if reset mode
      else if (io.wdtcsr&(1<<WDE)) {
        MCUSR|=(1<<WDRF);
        //TODO: reset!
      }
      //reset clock after timeout
      counts = wdt_set_counts();
    }
  }
  return &(io.wdtcsr);  
}

//TODO: on startup MCUSR|=(1<<PORF) //power on reset.
