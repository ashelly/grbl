#include <stdlib.h>
#include "io.h"

// dummy register variables
volatile io_sim_t io={{0}};


static port_monitor_fp io_portfuncs[SIM_PORT_COUNT] = {0}; 

volatile uint8_t *io_port(uint8_t portid){
  port_monitor_fp callback = io_portfuncs[portid];
  if (callback) { 
    io_portfuncs[portid] = NULL;  //hide callback to prevent recursion
    callback(io.port[portid]);
    io_portfuncs[portid]= callback;
  }
  return &io.port[portid];
} 


void io_sim_init(io_sim_monitor_t* hooks){
  int i;
  //assert hooks!=NULL;
  while (hooks->ddr_addr!=0) {
    for (i=0;i<SIM_PORT_COUNT;i++) {
      if (hooks->ddr_addr == &(io.ddr[i])) {
        io_portfuncs[i]=hooks->access_func;
        break;
      }
    }
    hooks++;
  }
}
