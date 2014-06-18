/*
  serial.c - replacement for the modul of the same name in grbl
    Make sure the simulator reads from stdin and writes to stdout.
    Also print info about the last buffered block.

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

#include "../serial.h"
#include <stdio.h>
#include "simulator.h"
#include <stdio.h>

#define serial_read orig_serial_read
#define serial_write orig_serial_write
#include "../serial.c"
#include "kbhit.h"
#undef serial_read
#undef serial_write

#define dprintf 

void serial_bg_loop(){
  //  printf("UCSR0B is %x\n",UCSR0B);
  while (UCSR0B & (1<<UDRIE0)){
	 interrupt_SERIAL_UDRE();
	 printf("%c",UDR0);
  }
}

void serial_write(uint8_t data) {

  /*
   uint8_t reset_flag = sys.execute & EXEC_RESET;
  //use reset force break if buffer fills, since we don't really have background processs emptying it
  sys.execute &= EXEC_RESET; 
  */

  dprintf("\n1: %x %x\n",tx_buffer_head+1,tx_buffer_tail);
  serial_bg_loop();
  dprintf("\n2: %x %x\n",tx_buffer_head+1,tx_buffer_tail);
  orig_serial_write(data);
  dprintf("\n3: %x %x\n",tx_buffer_head+1,tx_buffer_tail);
  serial_bg_loop();
  dprintf("\n4: %x %x\n",tx_buffer_head+1,tx_buffer_tail);

  printBlock();
  if(print_comment && data!='\n' && data!='\r') {
	  fprintf(block_out_file, "# ");
	  print_comment= 0;
  }
  if(data=='\n' || data=='\r')
	  print_comment= 1;

  fprintf(block_out_file, "%c", data);

  // Indicate the end of processing a command. See simulator.c for details
  runtime_second_call= 0;
}


uint8_t serial_read() {
  //int c;
  //  if((c = fgetc(stdin)) != EOF) {
  if (kbhit()) {
	 UDR0 = getchar();
	 interrupt_SERIAL_RX();
  }
    
  return orig_serial_read(); // SERIAL_NO_DATA;
}

