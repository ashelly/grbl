#define WDTCSR (*watchdog_sim()) 
#define WDP0 0
#define WDP1 1
#define WDP2 2
#define WDE  3
#define WDCE 4
#define WDP3 5
#define WDIE 6
#define WDIF 7


volatile uint8_t* watchdog_sim();
