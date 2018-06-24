#ifndef _RTRENCDBTN_H_
#define _RTRENCDBTN_H_

#include "hardware_init.h"
#include "eeprom_calib.h"
//#include <avr/interrupt.h>
//#include "timerInterrupt.h"
//#include "ds1307.h"

/*
 * The ports for the pins are defined in hardware_init.h
 * The D2 D3 D4 are used for the rotary encoder
 */

void initRtrEncoder();
void readButton(const portTickType now);
void readEncoder(void);
inline void checkHoldButton(void);
void checkStruct();
void resetSmartCnt(void);
uint8_t returnStateofProgram();
void smartChangeBtwnHeatCool(void);
void resetFlag(uint8_t *flag, uint8_t state);


typedef struct {
portTickType timeOfClick;
bool st_clicked;
} click;

static click ArrayOfClicks[5];

extern volatile uint8_t signalButton; // not to be optimized
static volatile uint8_t pnt=0;
uint8_t holdCnt;


void inline checkHoldButton(void) {

	if(!__READ(RtrEnc_SWTCH)) {  			//if button is logical LOW

		holdCnt++;

	}

	else if(__READ(RtrEnc_SWTCH)) {

		holdCnt=0;

	}

	if(holdCnt>4) setTimeLoop(); //this task is called every 800ms, for 3 sec we calculate a value ~ 4
}

#endif
