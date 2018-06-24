/*
 * GUIQ.c
 *
 *  Created on: Jun 21, 2018
 *      Author: unix
 */

#include "QueuesStructs.h"
#include "GUIQ.h"

/* define Queue to component*/
QueueHandle_t xQueueGUI;

GUIQ xMessageGUI = {0,0};

/* Imported signals*/
HeatingSysQ rxMessageHeatingtoGui;


void initGUIQueue(void) {

    xQueueGUI = xQueueCreate( 2 , sizeof(GUIQ) );
}

void GUIQSend(void) {

xQueueSend( xQueueGUI, ( void * ) &xMessageGUI, ( TickType_t ) 10 );

}

void GUIQueueRead(void) {

    if(xQueuePeek( xQueueHeatingSys, &(rxMessageHeatingtoGui ), ( TickType_t ) 10 )) {

	}

    else {

	   //error
	}
}
