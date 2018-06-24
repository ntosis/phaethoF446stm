#include "QueuesStructs.h"
#include "RtrEncdBtnQ.h"

/* define Queue to component*/
QueueHandle_t xQueueRtrEncdBtn;
/* Global struct */
RtrEncdBtnQ xMessageRtrEncdBtn = {0,0,0,0,0,0,0,0,0,0};

/* Imported Signals*/
HeatingSysQ RxedMessageToRtr;

void initRtrEncdBtnQueue(void) {

    xQueueRtrEncdBtn = xQueueCreate( 2 , sizeof(RtrEncdBtnQ) );
}

void rtrEncdBtnQueueSend(void) {
//xMessageHeatingSys.SOLLtemperature++;

xQueueSend( xQueueRtrEncdBtn, ( void * ) &xMessageRtrEncdBtn, ( TickType_t ) 10 );

}

void rtrEncdBtnQueueRead(void) {

    if(xQueuePeek( xQueueHeatingSys, &(RxedMessageToRtr ), ( TickType_t ) 10 )) {

	}

    else {

	   //error
	}
}
