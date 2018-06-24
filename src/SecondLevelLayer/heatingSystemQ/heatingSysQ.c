#include "QueuesStructs.h"
#include "heatingSysQ.h"
/* imported interface include */
#include "GUIQ.h"

/* define Queue to component*/
QueueHandle_t xQueueHeatingSys;

HeatingSysQ xMessageHeatingSys = {0,0,0,0,0,20,0,0,0,0};

GUIQ RxedMessage;
RtrEncdBtnQ RxedMessageRtr;

void initheatingSysQueue(void) {

    xQueueHeatingSys = xQueueCreate( 2 , sizeof(HeatingSysQ) );
}

void heatingSysQueueSend(void) {

xQueueSend( xQueueHeatingSys, ( void * ) &xMessageHeatingSys, ( TickType_t ) 10 );

}

void heatingSysQueueRead(void) {

    if(xQueuePeek( xQueueRtrEncdBtn, &(RxedMessageRtr ), ( TickType_t ) 10 )) {

	}

    else {

	   //error
	}

    if(xQueuePeek( xQueueGUI, &(RxedMessage ), ( TickType_t ) 10 )) {

	}

    else {

	   //error
	}


}
