#include "QueuesStructs.h"
#include "ctrlSystemQ.h"
#include "heatingSysQ.h"

/* define Queue to component*/
QueueHandle_t xQueueCtrlSubsystem;

ctrlSystemQueue xMessageCtrlSubsystem = {0,0,0,0};

HeatingSysQ pxRxedMessage;

void initctrlSystemQueue(void) {

    xQueueCtrlSubsystem = xQueueCreate( 2 , sizeof(ctrlSystemQueue) );
}

void ctrlSystemQueueSend(void) {
//xMessageCtrlSubsystem.SOLLtemperature++;
xQueueSend( xQueueCtrlSubsystem, ( void * ) &xMessageCtrlSubsystem, ( TickType_t ) 10 );
}


void ctrlSystemQueueRead(void) {

    if(xQueuePeek( xQueueHeatingSys, &(pxRxedMessage ), ( TickType_t ) 10 )) {

	}

    else {

	   //error
	}
}
