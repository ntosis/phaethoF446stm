#include "QueuesStructs.h"
#include "heatingSysQ.h"

/* define Queue to component*/
QueueHandle_t xQueueHeatingSys;

HeatingSysQ xMessageHeatingSys = {0,0,0,0,0,20,0,0,0,0};

void initheatingSysQueue(void) {

    xQueueHeatingSys = xQueueCreate( 2 , sizeof(HeatingSysQ) );
}

void heatingSysQueueSend(void) {

xQueueSend( xQueueHeatingSys, ( void * ) &xMessageHeatingSys, ( TickType_t ) 10 );

}
