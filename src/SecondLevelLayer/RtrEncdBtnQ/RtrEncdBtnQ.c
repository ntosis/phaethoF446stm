#include "RtrEncdBtnQ.h"
#include "QueuesStructs.h"
/* define Queue to component*/
QueueHandle_t xQueueRtrEncdBtn;
/* Global struct */
RtrEncdBtnQ xMessageRtrEncdBtn;


void rtrEncdBtnQueueSend(void) {
//xMessageHeatingSys.SOLLtemperature++;

xQueueSend( xQueueRtrEncdBtn, ( void * ) &xMessageRtrEncdBtn, ( TickType_t ) 10 );

}
