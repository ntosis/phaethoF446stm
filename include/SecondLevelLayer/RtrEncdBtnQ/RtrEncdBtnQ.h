/*
 * RtrEncdBtnQ.h
 *
 *  Created on: Jun 14, 2018
 *      Author: unix
 */

#ifndef RTRENCDBTNQ_H_
#define RTRENCDBTNQ_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "hardware_init.h"

void rtrEncdBtnQueueSend(void);
/* declare global Queue instance*/
extern QueueHandle_t xQueueRtrEncdBtn;




#endif /* RTRENCDBTNQ_H_ */
