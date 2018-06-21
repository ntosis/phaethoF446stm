/*
 * ctrlSystemQ.hh
 *
 *  Created on: Jun 13, 2018
 *      Author: unix
 */

#ifndef CTRLSYSTEMQ_HH_
#define CTRLSYSTEMQ_HH_

#include "FreeRTOS.h"
#include "queue.h"
#include "hardware_init.h"

void initctrlSystemQueue(void);
void ctrlSystemQueueSend(void);
void ctrlSystemQueueRead(void);

/* declare global Queue instance*/
extern QueueHandle_t xQueueCtrlSubsystem;
/* declare extern Queues to read */
extern QueueHandle_t xQueueHeatingSys;



/*define interfaces for the Matlab component (Always after the struct)*/
#define isCoolingOn xMessageCtrlSubsystem.isCoolingOn
#define selectProgram xMessageCtrlSubsystem.selectProgram
#define stateOfProgram xMessageCtrlSubsystem.stateOfProgram
#define isHeatingOn xMessageCtrlSubsystem.isHeatingOn

/* Import Signals */
extern HeatingSysQ pxRxedMessage;

#define autoProgramSelected pxRxedMessage.autoProgramSelected
#define manualProgramSelected pxRxedMessage.manualProgramSelected
#define autoProgramTimeEnabled pxRxedMessage.autoProgramTimeEnabled
#define stateOfHeatingSystem pxRxedMessage.stateOfHeatingSystem
#define smartCntFlag pxRxedMessage.smartCntFlag

#define SOLLtemperature pxRxedMessage.SOLLtemperature
#define smartCntDown pxRxedMessage.smartCntDown
#define smartCntUp pxRxedMessage.smartCntUp
#define up pxRxedMessage.up
#define Temperature pxRxedMessage.Temperature
#endif /* CTRLSYSTEMQ_HH_ */
