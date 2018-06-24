/*
 * GUIQ.h
 *
 *  Created on: Jun 21, 2018
 *      Author: unix
 */

#ifndef INCLUDE_GUI_GUIQ_GUIQ_H_
#define INCLUDE_GUI_GUIQ_GUIQ_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "hardware_init.h"

void GUIQueueSend(void);
void initGUIQueue(void);
void GUIQueueRead(void);

/* declare global Queue instance*/
extern QueueHandle_t xQueueGUI;

#define stateOfProgram xMessageGUI.stateOfProgram
#define selectProgram xMessageGUI.selectProgram

/*Imported Signals*/
extern HeatingSysQ rxMessageHeatingtoGui;

#define SOLLtemperature rxMessageHeatingtoGui.SOLLtemperature

#endif /* INCLUDE_GUI_GUIQ_GUIQ_H_ */
