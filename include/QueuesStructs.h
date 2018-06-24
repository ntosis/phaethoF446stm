/*
 * QueuesStructs.h
 *
 *  Created on: Jun 18, 2018
 *      Author: unix
 */

#ifndef INCLUDE_QUEUESSTRUCTS_H_
#define INCLUDE_QUEUESSTRUCTS_H_

#include "hardware_init.h"



typedef struct
 {
    bool isCoolingOn;                 /* '<S5>/Compare' */
    bool selectProgram;               /* '<S2>/XOR1' */
    bool stateOfProgram;              /* '<S4>/XOR1' */
    bool isHeatingOn;                 /* '<S3>/NOT' */
 }ctrlSystemQueue ;

typedef struct  {
    bool autoProgramSelected;
    bool manualProgramSelected;
    bool autoProgramTimeEnabled;
    bool stateOfHeatingSystem;
    bool smartCntFlag;
    int16_t SOLLtemperature;
    uint8_t smartCntDown;
    uint8_t smartCntUp;
    int8_t up;
    int8_t Temperature;
 }HeatingSysQ ;


typedef struct {
    bool clicked;
    bool doubleClicked;
    bool onStateofProgram;
    uint8_t holdCnt;
    //step between Heating/Cooling
    uint8_t smartCntUp;
    uint8_t smartCntDown;
    uint8_t smartCntFlag;
    uint8_t signalButton;
    int8_t up;
    bool TurnDetected;
 }RtrEncdBtnQ ;

typedef struct {
    bool stateOfProgram;
    bool selectProgram;
} GUIQ;

extern HeatingSysQ xMessageHeatingSys;
extern ctrlSystemQueue xMessageCtrlSubsystem;
extern RtrEncdBtnQ xMessageRtrEncdBtn;
extern GUIQ xMessageGUI;

extern QueueHandle_t xQueueRtrEncdBtn;
extern QueueHandle_t xQueueHeatingSys;
extern QueueHandle_t xQueueCtrlSubsystem;
extern QueueHandle_t xQueueGUI;

#endif /* INCLUDE_QUEUESSTRUCTS_H_ */
